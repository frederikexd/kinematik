#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Regenerate the electrical benchmark references (docs/ELECTRICAL_BENCHMARK_PLAN.md) with the reference tools.

  kicad     KiCad's own reader (pcbnew Python API) on KiCad's demo boards  -> kicad_boards.json
  spice     ngspice: precharge, pack along a lap, net resistance             -> spice_*.json
  atlc      atlc field solver: microstrip and differential impedance         -> atlc_impedance.json

Needs KiCad 7 (python3-pcbnew), ngspice and atlc installed; the `kicad` step must run under the Python that ships
pcbnew (e.g. /usr/bin/python3). The recorded JSON is what tests/test_electrical_benchmark.py checks against.
"""
import argparse
import glob
import json
import os
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "benchmarks" / "electrical"
DEMOS = "/usr/share/kicad/demos"


def kicad():
    import pcbnew
    boards = {}
    for path in sorted(glob.glob(f"{DEMOS}/**/*.kicad_pcb", recursive=True)):
        b = pcbnew.LoadBoard(path)
        nm = 1e-6
        straight = arcs = 0
        vias = []
        per_net: dict = {}
        for t in b.GetTracks():
            ty = t.Type()
            if ty == pcbnew.PCB_VIA_T:
                vias.append(round(t.GetDrillValue() * nm, 4))
                continue
            if ty == pcbnew.PCB_ARC_T:
                arcs += 1
            else:
                straight += 1
            d = per_net.setdefault(t.GetNetname(), {"length_mm": 0.0, "widths_mm": set()})
            d["length_mm"] += t.GetLength() * nm
            d["widths_mm"].add(round(t.GetWidth() * nm, 4))
        boards[os.path.relpath(path, DEMOS)] = {
            "straight_tracks": straight, "arc_tracks": arcs, "via_drills_mm": sorted(vias),
            "pads": len(b.GetPads()), "footprints": len(b.GetFootprints()),
            "nets": {k: {"length_mm": round(v["length_mm"], 6), "widths_mm": sorted(v["widths_mm"])}
                     for k, v in sorted(per_net.items())}}
    _write("kicad_boards.json", {"tool": f"KiCad {pcbnew.Version()} (pcbnew Python API)", "boards": boards})


PRECHARGE_CASES = (  # (name, V, R ohm, C F, t_switch s or None)
    ("600V_2k_500uF", 600.0, 2000.0, 500e-6, None),
    ("400V_1k_1.2mF", 400.0, 1000.0, 1.2e-3, None),
    ("300V_5k_200uF", 300.0, 5000.0, 200e-6, None),
    ("600V_2k_500uF_short_at_2.5s", 600.0, 2000.0, 500e-6, 2.5),
)


def _ngspice(netlist: str, wrfile: str | None = None) -> tuple:
    """Run ngspice in batch mode on a netlist string; return (stdout, rows of the wrdata file)."""
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        cir = Path(d) / "c.cir"
        cir.write_text(netlist.replace("@OUT@", str(Path(d) / "out.txt")))
        res = subprocess.run(["ngspice", "-b", str(cir)], capture_output=True, text=True, timeout=600)
        rows = []
        if wrfile is not None and (Path(d) / "out.txt").exists():
            for line in (Path(d) / "out.txt").read_text().split("\n"):
                parts = line.split()
                if parts:
                    rows.append([float(x) for x in parts])
        return res.stdout + res.stderr, rows


def _meas(out: str, name: str) -> float:
    for line in out.splitlines():
        t = line.split()
        if t and t[0].lower() == name.lower() and "=" in t:
            return float(t[t.index("=") + 1])
    raise ValueError(f"{name} not measured:\n{out[-2000:]}")


def spice_precharge():
    cases = {}
    for name, V, R, C, tsw in PRECHARGE_CASES:
        tau = R * C
        t_end = 2 * tsw if tsw else 10 * tau
        times = [t_end * k / 50 for k in range(1, 51)]
        sw = ""
        if tsw:
            # the resistor is shorted at tsw by a switch whose on-resistance is R/1000: the residual time constant
            # tau/1000 KinematiK models
            sw = (f"Vctl ctl 0 PWL(0 0 {tsw} 0 {tsw + 1e-9} 1)\n"
                  f"S1 in cap ctl 0 sw\n.model sw sw(vt=0.5 ron={R / 1000} roff=1e12)\n")
        net = f"""precharge {name}
V1 in 0 DC {V}
R1 in cap {R}
C1 cap 0 {C} IC=0
{sw}.options reltol=1e-7 abstol=1e-12 vntol=1e-9
.tran {t_end / 20000} {t_end} 0 {t_end / 20000} uic
.meas tran t90 WHEN v(cap)={0.9 * V} RISE=1
.meas tran t95 WHEN v(cap)={0.95 * V} RISE=1
.meas tran ipk MAX par('(v(in)-v(cap))/{R}') from=0 to={tsw or t_end}
.meas tran er INTEG par('(v(in)-v(cap))*(v(in)-v(cap))/{R}') from=0 to={tsw or 10 * tau}
.control
run
"""
        net += "".join(f"meas tran vc{k} FIND v(cap) AT={t}\n" for k, t in enumerate(times))
        net += ".endc\n.end\n"
        out, _ = _ngspice(net)
        cases[name] = {"V": V, "R": R, "C": C, "t_switch_s": tsw, "t_end_s": t_end, "times_s": times,
                       "v_cap_v": [_meas(out, f"vc{k}") for k in range(len(times))],
                       "t90_s": _meas(out, "t90"), "t95_s": _meas(out, "t95"), "i_peak_a": _meas(out, "ipk"),
                       "e_resistor_j": _meas(out, "er")}
    return cases


PACK = {"ocv_soc": ([0.0, 0.2, 0.8, 1.0], [480.0, 540.0, 590.0, 610.0]), "soc0": 0.95, "capacity_ah": 14.0,
        "r_pack_ohm": 0.25, "dt_s": 0.05}


def lap_power():
    """Terminal power (W) of one simulated lap of KinematiK's representative autocross, at 0.05 s: the input both
    KinematiK and SPICE get. Regen at 60 % up to 40 kW at the wheels, 1.5 kW auxiliaries."""
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from suspension.dynamics import VehicleDynamics, VehicleParams
    from suspension.ev_electrical_check import traction_power_w
    from suspension.laptime import Powertrain, default_autocross, simulate_lap
    lap = simulate_lap(VehicleDynamics(VehicleParams()), default_autocross(), Powertrain(power_kw=80.0))
    s, v = np.asarray(lap.s, float), np.maximum(np.asarray(lap.v, float), 0.5)
    t = np.concatenate([[0.0], np.cumsum(np.diff(s) / (0.5 * (v[1:] + v[:-1])))])
    tt = np.arange(0.0, t[-1], PACK["dt_s"])
    vv = np.interp(tt, t, v)
    a = np.gradient(vv, tt)
    _f, _pw, pp = traction_power_w(vv, a, regen_eff=0.6, regen_cap_w=40e3, aux_power_w=1500.0)
    return [round(float(x), 6) for x in tt], [round(float(x), 3) for x in pp]


def spice_pack():
    t, p = lap_power()
    soc_pts, v_pts = PACK["ocv_soc"]
    pwl_ocv = ", ".join(f"{s_}, {v_}" for s_, v_ in zip(soc_pts, v_pts))
    pwl_p = " ".join(f"{a} {b}" for a, b in zip(t, p))
    n = len(t)
    net = f"""pack along a lap
* state of charge: a 1 F capacitor whose voltage is the charge fraction, drained by I / (3600 Ah)
Csoc soc 0 1 IC={PACK["soc0"]}
Bsoc soc 0 I=i(Vsense)/{3600.0 * PACK["capacity_ah"]}
* open-circuit voltage from the same table, read linearly
Bocv ocv 0 V=pwl(v(soc), {pwl_ocv})
Rpack ocv term {PACK["r_pack_ohm"]}
Vsense term load 0
* the lap's terminal power, drawn as a constant-power load (negative = regen)
Vp pw 0 PWL({pwl_p})
Bload load 0 I=v(pw)/v(load)
.options reltol=1e-6 abstol=1e-9 vntol=1e-7
.tran {PACK["dt_s"] / 10} {t[-1]} 0 {PACK["dt_s"] / 10} uic
.control
run
linearize v(term) i(Vsense) v(soc)
"""
    net += "wrdata @OUT@ v(term) i(Vsense) v(soc)\n.endc\n.end\n"
    out, rows = _ngspice(net, wrfile="x")
    import numpy as np
    arr = np.array(rows)
    ts, vt, ii, sc = arr[:, 0], arr[:, 1], arr[:, 3], arr[:, 5]
    return {"inputs": {**PACK, "t_s": t, "p_terminal_w": p},
            "v_terminal_v": [float(x) for x in np.interp(t, ts, vt)],
            "i_a": [float(x) for x in np.interp(t, ts, ii)],
            "soc": [float(x) for x in np.interp(t, ts, sc)], "n": n}


def spice():
    import subprocess
    ver = subprocess.run(["ngspice", "-v"], capture_output=True, text=True).stdout.strip().splitlines()
    tool = " ".join(ver[:2]).strip() if ver else "ngspice"
    _write("spice_precharge.json", {"tool": tool, "cases": spice_precharge()})
    _write("spice_pack_lap.json", {"tool": tool, **spice_pack()})
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        print("pcbnew not importable here: run again under KiCad's Python for the net-resistance reference")
        return
    _write("spice_nets.json", {"tool": tool, "temp_c": NET_TEMP_C, "nets": spice_nets()})


RHO_CU_20C, ALPHA_CU, OZ_TO_UM, VIA_PLATING_MM, NET_TEMP_C = 1.724e-8, 3.93e-3, 34.8, 0.025, 60.0


def _net_netlist(board, net: str, pair, copper_oz: float, thickness_mm: float):
    """A SPICE netlist of one net's copper built only from KiCad's geometry and KiCad's hit tests: one resistor per
    piece of track between connection points, a star of two half-barrels per via, and a node per pad. Two items are
    connected where their copper overlaps, KiCad's own rule: a track end's copper disc (half its width) on a pad,
    a via or another track, and a via's ring on a pad. Returns (netlist, pad A node, pad B node) or None if a pad
    of the pair isn't on the board."""
    import pcbnew
    rho = RHO_CU_20C * (1 + ALPHA_CU * (NET_TEMP_C - 20.0))
    t_m = copper_oz * OZ_TO_UM * 1e-6
    tracks = [t for t in board.GetTracks() if t.GetNetname() == net and t.Type() != pcbnew.PCB_VIA_T]
    vias = [t for t in board.GetTracks() if t.GetNetname() == net and t.Type() == pcbnew.PCB_VIA_T]
    pads = [p for p in board.GetPads() if p.GetNetname() == net]
    names: dict = {}

    def node(key):
        return names.setdefault(key, f"n{len(names) + 1}")
    lines = []
    k = 0

    def res(a, b, ohm):
        nonlocal k
        k += 1
        lines.append(f"R{k} {a} {b} {max(ohm, 1e-9):.12g}")
    ends = [(t.GetLayer(), (t.GetStart().x, t.GetStart().y)) for t in tracks] + \
           [(t.GetLayer(), (t.GetEnd().x, t.GetEnd().y)) for t in tracks]
    halfw: dict = {}
    for t in tracks:
        for q in (t.GetStart(), t.GetEnd()):
            key = (t.GetLayer(), (q.x, q.y))
            halfw[key] = max(halfw.get(key, 0), t.GetWidth() // 2)
    for t in tracks:
        ly = t.GetLayer()
        a, b = t.GetStart(), t.GetEnd()
        w = t.GetWidth() * 1e-9
        if t.Type() == pcbnew.PCB_ARC_T:
            res(node((ly, (a.x, a.y))), node((ly, (b.x, b.y))), rho * t.GetLength() * 1e-9 / (w * t_m))
            continue
        ax, ay, bx, by = a.x, a.y, b.x, b.y
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        cuts = {0.0: (ax, ay), 1.0: (bx, by)}
        others = [pt for lyr, pt in ends if lyr == ly] + \
                 [(v.GetPosition().x, v.GetPosition().y) for v in vias if v.IsOnLayer(ly)]
        for pt in others:                         # T-junctions and vias landing on this track's copper
            hw = halfw.get((ly, pt), 0)
            if pt in ((ax, ay), (bx, by)) or L2 == 0 or \
                    not t.HitTest(pcbnew.VECTOR2I(int(pt[0]), int(pt[1])), int(hw)):
                continue
            u = min(max(((pt[0] - ax) * (bx - ax) + (pt[1] - ay) * (by - ay)) / L2, 0.0), 1.0)
            cuts[u] = (ax + u * (bx - ax), ay + u * (by - ay))
            res(node((ly, cuts[u])), node((ly, pt)), 1e-9)
        us = sorted(cuts)
        Ltot = t.GetLength() * 1e-9
        for u0, u1 in zip(us, us[1:]):
            res(node((ly, cuts[u0])), node((ly, cuts[u1])), rho * Ltot * (u1 - u0) / (w * t_m))
    for i, v in enumerate(vias):
        hub = node(("via", i))
        r = rho * thickness_mm * 1e-3 / (3.141592653589793 * v.GetDrillValue() * 1e-9 * VIA_PLATING_MM * 1e-3)
        for key in list(names):
            if len(key) != 2 or not isinstance(key[0], int):
                continue
            ly, pt = key
            if v.IsOnLayer(ly) and v.HitTest(pcbnew.VECTOR2I(int(pt[0]), int(pt[1])), int(halfw.get(key, 0))):
                res(hub, names[(ly, pt)], r / 2.0)
    pad_node = {}
    for p in pads:
        key = ("pad", p.GetParent().GetReference(), p.GetNumber())
        pn = node(key)
        pad_node[f"{key[1]}.{key[2]}"] = pn
        for key in list(names):
            if len(key) != 2 or not isinstance(key[0], int):
                continue
            ly, pt = key
            if p.IsOnLayer(ly) and p.HitTest(pcbnew.VECTOR2I(int(pt[0]), int(pt[1])), int(halfw.get(key, 0))):
                res(pn, names[(ly, pt)], 1e-9)
        for i, v in enumerate(vias):             # a via whose copper ring overlaps the pad, on a shared layer
            shared = any(p.IsOnLayer(ly) and v.IsOnLayer(ly) for ly in (pcbnew.F_Cu, pcbnew.B_Cu))
            if shared and p.HitTest(v.GetPosition(), int(v.GetWidth() // 2)):
                r = rho * thickness_mm * 1e-3 / (3.141592653589793 * v.GetDrillValue() * 1e-9 * VIA_PLATING_MM * 1e-3)
                res(pn, node(("via", i)), r / 2.0)
    if pair[0] not in pad_node or pair[1] not in pad_node:
        return None
    a, b = pad_node[pair[0]], pad_node[pair[1]]
    leak = "".join(f"Rleak{i} {n} 0 1e15\n" for i, n in enumerate(sorted(set(names.values()))))
    net_txt = (f"net {net}\n" + "\n".join(lines) + "\n" + leak + f"Vref {b} 0 0\nI1 0 {a} DC 1\n"
               ".options gmin=1e-18\n.op\n.control\nrun\n" + f"print v({a})\n.endc\n.end\n")
    return net_txt, a, b


def spice_nets():
    import pcbnew
    pairs = json.loads((OUT / "net_pairs.json").read_text(encoding="utf-8"))["pairs"]
    boards: dict = {}
    out = []
    for q in pairs:
        if q["board"] not in boards:
            boards[q["board"]] = pcbnew.LoadBoard(f"{DEMOS}/{q['board']}")
        built = _net_netlist(boards[q["board"]], q["net"], q["pair"], q["copper_oz"], q["board_thickness_mm"])
        if built is None:
            out.append({**q, "spice_ohm": None, "note": "pad not found by KiCad"})
            continue
        txt, a, _b = built
        so, _ = _ngspice(txt)
        val = None
        for line in so.splitlines():
            if line.strip().lower().startswith(f"v({a})"):
                val = float(line.split("=")[1])
        out.append({**q, "spice_ohm": val})
    return out


RED, GREEN, BLUE, WHITE, FR4 = (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255), (0xCC, 0x00, 0xFF)


def _atlc(img, er: float) -> dict:
    """Run atlc on an RGB image (numpy uint8 HxWx3; red = +1 V, blue = −1 V, green = ground, FR4 colour = the
    dielectric of permittivity er, white = air). Returns the impedances atlc prints (Ω)."""
    import re
    import subprocess
    import tempfile
    from PIL import Image
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "x.bmp"
        Image.fromarray(img, "RGB").save(f)
        res = subprocess.run(["atlc", "-s", "-S", "-d", f"cc00ff={er}", str(f)], capture_output=True, text=True,
                             timeout=3600, cwd=d)
        out = res.stdout + res.stderr
    vals = {k: float(v) for k, v in re.findall(r"(Zo|Zodd|Zeven|Zdiff|Zcomm|Er)\s*=\s*([-\d.eE+]+)", out)}
    if not vals:
        raise RuntimeError(out[-2000:])
    return vals


def _box(h_px: int, w_px: int, fill=WHITE):
    import numpy as np
    img = np.zeros((h_px, w_px, 3), np.uint8)
    img[:] = fill
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = GREEN       # grounded enclosure
    return img


def stripline_check(er: float, b_px: int = 320, w_px: int = 160):
    """Centred zero-thickness stripline (1 px thick), plates b apart, side walls 5 b away: atlc against Cohn's
    exact Z0 = (30 π / √εr) K(k) / K(k'), k = sech(π w / 2 b)."""
    import math
    from scipy.special import ellipk
    W = 2 * 5 * b_px + w_px
    img = _box(b_px + 2, W, FR4 if er != 1.0 else WHITE)
    mid = 1 + b_px // 2
    x0 = (W - w_px) // 2
    img[mid, x0:x0 + w_px] = RED
    z = _atlc(img, er)["Zo"]
    k = 1.0 / math.cosh(math.pi * w_px / (2.0 * b_px))
    exact = 30.0 * math.pi / math.sqrt(er) * ellipk(k * k) / ellipk(1.0 - k * k)
    return {"er": er, "w_over_b": w_px / b_px, "atlc_ohm": z, "exact_ohm": exact,
            "error_pct": 100.0 * (z - exact) / exact}


PX_MM, H_MM, T_PX, ER = 0.005, 0.2, 7, 4.3            # 5 µm pixels: h = 40 px, 35 µm copper = 7 px
SE_W_OVER_H = (0.2, 0.5, 1.0, 1.5, 2.0)
DIFF_CASES = tuple((w, s_) for w in (0.5, 1.0) for s_ in (0.5, 1.0, 2.0))   # (w/h, s/h)


def _microstrip(w_px: int, s_px: int | None = None):
    """Microstrip cross-section: ground plane, an FR-4 slab h thick, one trace (or an edge-coupled pair s apart) on
    top, air above; grounded enclosure 15 h away on every side."""
    h_px = round(H_MM / PX_MM)
    margin = 15 * h_px
    span = w_px if s_px is None else 2 * w_px + s_px
    W = 2 * margin + span
    H = 2 + h_px + T_PX + margin
    img = _box(H, W)
    ground = H - 2                                     # last row inside the green border is the plane
    img[ground, :] = GREEN
    img[ground - h_px:ground, 1:-1] = FR4
    top = ground - h_px - T_PX
    x0 = margin
    img[top:top + T_PX, x0:x0 + w_px] = RED
    if s_px is not None:
        x1 = x0 + w_px + s_px
        img[top:top + T_PX, x1:x1 + w_px] = BLUE
    return img


def atlc_impedance():
    h_px = round(H_MM / PX_MM)
    se = []
    for r in SE_W_OVER_H:
        w = round(r * h_px)
        z = _atlc(_microstrip(w), ER)
        se.append({"w_over_h": r, "w_mm": w * PX_MM, "h_mm": H_MM, "t_mm": T_PX * PX_MM, "er": ER, "atlc_z0_ohm": z["Zo"]})
    diff = []
    for r, q in DIFF_CASES:
        w, sp = round(r * h_px), round(q * h_px)
        z = _atlc(_microstrip(w, sp), ER)
        diff.append({"w_over_h": r, "s_over_h": q, "w_mm": w * PX_MM, "s_mm": sp * PX_MM, "h_mm": H_MM,
                     "t_mm": T_PX * PX_MM, "er": ER, "atlc_zdiff_ohm": z.get("Zdiff"), "atlc_zodd_ohm": z.get("Zodd"),
                     "atlc_zeven_ohm": z.get("Zeven")})
    checks = [stripline_check(1.0), stripline_check(ER)]
    return {"stripline_checks": [{k: float(v) for k, v in c.items()} for c in checks], "single_ended": se,
            "differential": diff, "pixel_mm": PX_MM}


def atlc():
    import subprocess
    ver = subprocess.run(["atlc"], capture_output=True, text=True)
    first = (ver.stdout + ver.stderr).strip().splitlines()[0].split(":")[0]
    _write("atlc_impedance.json", {"tool": first, **atlc_impedance()})


def _write(name, obj):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(obj, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print("wrote", OUT / name)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["kicad", "spice", "atlc"])
    a = ap.parse_args(argv)
    {"kicad": kicad, "spice": spice, "atlc": atlc}[a.step]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
