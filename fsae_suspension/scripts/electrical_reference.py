#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Regenerate the electrical benchmark references (docs/ELECTRICAL_BENCHMARK_PLAN.md) with the reference tools.

  kicad     KiCad's own reader (pcbnew Python API) on KiCad's demo boards  -> kicad_boards.json
  spice     ngspice: precharge, pack along a lap; net resistance for the pairs in nets_round2.json
                                                                          -> spice_*.json, nets_round2.json
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


def kicad(root: str = DEMOS, out_name: str = "kicad_boards.json"):
    import pcbnew
    boards = {}
    skipped = {}
    for path in sorted(glob.glob(f"{root}/**/*.kicad_pcb", recursive=True)):
        try:
            b = pcbnew.LoadBoard(path)
        except Exception as e:                     # a newer file format than this KiCad reads
            skipped[os.path.relpath(path, root)] = str(e).splitlines()[0][:200]
            continue
        if b is None:
            skipped[os.path.relpath(path, root)] = "KiCad returned no board"
            continue
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
        boards[os.path.relpath(path, root)] = {
            "straight_tracks": straight, "arc_tracks": arcs, "via_drills_mm": sorted(vias),
            "pads": len(b.GetPads()), "footprints": len(b.GetFootprints()),
            "nets": {k: {"length_mm": round(v["length_mm"], 6), "widths_mm": sorted(v["widths_mm"])}
                     for k, v in sorted(per_net.items())}}
    rec = {"tool": f"KiCad {pcbnew.Version()} (pcbnew Python API)", "boards": boards}
    if skipped:
        rec["not_loaded"] = skipped
    _write(out_name, rec)


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
    rec = json.loads((OUT / "nets_round2.json").read_text(encoding="utf-8"))
    rec["nets"] = spice_nets()
    rec["spice_tool"], rec["temp_c"] = tool, NET_TEMP_C
    _write("nets_round2.json", rec)


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


def spice_nets(pairs_file: str = "nets_round2.json"):
    """SPICE resistance for every (board, net, pad pair) listed in pairs_file (KinematiK's worst pair per net)."""
    import pcbnew
    pairs = json.loads((OUT / pairs_file).read_text(encoding="utf-8"))["nets"]
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


def _write_bmp(path, img) -> None:
    """24-bit uncompressed BMP of an RGB uint8 array (HxWx3), written directly: KinematiK opens no image library
    (tests/test_image_safety.py)."""
    import struct
    import numpy as np
    h_, w_ = img.shape[:2]
    row = (w_ * 3 + 3) & ~3
    data = bytearray()
    for y in range(h_ - 1, -1, -1):                               # BMP rows run bottom-up, pixels as BGR
        line = np.ascontiguousarray(img[y, :, ::-1]).tobytes()
        data += line + b"\0" * (row - len(line))
    head = struct.pack("<2sIHHI", b"BM", 54 + len(data), 0, 0, 54)
    info = struct.pack("<IiiHHIIiiII", 40, w_, h_, 1, 24, 0, len(data), 2835, 2835, 0, 0)
    Path(path).write_bytes(head + info + bytes(data))


def _atlc(img, er: float) -> dict:
    """Run atlc on an RGB image (numpy uint8 HxWx3; red = +1 V, blue = −1 V, green = ground, FR4 colour = the
    dielectric of permittivity er, white = air). Returns the impedances atlc prints (Ω)."""
    import re
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "x.bmp"
        _write_bmp(f, img)
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


ROUND2_SE = tuple((er, r) for er in (3.66, 4.6) for r in (0.3, 0.8, 1.3, 1.8))
ROUND2_DIFF = ((4.6, 0.75, 0.75), (4.6, 1.25, 1.5))
ROUND2_H_MM = 0.15


def atlc_round2():
    """Round 2: geometries registered after round 1 and never used to choose a formula (plan addendum)."""
    global H_MM
    keep, H_MM = H_MM, ROUND2_H_MM
    try:
        h_px = round(H_MM / PX_MM)
        se = []
        for er, r in ROUND2_SE:
            w = round(r * h_px)
            se.append({"er": er, "w_over_h": r, "w_mm": w * PX_MM, "h_mm": H_MM, "t_mm": T_PX * PX_MM,
                       "atlc_z0_ohm": _atlc(_microstrip(w), er)["Zo"]})
        diff = []
        for er, r, q in ROUND2_DIFF:
            w, sp = round(r * h_px), round(q * h_px)
            z = _atlc(_microstrip(w, sp), er)
            diff.append({"er": er, "w_over_h": r, "s_over_h": q, "w_mm": w * PX_MM, "s_mm": sp * PX_MM, "h_mm": H_MM,
                         "t_mm": T_PX * PX_MM, "atlc_zdiff_ohm": z.get("Zdiff")})
        return {"single_ended": se, "differential": diff, "pixel_mm": PX_MM}
    finally:
        H_MM = keep


def atlc():
    import subprocess
    ver = subprocess.run(["atlc"], capture_output=True, text=True)
    first = (ver.stdout + ver.stderr).strip().splitlines()[0].split(":")[0]
    _write("atlc_impedance.json", {"tool": first, **atlc_impedance()})
    _write("atlc_impedance_round2.json", {"tool": first, **atlc_round2()})


def _cells_capsule(ax, ay, bx, by, half_w, h, ox: float = 0.0, oy: float = 0.0, ss: int = 0):
    """Grid cells (ix, iy) and copper coverage (0–1) of a track: a capsule from (ax, ay) to (bx, by) with
    half-width half_w (mm), on an h (mm) grid whose cell centres sit at ((ix + 0.5 + ox) h, (iy + 0.5 + oy) h).
    With ss > 0 the coverage is the area fraction from ss × ss samples per cell; with ss = 0 it comes from the
    signed distance of each cell centre to the capsule edge. A long track is rasterized in pieces along its length
    (each against the whole capsule), so a diagonal track costs its band of cells, not its bounding box."""
    import numpy as np
    dx, dy = bx - ax, by - ay
    L = float(np.hypot(dx, dy))
    L2 = dx * dx + dy * dy
    piece = max(0.1, 8 * h)
    npieces = max(1, int(np.ceil(L / piece)))
    out_i, out_j, out_c = [], [], []
    sub = (np.arange(ss) + 0.5) / ss if ss else None
    for k in range(npieces):
        f0, f1 = k / npieces, (k + 1) / npieces
        px0, py0, px1, py1 = ax + f0 * dx, ay + f0 * dy, ax + f1 * dx, ay + f1 * dy
        x0, x1 = min(px0, px1) - half_w - h, max(px0, px1) + half_w + h
        y0, y1 = min(py0, py1) - half_w - h, max(py0, py1) + half_w + h
        ix = np.arange(int(np.floor(x0 / h - ox)), int(np.ceil(x1 / h - ox)) + 1)
        iy = np.arange(int(np.floor(y0 / h - oy)), int(np.ceil(y1 / h - oy)) + 1)
        if ss:
            X, Y = np.meshgrid(((ix[:, None] + sub[None, :]).ravel() + ox) * h,
                               ((iy[:, None] + sub[None, :]).ravel() + oy) * h, indexing="ij")
        else:
            X, Y = np.meshgrid((ix + 0.5 + ox) * h, (iy + 0.5 + oy) * h, indexing="ij")
        u = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0.0, 1.0) if L2 > 0 else np.zeros_like(X)
        d = np.hypot(X - (ax + u * dx), Y - (ay + u * dy))
        if ss:
            c = (d <= half_w).reshape(len(ix), ss, len(iy), ss).mean(axis=(1, 3))
        else:
            c = np.clip((half_w - d) / h + 0.5, 0.0, 1.0)
        m = c > 0
        I, J = np.meshgrid(ix, iy, indexing="ij")
        out_i.append(I[m]); out_j.append(J[m]); out_c.append(c[m])
    return np.concatenate(out_i), np.concatenate(out_j), np.concatenate(out_c)


def _cells_polygon(pts, h, ss: int = 4, ox: float = 0.0, oy: float = 0.0):
    """Grid cells and coverage of a polygon (mm points), by supersampling each cell ss × ss (grid offset as in
    _cells_capsule)."""
    import numpy as np
    from matplotlib.path import Path as MPath
    pts = np.asarray(pts, float)
    x0, y0 = pts.min(axis=0) - h
    x1, y1 = pts.max(axis=0) + h
    ix = np.arange(int(np.floor(x0 / h - ox)), int(np.ceil(x1 / h - ox)) + 1)
    iy = np.arange(int(np.floor(y0 / h - oy)), int(np.ceil(y1 / h - oy)) + 1)
    sub = (np.arange(ss) + 0.5) / ss
    X, Y = np.meshgrid(((ix[:, None] + sub[None, :]).ravel() + ox) * h,
                       ((iy[:, None] + sub[None, :]).ravel() + oy) * h, indexing="ij")
    inside = MPath(pts).contains_points(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)
    c = inside.reshape(len(ix), ss, len(iy), ss).mean(axis=(1, 3))
    m = c > 0
    I, J = np.meshgrid(ix, iy, indexing="ij")
    return I[m], J[m], c[m]


def _raster_scan(pts, h, ss: int = 2, ox: float = 0.0, oy: float = 0.0):
    """Coverage (0-1) of cells of an h grid (centres at (i+0.5+ox)h) by a polygon (even-odd), exact horizontally,
    ss sample rows per cell vertically. Returns I, J, c."""
    import numpy as np
    P=np.asarray(pts,float); x0,y0=P.min(0); x1,y1=P.max(0)
    i0=int(np.floor(x0/h-ox))-1; i1=int(np.ceil(x1/h-ox))+1
    j0=int(np.floor(y0/h-oy))-1; j1=int(np.ceil(y1/h-oy))+1
    nI=i1-i0; nJ=j1-j0
    cov=np.zeros((nJ,nI))
    ys=((np.arange(j0,j1)[:,None]+(np.arange(ss)+0.5)[None,:]/ss)+oy)*h     # sample rows (nJ, ss)
    ys=ys.ravel()
    ax,ay=P[:,0],P[:,1]; bx,by=np.roll(ax,-1),np.roll(ay,-1)
    keep=ay!=by; ax,ay,bx,by=ax[keep],ay[keep],bx[keep],by[keep]
    lo=np.minimum(ay,by); hi=np.maximum(ay,by)
    # rows each edge crosses
    r0=np.searchsorted(ys,lo,side='left'); r1=np.searchsorted(ys,hi,side='left')
    cnt=r1-r0; tot=int(cnt.sum())
    if tot==0: return np.zeros(0,int),np.zeros(0,int),np.zeros(0)
    e=np.repeat(np.arange(len(ax)),cnt); r=np.arange(tot)-np.repeat(np.cumsum(cnt)-cnt,cnt)+r0[e]
    xi=ax[e]+(ys[r]-ay[e])*(bx[e]-ax[e])/(by[e]-ay[e])
    o=np.lexsort((xi,r)); r,xi=r[o],xi[o]
    # pair consecutive crossings in each row (even-odd)
    first=np.r_[True,r[1:]!=r[:-1]]
    k=np.arange(len(r))-np.maximum.accumulate(np.where(first,np.arange(len(r)),0))
    a=(k%2==0); sa=np.where(a)[0]; sb=sa+1; sb=sb[sb<len(r)]; sa=sa[:len(sb)]
    ok=r[sa]==r[sb]; sa,sb=sa[ok],sb[ok]
    row=r[sa]; xa=xi[sa]/h-ox-i0; xb=xi[sb]/h-ox-i0          # in cell units from i0
    # add span coverage [xa,xb] into cells of row//ss using a difference array of fractional coverage
    jr=row//ss
    acc=np.zeros((nJ,nI+2))
    fa=np.floor(xa).astype(int); fb=np.floor(xb).astype(int)
    same=fa==fb
    np.add.at(acc,(jr[same],fa[same]+1),(xb-xa)[same]); np.add.at(acc,(jr[same],fa[same]+2),-(xb-xa)[same])
    d=~same
    # partial first cell, full middle cells, partial last cell
    np.add.at(acc,(jr[d],fa[d]+1),(fa[d]+1-xa[d])); np.add.at(acc,(jr[d],fa[d]+2),-(fa[d]+1-xa[d]))
    np.add.at(acc,(jr[d],fa[d]+2),1.0); np.add.at(acc,(jr[d],fb[d]+1),-1.0)
    np.add.at(acc,(jr[d],fb[d]+1),(xb[d]-fb[d])); np.add.at(acc,(jr[d],fb[d]+2),-(xb[d]-fb[d]))
    # acc holds, per cell, diff of "per-cell constant" -- partial terms were added as single-cell bumps
    cov=np.cumsum(acc,axis=1)[:,1:nI+1]/ss
    J,I=np.nonzero(cov>1e-12)
    return I+i0, J+j0, np.minimum(cov[J,I],1.0)


def copper2d_resistance(board, net: str, pair, copper_oz: float, thickness_mm: float, h: float = 0.0125,
                        k_term: float = 1e4, SS: int = 4, include_zones: bool = False):
    """Pad-to-pad resistance (Ω) of one net by a 2-D solve of its copper (plan addendum 2, as corrected in
    addendum 3). Every copper item of the net is drawn on an h-mm grid per layer from KiCad's own shapes (tracks and
    arcs as capsules, pads from KiCad's polygon, vias as discs). Two neighbouring cells conduct σ t times the copper
    fraction of the cell-sized box centred on the face between them. Pads and via rings are near-perfect conductors
    (k_term times the copper's conductance over the part of that box they cover), so a terminal's edge is
    anti-aliased like a track's instead of snapping to whole cells. A through-hole pad joins its layers at no
    resistance; a via joins them through its barrel, split evenly between successive copper layers. 1 A in at the
    centre of pad A, out at the centre of pad B."""
    import numpy as np
    import pcbnew
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.sparse.linalg import spsolve
    rho = RHO_CU_20C * (1 + ALPHA_CU * (NET_TEMP_C - 20.0))
    sheet_g = copper_oz * OZ_TO_UM * 1e-6 / rho            # σ t, siemens per square
    stack = list(board.GetEnabledLayers().CuStack())
    tracks = [t for t in board.GetTracks() if t.GetNetname() == net and t.Type() != pcbnew.PCB_VIA_T]
    vias = [t for t in board.GetTracks() if t.GetNetname() == net and t.Type() == pcbnew.PCB_VIA_T]
    pads = [p for p in board.GetPads() if p.GetNetname() == net]
    nm = 1e-6
    off = 1 << 20
    shapes: list = []                  # (layer, "capsule"|"polygon", args, terminal key or None)
    for t in tracks:
        ly, hw = t.GetLayer(), t.GetWidth() * nm / 2.0
        if t.Type() == pcbnew.PCB_ARC_T:
            a, m_, b = t.GetStart(), t.GetMid(), t.GetEnd()
            pts = _arc_pts((a.x * nm, a.y * nm), (m_.x * nm, m_.y * nm), (b.x * nm, b.y * nm))
        else:
            pts = [(t.GetStart().x * nm, t.GetStart().y * nm), (t.GetEnd().x * nm, t.GetEnd().y * nm)]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            shapes.append((ly, "capsule", (x0, y0, x1, y1, hw), None))
    for i, v in enumerate(vias):
        c = v.GetPosition()
        for ly in stack:
            if v.IsOnLayer(ly):
                shapes.append((ly, "capsule", (c.x * nm, c.y * nm, c.x * nm, c.y * nm, v.GetWidth() * nm / 2.0),
                               ("via", i)))
    if include_zones:                  # addendum 4: the pour's copper as KiCad filled it (clearances, spokes)
        for z in board.Zones():
            if z.GetNetname() != net or z.GetIsRuleArea():
                continue
            for ly in stack:
                if not z.IsOnLayer(ly):
                    continue
                ps = z.GetFilledPolysList(ly)
                ps = pcbnew.SHAPE_POLY_SET(ps)
                ps.Fracture(pcbnew.SHAPE_POLY_SET.PM_FAST)
                for k in range(ps.OutlineCount()):
                    o = ps.Outline(k)
                    shapes.append((ly, "scan", ([(o.CPoint(j).x * nm, o.CPoint(j).y * nm)
                                                  for j in range(o.PointCount())],), None))
    pad_key: dict = {}
    pad_pos: dict = {}
    for p in pads:
        key = ("pad", p.GetParent().GetReference(), p.GetNumber(), id(p))
        pad_key.setdefault(f"{key[1]}.{key[2]}", []).append(key)
        pad_pos[key] = (p.GetPosition().x * nm, p.GetPosition().y * nm)
        for ly in stack:
            if not p.IsOnLayer(ly):
                continue
            ps = pcbnew.SHAPE_POLY_SET()
            p.TransformShapeToPolygon(ps, ly, 0, 500)
            for k in range(ps.OutlineCount()):
                o = ps.Outline(k)
                shapes.append((ly, "polygon", ([(o.CPoint(j).x * nm, o.CPoint(j).y * nm)
                                                for j in range(o.PointCount())],), key))
    def resolve(name):                 # "REF.NUM", or "REF.NUM@x,y" where two pads share the name
        base, _, at = name.partition("@")
        ks = pad_key.get(base, [])
        if at:
            x, y = (float(v) for v in at.split(","))
            ks = [min(ks, key=lambda k: (pad_pos[k][0] - x) ** 2 + (pad_pos[k][1] - y) ** 2)] if ks else []
        return ks[0] if len(ks) == 1 else None
    ka, kb = resolve(pair[0]), resolve(pair[1])
    if ka is None or kb is None or not shapes:
        return None

    # All coverages come from one rasterization on a half-size grid (2 × 2 samples per half cell, the same sample
    # density as 4 × 4 per cell): a cell is the mean of its 2 × 2 half cells, and the box around the face between
    # two cells is the mean of the 2 × 2 half cells straddling it.
    hh = h / 2.0
    half = {False: ([], [], [], []), True: ([], [], [], [])}
    for ly, kind, args, tk in shapes:
        P, Q, c = (_cells_capsule(*args, hh, ss=max(SS // 2, 1)) if kind == "capsule"
                   else _raster_scan(*args, hh, ss=max(SS // 2, 1)) if kind == "scan"
                   else _cells_polygon(*args, hh, ss=max(SS // 2, 1)))
        for t_ in ((False, True) if tk is not None else (False,)):
            Ls_, Ps_, Qs_, Cs_ = half[t_]
            Ls_.append(np.full(P.size, ly, np.int64)); Ps_.append(P.astype(np.int64)); Qs_.append(Q.astype(np.int64))
            Cs_.append(c)

    def union(t_):
        Ls_, Ps_, Qs_, Cs_ = half[t_]
        if not Ls_:
            return (np.zeros(0, np.int64),) * 3 + (np.zeros(0),)
        L0, P0, Q0, C0 = (np.concatenate(x) for x in (Ls_, Ps_, Qs_, Cs_))
        k0 = (L0 << 42) | ((P0 + off) << 21) | (Q0 + off)
        o_ = np.lexsort((C0, k0))
        k0, L0, P0, Q0, C0 = k0[o_], L0[o_], P0[o_], Q0[o_], C0[o_]
        last = np.r_[k0[1:] != k0[:-1], True]
        return L0[last], P0[last], Q0[last], np.minimum(C0[last], 1.0)

    def gather(Lh, Ph, Qh, Ch, fi, fj):
        """Sum of half-cell coverage / 4 into cells keyed by (layer, fi(P), fj(Q))."""
        if Lh.size == 0:
            return np.zeros(0, np.int64), np.zeros(0)
        k_ = (Lh << 42) | ((fi(Ph) + off) << 21) | (fj(Qh) + off)
        u_, inv = np.unique(k_, return_inverse=True)
        return u_, np.minimum(np.bincount(inv, weights=Ch) / 4.0, 1.0)

    cell_ = lambda v: v // 2                # half cells 2i, 2i+1 -> cell i
    face_ = lambda v: (v - 1) // 2          # half cells 2i+1, 2i+2 -> face after cell i
    Ha, Ht = union(False), union(True)

    def grid(ox, oy, only_terms=False):
        H_ = Ht if only_terms else Ha
        return gather(*H_, face_ if ox else cell_, face_ if oy else cell_)

    def lookup(keys, k_, c_):
        if k_.size == 0:
            return np.zeros(keys.size)
        pos = np.searchsorted(k_, keys)
        ok = (pos < k_.size) & (k_[np.minimum(pos, k_.size - 1)] == keys)
        return np.where(ok, c_[np.minimum(pos, k_.size - 1)], 0.0)

    key, _C = grid(0.0, 0.0)
    n = key.size
    rows, cols, gs = [], [], []
    for di, dj in ((1, 0), (0, 1)):
        fk, fc = grid(0.5 * di, 0.5 * dj)
        tk_, tc_ = grid(0.5 * di, 0.5 * dj, only_terms=True)
        nk = key + (np.int64(di) << 21) + np.int64(dj)
        pos = np.searchsorted(key, nk)
        ok = (pos < n) & (key[np.minimum(pos, n - 1)] == nk)
        a_i, b_i = np.nonzero(ok)[0], pos[ok]
        g = sheet_g * (lookup(key[a_i], fk, fc) + k_term * lookup(key[a_i], tk_, tc_))
        rows.append(a_i); cols.append(b_i); gs.append(g)
    # terminal anchors: the cell nearest each pad's (or via's) centre on each of its layers
    anchor: dict = {}
    for ly, kind, args, tk in shapes:
        if tk is None:
            continue
        if kind == "capsule":
            cx, cy = args[0], args[1]
        else:
            pts = np.asarray(args[0]); cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
        ck = (np.int64(ly) << 42) | ((np.int64(np.floor(cx / h)) + off) << 21) | (np.int64(np.floor(cy / h)) + off)
        pidx = int(np.searchsorted(key, ck))
        if pidx < n and key[pidx] == ck:
            anchor.setdefault((tk, ly), pidx)
    big = sheet_g * k_term * 100.0
    for p_keys in pad_key.values():                       # a pad on several layers: one conductor through its hole
        for k0 in p_keys:
            a_ = [anchor[(k0, ly)] for ly in stack if (k0, ly) in anchor]
            for x, y in zip(a_, a_[1:]):
                rows.append(np.array([x])); cols.append(np.array([y])); gs.append(np.array([big]))
    for i, v in enumerate(vias):                          # via barrels, chained through the layers they touch
        lys = [ly for ly in stack if (("via", i), ly) in anchor]
        if len(lys) < 2:
            continue
        r_full = rho * thickness_mm * 1e-3 / (3.141592653589793 * v.GetDrillValue() * nm * 1e-3
                                              * VIA_PLATING_MM * 1e-3)
        seg = r_full / max(len(stack) - 1, 1)
        for l0, l1 in zip(lys, lys[1:]):
            gap = stack.index(l1) - stack.index(l0)
            rows.append(np.array([anchor[(("via", i), l0)]])); cols.append(np.array([anchor[(("via", i), l1)]]))
            gs.append(np.array([1.0 / (seg * gap)]))
    R, Cc, G = np.concatenate(rows), np.concatenate(cols), np.concatenate(gs)
    m = (R != Cc) & (G > 0)
    R, Cc, G = R[m], Cc[m], G[m]
    M = coo_matrix((np.r_[G, G, -G, -G], (np.r_[R, Cc, R, Cc], np.r_[R, Cc, Cc, R])), shape=(n, n)).tocsr()
    a_idx = next((anchor[(ka, ly)] for ly in stack if (ka, ly) in anchor), None)
    b_idx = next((anchor[(kb, ly)] for ly in stack if (kb, ly) in anchor), None)
    if a_idx is None or b_idx is None:
        return None
    _nc, lab = connected_components(M != 0, directed=False)
    if lab[a_idx] != lab[b_idx]:
        return float("inf")
    keep = np.where(lab == lab[a_idx])[0]
    keep = keep[keep != b_idx]
    where = np.full(n, -1)
    where[keep] = np.arange(keep.size)
    Mk = M[keep][:, keep]
    rhs = np.zeros(keep.size)
    rhs[where[a_idx]] = 1.0
    if keep.size <= 150_000:
        sol = spsolve(Mk.tocsc(), rhs)
    else:                              # a direct solve of a long net's fine grid runs out of memory: AMG-preconditioned CG
        import pyamg
        from scipy.sparse.linalg import cg
        Mk = Mk.tocsr()
        ml = pyamg.smoothed_aggregation_solver(Mk, symmetry="symmetric", max_coarse=500)
        sol, info = cg(Mk, rhs, rtol=1e-11, atol=0.0, maxiter=4000, M=ml.aspreconditioner(cycle="V"))
        if info != 0:
            return None
    return float(sol[where[a_idx]])


def _arc_pts(p0, pm, p1, n: int = 24):
    import math
    (x0, y0), (xm, ym), (x1, y1) = p0, pm, p1
    d = 2.0 * (x0 * (ym - y1) + xm * (y1 - y0) + x1 * (y0 - ym))
    if abs(d) < 1e-12:
        return [p0, p1]
    ux = ((x0 ** 2 + y0 ** 2) * (ym - y1) + (xm ** 2 + ym ** 2) * (y1 - y0) + (x1 ** 2 + y1 ** 2) * (y0 - ym)) / d
    uy = ((x0 ** 2 + y0 ** 2) * (x1 - xm) + (xm ** 2 + ym ** 2) * (x0 - x1) + (x1 ** 2 + y1 ** 2) * (xm - x0)) / d
    r = math.hypot(x0 - ux, y0 - uy)
    a0, am, a1 = (math.atan2(y - uy, x - ux) for x, y in (p0, pm, p1))
    sweep = (a1 - a0) % (2 * math.pi)
    if (am - a0) % (2 * math.pi) > sweep:
        sweep -= 2 * math.pi
    return [(ux + r * math.cos(a0 + sweep * k / n), uy + r * math.sin(a0 + sweep * k / n)) for k in range(n + 1)]


def altium(folder: str):
    """Score the Altium kit (benchmarks/electrical/altium/README.md): the impedance table and the boards."""
    import csv
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from suspension import pcb_doctor as D
    from suspension.electronics import DiffPair
    folder = Path(folder)
    out = {"impedance": [], "boards": []}
    atlc_all = (json.loads((OUT / "atlc_impedance.json").read_text())["single_ended"]
                + json.loads((OUT / "atlc_impedance.json").read_text())["differential"]
                + json.loads((OUT / "atlc_impedance_round2.json").read_text())["single_ended"]
                + json.loads((OUT / "atlc_impedance_round2.json").read_text())["differential"])
    imp = folder / "impedance_template.csv"
    if imp.exists():
        for row, ref in zip(csv.DictReader(imp.open(encoding="utf-8")), atlc_all):
            if not (row.get("altium_impedance_ohm") or "").strip():
                continue
            z_alt = float(row["altium_impedance_ohm"])
            w = float(row["altium_width_mm_if_you_could_only_set_a_target"] or row["w_mm"])
            dp = DiffPair("x", "e", trace_w_mm=w, height_mm=float(row["h_mm"]), eps_r=float(row["er"]),
                          spacing_mm=float(row["s_mm"] or 0.2))
            diff = row["kind"] == "differential"
            z_kk = dp.differential_z0_ohm() if diff else dp.single_ended_z0_ohm()
            z_atlc = ref.get("atlc_zdiff_ohm" if diff else "atlc_z0_ohm")
            lim = 10.0 if diff else 5.0
            e = 100.0 * (z_kk - z_alt) / z_alt
            out["impedance"].append({"case": row["case"], "kind": row["kind"], "altium_ohm": z_alt, "kinematik_ohm": z_kk,
                                     "error_pct": e, "within_limit": abs(e) <= lim,
                                     "atlc_vs_altium_pct": (100.0 * (z_atlc - z_alt) / z_alt
                                                            if row["altium_width_mm_if_you_could_only_set_a_target"] == ""
                                                            else None)})
    brd = folder / "boards_template.csv"
    kicad_ref = json.loads((OUT / "kicad_boards.json").read_text())["boards"]
    if brd.exists():
        for row in csv.DictReader(brd.open(encoding="utf-8")):
            if not (row.get("tracks") or "").strip():
                continue
            stem = Path(row["board"]).stem
            for kind in ("binary", "ascii"):
                f = folder / f"{stem}_{kind}.PcbDoc"
                if not f.exists():
                    continue
                b = D.parse_board(f.read_bytes(), f.name)
                kk = {"tracks": sum(1 for s_ in b.segments if not s_.from_arc),
                      "arcs": len({s_.width_span for s_ in b.segments if s_.from_arc}) if kind == "ascii"
                      else sum(1 for s_ in b.segments if s_.from_arc),
                      "vias": len(b.vias), "pads": sum(len(fp.pads) for fp in b.footprints),
                      "components": len(b.footprints)}
                alt = {k: int(row[k]) for k in kk if (row.get(k) or "").strip()}
                lengths = {}
                for s_ in b.segments:
                    lengths[b.net_name(s_.net)] = lengths.get(b.net_name(s_.net), 0.0) + s_.length_mm
                ref_nets = kicad_ref.get(row["board"], {}).get("nets", {})
                common = [n for n in ref_nets if n in lengths]
                worst = max((abs(lengths[n] - ref_nets[n]["length_mm"]) / max(ref_nets[n]["length_mm"], 1e-9)
                             for n in common), default=None)
                out["boards"].append({"board": row["board"], "file": kind, "altium": alt, "kinematik": kk,
                                      "counts_match": all(kk[k] == v for k, v in alt.items()),
                                      "nets_compared": len(common), "worst_length_rel": worst})
    _write("altium_results.json", out)
    for r in out["impedance"]:
        print(f"{r['case']:8s} {r['kind']:12s} Altium {r['altium_ohm']:7.2f}  KinematiK {r['kinematik_ohm']:7.2f} "
              f"{r['error_pct']:+6.2f} %  {'OK' if r['within_limit'] else 'MISS'}")
    for r in out["boards"]:
        print(r["board"], r["file"], "counts match" if r["counts_match"] else f"COUNTS DIFFER {r}",
              f"nets {r['nets_compared']}, worst length {r['worst_length_rel']}")


def _write(name, obj):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(obj, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print("wrote", OUT / name)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["kicad", "spice", "atlc", "altium"])
    ap.add_argument("--dir", help="altium: the folder with the returned kit files")
    a = ap.parse_args(argv)
    if a.step == "altium":
        altium(a.dir)
    else:
        {"kicad": kicad, "spice": spice, "atlc": atlc}[a.step]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
