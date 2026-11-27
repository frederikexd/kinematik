"""Characterization suite for suspension/sim_handoff.py (mutation gate); see tests/_golden.py. Plates on several
patterns and hole sizes, the flange and box candidates, odd candidates (no meta, starter-only outlines, a thickness
key, holes that match no thread), each through the section, its bolt inference, pattern phrase, mesh shortlist,
named selections, study, trust map, manifest, DXF (every group-code pair) and bundle (every member)."""
import hashlib
import io
import sys
import zipfile
from pathlib import Path

from suspension import sim_handoff as sh
from tests._golden import plain
from tests._golden_lazy import check_lazy

sys.path.insert(0, str(Path(__file__).parent))
from test_sim_handoff import _box_candidate, _flange_candidate, _plate_candidate, _rect  # noqa: E402


def _try(fn, *a, **kw):
    try:
        return plain(fn(*a, **kw))
    except (ValueError, KeyError, TypeError, AttributeError) as e:
        return {"raised": type(e).__name__, "message": str(e)}


def _candidates():
    c = {"plate3": _plate_candidate(), "plate4": _plate_candidate(pcd=70.0, n=4, hole=6.4, plate=100.0),
         "plate6": _plate_candidate(pcd=110.0, n=6, hole=10.5, plate=150.0),
         "plate2": _plate_candidate(pcd=50.0, n=2, hole=13.0, plate=80.0),
         "plate_odd_hole": _plate_candidate(hole=7.1), "flange": _flange_candidate(), "box": _box_candidate()}
    p = _plate_candidate()
    p["meta"] = {"Plate thickness (mm)": 6.0, "Design load (N)": 9000.0, "Material": "4130 steel"}
    c["thick"] = p
    p = _plate_candidate()
    p["meta"] = {}
    c["nometa"] = p
    p = _plate_candidate()
    p["meta"] = {"Extrude depth": 12.0, "Max load": "n/a", "Material": "unobtainium"}
    c["oddmeta"] = p
    c["outline_only"] = {"label": "Bracket", "meta": {"Envelope (mm)": 60.0},
                         "dxf_kwargs": {"polylines": [{"pts": _rect(60.0, 40.0), "closed": True},
                                                      {"pts": _rect(10.0, 10.0, 25.0, 15.0), "closed": True},
                                                      {"pts": [(0.0, 0.0), (5.0, 5.0)], "closed": False}]}}
    c["bore"] = {"label": "Motor register", "meta": {"Bore (mm)": 75.0},
                 "dxf_kwargs": {"polylines": [{"pts": _rect(140.0, 140.0, -70.0, -70.0), "closed": True}],
                                "circles": [{"c": (0.0, 0.0), "r": 37.5}, {"c": (55.0, 55.0), "r": 4.2},
                                            {"c": (-55.0, 55.0), "r": 4.2}]}}
    c["empty"] = {"label": "Nothing", "meta": {}, "dxf_kwargs": {}}
    return c


def _dxf(b):
    lines = b.decode("utf-8", "replace").splitlines()
    return {"n": len(lines), "sha": hashlib.sha256(b).hexdigest()[:20], "head": lines[:40]}


def _bundle(b):
    z = zipfile.ZipFile(io.BytesIO(b))
    return {n: hashlib.sha256(z.read(n)).hexdigest()[:16] for n in sorted(z.namelist())}


def _everything():
    for d in (3.4, 4.5, 5.5, 6.4, 6.6, 8.4, 9.0, 10.5, 11.0, 13.0, 13.5, 17.5, 2.0, 25.0, 75.0, 0.0, -1.0):
        for grade in ("8.8", "10.9", "12.9", "A2-70"):
            yield f"bolt.{d}.{grade}", _try(sh.infer_bolt, d, grade)
    yield "bolt.preload", _try(sh.infer_bolt, 8.4, "10.9", preload_fraction=0.75, nut_factor=0.15)
    for k, cand in _candidates().items():
        sec = sh.section_from_candidate("suspension", cand)
        r = {"sec": _try(lambda: sec)}
        for attr in ("bbox", "width_mm", "height_mm", "max_dim_mm", "area_mm2", "bore", "digest"):
            r[attr] = _try(lambda a=attr: (lambda v: v() if callable(v) else v)(getattr(sec, a)))
        r["groups"] = [[_try(lambda g=g: g.selection() if callable(g.selection) else g.selection),
                        _try(g.as_dict), _try(sh.pattern_phrase, g)]
                       for g in getattr(sec, "fastener_groups", None) or getattr(sec, "groups", None) or []]
        mesh = sh.mesh_shortlist(sec)
        r["mesh"] = plain(mesh.as_dict())
        r["mesh_rows"] = plain(sh.mesh_rows(mesh))
        r["named"] = _try(sh.named_selections, sec)
        r["study"] = _try(lambda: sh.study_spec(sec).as_dict())
        r["trust"] = _try(sh.trust_map, sec)
        r["manifest"] = _try(sh.handoff_manifest, sec, "FR-26")
        r["dxf"] = _try(lambda: _dxf(sh.section_dxf(sec)))
        r["bundle"] = _try(lambda: _bundle(sh.bundle_bytes(sec, "FR-26")))
        r["handoff"] = _try(lambda: {k2: (v2 if not isinstance(v2, bytes) else _bundle(v2) if v2[:2] == b"PK" else hashlib.sha256(v2).hexdigest()[:16])
                                     for k2, v2 in sh.handoff_for_candidate("aero", cand, "T1").items()})
        yield f"cand.{k}", r
    yield "consts", [sh.SCHEMA, sh.DISCLAIMER]


def test_every_candidate_through_the_handoff():
    check_lazy("sim_handoff", _everything())
