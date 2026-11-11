#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""A team's own car, blind (suspension/team_test.py).

  seal    --plan plan.json --out SEALED.json      before the test day; push it or post its sha256 publicly
  verify  --record SEALED.json                    unchanged, same code, same input files, reproduces
  tolerance --plan plan.json --sigma-mm 0.2 --gauge-deg 0.1   camber/toe tolerance from your measuring error
  score   --record SEALED.json --measured measured.csv --out DIR   after the test day

File paths inside the plan (tire_json, course_csv) are read relative to the plan's folder.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension import team_test as T  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("seal")
    s.add_argument("--plan", required=True)
    s.add_argument("--out", required=True)
    v = sub.add_parser("verify")
    v.add_argument("--record", required=True)
    v.add_argument("--root", default=None, help="folder the plan's file paths are relative to (default: record's)")
    t = sub.add_parser("tolerance")
    t.add_argument("--plan", required=True)
    t.add_argument("--sigma-mm", type=float, required=True, help="hardpoint measuring error, 1 sigma, mm")
    t.add_argument("--gauge-deg", type=float, required=True, help="angle gauge accuracy, deg")
    t.add_argument("--draws", type=int, default=200)
    k = sub.add_parser("score")
    k.add_argument("--record", required=True)
    k.add_argument("--measured", required=True)
    k.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "seal":
        plan_path = Path(a.plan)
        rec = T.seal(json.loads(plan_path.read_text(encoding="utf-8")), root=plan_path.parent)
        T.write_json(rec, a.out)
        print(f"sealed {len(rec['predictions'])} setups -> {a.out}\nrecord sha256 {rec['record_sha256']}\n"
              "Push this file, or post the sha256 in a public issue, BEFORE the test day.")
        return 0
    if a.cmd == "tolerance":
        plan_path = Path(a.plan)
        tol = T.suggested_tolerance(json.loads(plan_path.read_text(encoding="utf-8")), a.sigma_mm, a.gauge_deg,
                                    root=plan_path.parent, n=a.draws)
        print(json.dumps(tol) + "\nPut this in the plan as \"tolerance\" before sealing.")
        return 0
    rec = json.loads(Path(a.record).read_text(encoding="utf-8"))
    if a.cmd == "verify":
        res = T.verify(rec, root=Path(a.root) if a.root else Path(a.record).parent)
        print(json.dumps(res))
        return 0 if res["intact"] else 1
    try:
        sc = T.score(rec, T.load_measured_csv(a.measured))
    except ValueError as e:
        print(f"REFUSED: {e}", file=sys.stderr)
        return 1
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    T.write_json(sc, out / "scorecard.json")
    md = T.scorecard_markdown(sc)
    (out / "SCORECARD.md").write_text(md, encoding="utf-8")
    print(md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
