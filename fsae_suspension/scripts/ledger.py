#!/usr/bin/env python3
# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The prediction ledger, from the command line.

  seal    predictions for a specs CSV -> SEALED.json (push it before the results are used)
  score   a sealed record against an official-results CSV -> SCORECARD.md and scorecard.json
  verify  is a sealed record unaltered, and does today's code reproduce it?

Examples (from the repo root):
  python fsae_suspension/scripts/ledger.py seal --specs benchmarks/ledger/fsg2027/specs.csv \\
      --event "FSG 2027" --kind blind --out benchmarks/ledger/fsg2027/SEALED.json
  python fsae_suspension/scripts/ledger.py score --record benchmarks/ledger/fsg2027/SEALED.json \\
      --results benchmarks/ledger/fsg2027/results.csv --out benchmarks/ledger/fsg2027
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension import ledger  # noqa: E402


def _clean(o):
    if isinstance(o, float) and math.isnan(o):
        return None
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_clean(v) for v in o]
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("seal")
    s.add_argument("--specs", required=True)
    s.add_argument("--event", required=True)
    s.add_argument("--kind", required=True, choices=["blind", "hindcast"])
    s.add_argument("--note", default="")
    s.add_argument("--out", required=True)
    c = sub.add_parser("score")
    c.add_argument("--record", required=True)
    c.add_argument("--results", required=True)
    c.add_argument("--out", required=True)
    v = sub.add_parser("verify")
    v.add_argument("--record", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "seal":
        rec = ledger.seal(ledger.load_specs_csv(a.specs), a.event, a.kind, a.note)
        Path(a.out).write_text(json.dumps(rec, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"sealed {len(rec['predictions'])} cars -> {a.out}\nrecord sha256 {rec['record_sha256']}")
        return 0
    rec = json.loads(Path(a.record).read_text(encoding="utf-8"))
    if a.cmd == "verify":
        chk = ledger.verify(rec)
        print(json.dumps(chk))
        return 0 if chk["intact"] else 1
    chk = ledger.verify(rec)
    if not chk["intact"]:
        print("REFUSED: the sealed record has been altered since it was sealed", file=sys.stderr)
        return 1
    sc = ledger.score(rec, ledger.load_results_csv(a.results))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "SCORECARD.md").write_text(ledger.scorecard_markdown(sc), encoding="utf-8")
    (out / "scorecard.json").write_text(json.dumps(_clean(sc), indent=1, ensure_ascii=False) + "\n",
                                        encoding="utf-8")
    print(ledger.scorecard_markdown(sc))
    return 0


if __name__ == "__main__":
    sys.exit(main())
