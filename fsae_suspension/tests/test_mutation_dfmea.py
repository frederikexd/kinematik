"""Characterization suite for suspension/dfmea.py (mutation gate); see tests/_golden.py. Ratings across the clamp
(garbage, fractions, out of range), every risk band at and either side of each severity and RPN threshold, rows from
loose spreadsheet dicts in every column spelling, the seed table, and the dashboard and action tracker on records
built to hit every data-quality flag, tie and sort order."""
from suspension import dfmea as F
from tests._golden import plain
from tests._golden_lazy import check_lazy

RATINGS = [None, "", "x", "3", "4.5", 4.49, 0, -3, 1, 10, 11, 9.5, 6.6, float("nan"), True]


def _records():
    out = []
    for s, o, d, status, action, owner, ev in (
            (9, 1, 1, "Open", "fix", "", ""), (7, 2, 2, "In Progress", "", "", ""), (5, 5, 8, "Closed", "done", "A", ""),
            (5, 8, 5, "Accepted", "x", "", "evidence"), (3, 3, 9, "Open", "act", "B", ""), (10, 10, 10, " closed ", "", "",
                                                                                                   "e"),
            (2, 2, 2, "Open", "small", "C", ""), (6, 7, 5, "open", "  ", "", ""), (6, 5, 7, "Open", "tie", "", ""),
            (8, 3, 3, "ACCEPTED", "y", "", ""), (4, 5, 4, "Open", "border", "D", ""), (4, 4, 5, "Open", "b2", "", "")):
        out.append({"Subsystem": f"S{s % 3}", "Item / Component": f"item{s}{o}{d}", "Severity": s, "Occurrence": o,
                    "Detection": d, "Status": status, "Recommended Action": action, "Owner": owner,
                    "Evidence / Notes": ev, "Due Date": "2026-12-01"})
    out.append({"Subsystem": "S1", "Item / Component": "tie10", "Severity": 10, "Occurrence": 4, "Detection": 5,
                "Status": "Open", "Recommended Action": "tie high sev"})
    out.append({"Subsystem": "S1", "Item / Component": "tie5", "Severity": 5, "Occurrence": 5, "Detection": 8,
                "Status": "Open", "Recommended Action": "tie low sev"})
    out.append({"Item / Component": "closed no notes", "Severity": 3, "Status": "Closed", "Recommended Action": "x"})
    out.append({"Item / Component": "closed no notes 2", "Severity": 4, "Status": "Accepted"})
    out.append({"Item / Component": "wip", "Severity": 7, "Occurrence": 6, "Detection": 4, "Status": "In Progress",
                "Recommended Action": "keep going", "Owner": "E"})
    out.append({"Item / Component": "no subsystem", "Severity": 9, "Recommended Action": "assign"})
    out.append({"Severity": "bad"})
    out.append({"Subsystem": "", "Recommended Action": "orphan"})
    return out


def _cases():
    yield "clamp", [F._clamp_rating(v) for v in RATINGS]
    yield "rpn", [F.compute_rpn(a, b, c) for a in RATINGS[:6] for b in (1, "5", 12) for c in (None, 3)]
    for s in (1, 6, 7, 8, 9, 10, "x"):
        yield f"band.{s}", [F.classify_risk(s, r).value for r in (0, 79, 79.9, 80, 199, 200, 299, 300, 1000, "x", None)]
    yield "row.default", [plain(F.DFMEARow()), F.DFMEARow().rpn, F.DFMEARow().band.value, F.DFMEARow().to_record()]
    loose = [{"Subsystem": "Brakes", "ITEM": "caliper", " Function ": "stop", "Failure Mode": "fade", "effect": "long",
              "Severity": "8", "cause": "heat", "Occurrence": 3.6, "Prevention Controls": "duct", "Detection Controls": "test",
              "Detection": 11, "action": "bigger rotor", "Owner": "BR", "due date": "2027-01-01", "Status": "Open",
              "evidence": "log", "RPN": 999, "unknown col": 1},
             {"item / component": "pump", "function / requirement": "flow", "effect of failure": "overheat",
              "potential cause / mechanism": "cavitation", "recommended action": "prime", "evidence / notes": "n",
              "severity": None, "occurrence": "x", "detection": 0},
             {}, {7: "seven", None: "none"},
             {"unknown first": 1, "Item / Component": None, "Failure Mode": "creep", "Severity": 6, "RPN": 5}]
    for i, m in enumerate(loose):
        r = F.row_from_mapping(m)
        yield f"loose.{i}", [plain(r), r.rpn, r.band.value, r.to_record()]
    yield "from_records", [plain(r) for r in F.from_records(loose + [{"Severity": 9}])]
    seed = F.seed_rows()
    yield "seed", seed
    yield "seed.rows", [[plain(r), r.rpn, r.band.value] for r in F.from_records(seed)]
    recs = _records()
    for n in (8, 3, 0, 100):
        yield f"dash.{n}", plain(F.dashboard_stats(recs, top_n=n))
    yield "dash.seed", plain(F.dashboard_stats(seed))
    yield "dash.empty", plain(F.dashboard_stats([]))
    yield "actions", F.action_items(recs)
    yield "actions.seed", F.action_items(seed)
    yield "resolved", [F._is_resolved(s) for s in ("Closed", " accepted", "Open", "In Progress", None, "closedx")]
    yield "tables", [F.COLUMNS, F.SUBSYSTEM_OPTIONS, F.STATUS_OPTIONS, plain(F.SEVERITY_SCALE),
                     plain(F.OCCURRENCE_SCALE), plain(F.DETECTION_SCALE), [b.value for b in F.RiskBand]]


def test_every_rating_band_and_rollup():
    check_lazy("dfmea", _cases())
