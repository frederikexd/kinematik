"""Ripple (suspension/ripple.py): what each edit did, with per-entry undo, and how much of the car is still the
example. These pin the bookkeeping: what counts as a change, when edits merge, what undo writes back, and the
"N of 7 inputs are yours" count."""
from suspension import ripple as r

VP = {"mass": 280.0, "cg_height": 300.0, "weight_dist_front": 0.47, "use_spring_rates": False,
      "spring_rate_front": 35.0, "spring_rate_rear": 35.0, "arb_rate_front": 0.0, "arb_rate_rear": 0.0,
      "roll_stiffness_front": 350.0, "roll_stiffness_rear": 300.0, "name": "car", "nan_field": float("nan"),
      "tire_coeffs": {"a": 1}}
HP = {"upper_outer": [0.0, 540.0, 300.0], "lower_outer": [0.0, 560.0, 120.0], "pushrod_outer": [1.0, 2.0, 3.0],
      "static_camber": -1.5, "static_toe": 0.0}
POINTS, ROCKER = ["upper_outer", "lower_outer"], ["pushrod_outer"]


def snap(vp=None, hp=None, topo="double_wishbone", preset="Front (default)", pts=None):
    return r.snapshot(dict(VP, **(vp or {})), {**HP, **(hp or {})}, topo, preset, pts)


def test_snapshot_flattens_and_normalizes():
    s = snap(pts={"st": [1, 2, 3]}, topo="macpherson_strut")
    assert s[("vp", "mass")] == 280.0 and s[("vp", "use_spring_rates")] is False and s[("vp", "name")] == "car"
    assert ("vp", "nan_field") not in s and ("vp", "tire_coeffs") not in s       # not comparable scalars
    assert s[("hp", "upper_outer")] == (0.0, 540.0, 300.0)
    assert s[("ss", "topology")] == "macpherson_strut" and s[("topo", "macpherson_strut/st")] == (1.0, 2.0, 3.0)
    assert ("vp", "odd") not in r.snapshot({"odd": object}, {}, "t", "p")         # not a number: not tracked


def test_diff_reports_only_real_changes():
    a = snap()
    assert r.diff(a, snap(vp={"mass": 280.0 + 1e-12})) == []                       # float noise is not a change
    ch = r.diff(a, snap(vp={"mass": 290.0}, hp={"upper_outer": [0.0, 540.0, 305.0], "pushrod_outer": None}))
    assert [p for p, _a, _b in ch] == [("hp", "pushrod_outer"), ("hp", "upper_outer"), ("vp", "mass")]
    assert ch[2][1:] == (280.0, 290.0)
    assert r.diff({("vp", "x"): (1.0, 2.0)}, {("vp", "x"): (1.0, 2.0, 3.0)})          # length change counts


def test_record_merges_a_drag_and_drops_a_round_trip():
    feed, nid = [], 1
    nid = r.record(feed, [], None, None, 0.0, nid)
    assert feed == [] and nid == 1
    nid = r.record(feed, [(("vp", "mass"), 280.0, 285.0)], {"m": 1}, {"m": 2}, 100.0, nid)
    nid = r.record(feed, [(("vp", "mass"), 285.0, 290.0)], {"m": 2}, {"m": 3}, 110.0, nid)
    assert len(feed) == 1 and feed[0].changes == [(("vp", "mass"), 280.0, 290.0)]
    assert feed[0].m_before == {"m": 1} and feed[0].m_after == {"m": 3} and nid == 2
    # back to where it started, still within the window: the entry disappears
    nid = r.record(feed, [(("vp", "mass"), 290.0, 280.0)], {"m": 3}, {"m": 1}, 115.0, nid)
    assert feed == []


def test_record_starts_a_new_entry_for_other_inputs_late_edits_or_after_undo():
    feed, nid = [], 1
    nid = r.record(feed, [(("vp", "mass"), 280.0, 285.0)], None, None, 0.0, nid)
    nid = r.record(feed, [(("vp", "cg_height"), 300.0, 290.0)], None, None, 1.0, nid)     # other input
    nid = r.record(feed, [(("vp", "cg_height"), 290.0, 280.0)], None, None, 100.0, nid)   # too late to merge
    feed[0].undone = True
    nid = r.record(feed, [(("vp", "cg_height"), 280.0, 270.0)], None, None, 101.0, nid)   # top is undone
    assert [e.id for e in feed] == [4, 3, 2, 1] and nid == 5
    for i in range(20):
        nid = r.record(feed, [(("vp", f"k{i}"), 0.0, 1.0)], None, None, 200.0 + i, nid)
    assert len(feed) == r.MAX_ENTRIES


def test_undo_targets_put_back_only_that_entry_and_skip_new_generic_points():
    e = r.Entry(1, 0.0, [(("vp", "mass"), 280.0, 290.0), (("hp", "pushrod_outer"), (1.0, 2.0, 3.0), None),
                         (("ss", "topology"), "double_wishbone", "macpherson_strut"),
                         (("topo", "macpherson_strut/st"), None, (1.0, 2.0, 3.0)),
                         (("topo", "macpherson_strut/lca"), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))])
    assert r.undo_targets(e) == [(("vp", "mass"), 280.0), (("hp", "pushrod_outer"), (1.0, 2.0, 3.0)),
                                 (("ss", "topology"), "double_wishbone"),
                                 (("topo", "macpherson_strut/lca"), (0.0, 0.0, 0.0))]


def test_progress_counts_inputs_that_are_yours():
    ex = snap()
    assert r.progress(ex, ex, POINTS, ROCKER) == (0, 7, ("mass", "Mass"))
    cur = snap(vp={"mass": 300.0, "use_spring_rates": True}, hp={"static_toe": 0.1, "pushrod_outer": None})
    done, total, nxt = r.progress(cur, ex, POINTS, ROCKER, confirmed={"cg"})
    assert (done, total, nxt) == (5, 7, ("split", "Weight split"))      # mass, cg (confirmed), springs, camber, rocker
    allmine = snap(vp={"mass": 1.0, "cg_height": 1.0, "weight_dist_front": 0.5, "roll_stiffness_rear": 1.0},
                   hp={"static_camber": -2.0, "pushrod_outer": None}, topo="macpherson_strut")
    assert r.progress(allmine, ex, POINTS, ROCKER) == (7, 7, None)       # a topology switch counts as points
