"""suspension/frame_fe.py against textbook beam results: a cantilever's tip deflection and slope under a force and a
moment (PL^3/3EI, PL^2/2EI, ML^2/2EI, ML/EI), its stretch PL/EA and twist TL/GJ, in every orientation the element
frame handles; rigid-body modes in the stiffness null space; Maxwell-Betti symmetry; and the STEP-axis frame builder
on small frames whose joints are known."""
import numpy as np
import pytest

from suspension import frame_fe as fe

SEC = fe.Section(25.4, 1.65)
E, G = fe.E_STEEL, fe.G_STEEL


def test_tube_section_properties():
    ro, ri = 12.7, 12.7 - 1.65
    assert SEC.A == pytest.approx(np.pi * (ro ** 2 - ri ** 2))
    assert SEC.I == pytest.approx(np.pi / 4 * (ro ** 4 - ri ** 4))
    assert SEC.J == pytest.approx(2 * SEC.I)
    assert (E, G) == (205000.0, 79500.0)


def _cantilever(direction, L=500.0, max_el=None):
    fr = fe.Frame()
    a = fr.add_node([10.0, -20.0, 30.0])
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    b = fr.add_node(np.array([10.0, -20.0, 30.0]) + L * d)
    fr.add_elem(a, b, SEC, "arm", max_el)
    return fr, a, b, d


def _perp(d):
    p = np.cross(d, [0.3, 0.5, 0.8])
    return p / np.linalg.norm(p)


@pytest.mark.parametrize("direction", [[1, 0, 0], [0, 0, 1], [0, 0, -1], [0.3, -0.8, 0.5], [0.2, 0.1, 0.97]])
@pytest.mark.parametrize("max_el", [None, 120.0])
def test_a_cantilever_bends_stretches_and_twists_by_the_textbook(direction, max_el):
    L, P, M = 500.0, 100.0, 2.0e4
    fr, a, b, d = _cantilever(direction, L, max_el)
    if max_el:
        assert len(fr.elems) == 5 and len(fr.nodes) == 6       # ceil(500 / 120) elements
    fixed = {a: range(6)}
    n = _perp(d)
    u = fr.solve({b: np.r_[P * n, 0, 0, 0]}, fixed)[b]
    assert u[:3] == pytest.approx(P * L ** 3 / (3 * E * SEC.I) * n, rel=1e-9, abs=1e-12)
    assert u[3:] == pytest.approx(P * L ** 2 / (2 * E * SEC.I) * np.cross(d, n), rel=1e-9, abs=1e-12)
    u = fr.solve({b: np.r_[P * d, 0, 0, 0]}, fixed)[b]
    assert u[:3] == pytest.approx(P * L / (E * SEC.A) * d, rel=1e-9, abs=1e-12)
    u = fr.solve({b: np.r_[0, 0, 0, M * d]}, fixed)[b]
    assert u[3:] == pytest.approx(M * L / (G * SEC.J) * d, rel=1e-9, abs=1e-12)
    m = np.cross(d, n)
    u = fr.solve({b: np.r_[0, 0, 0, M * m]}, fixed)[b]
    assert u[3:] == pytest.approx(M * L / (E * SEC.I) * m, rel=1e-9, abs=1e-12)
    assert u[:3] == pytest.approx(M * L ** 2 / (2 * E * SEC.I) * n, rel=1e-9, abs=1e-12)


def test_the_flexibility_is_the_solve_under_unit_loads_and_symmetric():
    fr, a, b, d = _cantilever([0.3, -0.8, 0.5], 400.0, 100.0)
    mid = 2
    C = fr.condensed_compliance([b, mid], {a: range(6)})
    assert C.shape == (6, 6) and np.allclose(C, C.T, rtol=1e-9, atol=1e-15)
    for k in range(6):
        f = np.zeros(6)
        f[k % 3] = 1.0
        u = fr.solve({(b, mid)[k // 3]: np.r_[f[:3], 0, 0, 0]}, {a: range(6)})
        assert C[:, k] == pytest.approx(np.r_[u[b, :3], u[mid, :3]], rel=1e-9, abs=1e-15)


def test_rigid_body_modes_do_no_work():
    fr, a, b, d = _cantilever([0.3, -0.8, 0.5], 400.0, 100.0)
    fr.add_elem(fr.add_node([0.0, 300.0, 0.0]), a, fe.Section(19.05, 0.9))
    K = fr.stiffness()
    for x0 in (None, [100.0, -50.0, 20.0]):
        R = fr.rigid_modes(x0)
        assert np.abs(K @ R).max() < 1e-6 * np.abs(K).max()
    R = fr.rigid_modes([1.0, 2.0, 3.0])
    p = fr.nodes[1] - np.array([1.0, 2.0, 3.0])
    assert R[6:9, 0:3] == pytest.approx(np.eye(3))
    assert R[6:9, 5] == pytest.approx(np.cross([0, 0, 1.0], p)) and R[9:12, 3:6] == pytest.approx(np.eye(3))
    assert fr.rigid_modes()[6:9, 3] == pytest.approx(np.cross([1.0, 0, 0], fr.nodes[1]))


def test_elements_need_length_and_split_evenly():
    fr = fe.Frame()
    a, b = fr.add_node([0, 0, 0]), fr.add_node([0, 0, 1e-10])
    with pytest.raises(ValueError, match="^zero-length element$"):
        fr.add_elem(a, b, SEC)
    c = fr.add_node([300.0, 0, 0])
    fr.add_elem(a, c, SEC, "x", 100.0)
    assert [tuple(e[:2]) for e in fr.elems] == [(0, 3), (3, 4), (4, 2)] and {e[3] for e in fr.elems} == {"x"}
    assert fr.nodes[3] == pytest.approx([100.0, 0, 0]) and fr.nodes[4] == pytest.approx([200.0, 0, 0])
    fr.add_elem(a, c, SEC, "y", 300.0)
    assert fr.elems[-1] == (0, 2, SEC, "y")
    fr.add_elem(a, c, SEC)
    assert fr.elems[-1][3] == "" and fr.nodes[0].dtype == np.float64
    assert fr.length_mm() == pytest.approx(900.0)
    assert fr.nearest([90.0, 5.0, 0.0]) == 3 and fr.nearest([90.0, 5.0, 0.0], among=[0, 2]) == 0


def test_tube_mass_is_lumped_half_to_each_end_and_the_extra_is_placed_at_the_cg():
    fr = fe.Frame()
    n = [fr.add_node(p) for p in ([0, 0, 0], [0, 1000.0, 0], [0, 1000.0, 600.0], [0, 0, 600.0])]
    for i in range(4):
        fr.add_elem(n[i], n[(i + 1) % 4], SEC)
    tube = SEC.A * 7.85e-6
    m = fr.lumped_mass()
    assert m == pytest.approx([0.5 * tube * (1000 + 600)] * 4)
    assert fr.lumped_mass(density=1e-6) == pytest.approx(m * 1e-6 / 7.85e-6)
    even = fr.lumped_mass(extra_kg=40.0)
    assert even.sum() == pytest.approx(m.sum() + 40.0) and even == pytest.approx(m + 10.0)
    target = np.array([0.0, 650.0, 250.0])
    placed = fr.lumped_mass(extra_kg=40.0, cg=target)
    X = np.array(fr.nodes)
    assert placed.sum() == pytest.approx(m.sum() + 40.0)
    assert (placed[:, None] * X).sum(0)[1:] / placed.sum() == pytest.approx(target[1:], abs=1e-3)
    with pytest.raises(ValueError, match="lumped_mass: centre of mass .* cannot reach"):
        fr.lumped_mass(extra_kg=1.0, cg=[0.0, 5000.0, 250.0])


def test_inertia_relief_gives_the_free_bar_its_stretch():
    fr = fe.Frame()
    a = fr.add_node([0.0, 0.0, 0.0])
    b = fr.add_node([800.0, 0.0, 10.0])
    c = fr.add_node([400.0, 300.0, 400.0])
    for i, j in ((a, b), (b, c), (c, a)):
        fr.add_elem(i, j, SEC, max_el_mm=100.0)
    m = fr.lumped_mass(extra_kg=20.0)
    C = fr.inertia_relief_flexibility([a, b], m)
    assert C.shape == (6, 6) and np.allclose(C, C.T, rtol=1e-6, atol=1e-12)
    # equal and opposite forces along a-b stretch the triangle by what a fixed-a solve gives
    d = (fr.nodes[b] - fr.nodes[a]) / np.linalg.norm(fr.nodes[b] - fr.nodes[a])
    e = np.r_[-d, d]
    fixed = fr.solve({a: np.r_[-d, 0, 0, 0], b: np.r_[d, 0, 0, 0]}, {c: range(6)})
    assert e @ C @ e == pytest.approx((fixed[b, :3] - fixed[a, :3]) @ d, rel=1e-6)


def test_components_and_attachments():
    fr = fe.Frame()
    a, b = fr.add_node([0, 0, 0]), fr.add_node([100.0, 0, 0])
    c, d = fr.add_node([0, 500.0, 0]), fr.add_node([0, 600.0, 0])
    e = fr.add_node([0, 900.0, 0])
    fr.add_elem(a, b, SEC)
    fr.add_elem(c, d, SEC)
    fr.add_elem(d, e, SEC)
    assert fr.components() == [[2, 3, 4], [0, 1]]
    k = fr.attach([50.0, 0, 120.0], [a, b], SEC)
    assert k == 5 and fr.components()[1] == [0, 1, 5] or sorted(map(sorted, fr.components()))[0] == [0, 1, 5] + \
        [n for n in range(6, len(fr.nodes))]
    assert {t for *_, t in fr.elems[3:]} == {"bay"}
    n_before = len(fr.elems)
    fr.attach([50.0, 0, -500.0], [a], SEC, tag="tab", max_el_mm=None)
    assert len(fr.elems) == n_before + 1 and fr.elems[-1][3] == "tab"
    n_before = len(fr.elems)
    fr.attach([50.0, 0, -500.0], [b], SEC, tag="tab2")                     # 50 mm default subdivision
    assert len(fr.elems) - n_before == int(np.ceil(np.linalg.norm([50.0, 0, -500.0 - 0.0] - np.array([100.0, 0, 0]))
                                                   / 50.0))


def test_segment_distance_is_clipped_to_the_segment():
    a, b = np.array([0.0, 0, 0]), np.array([10.0, 0, 0])
    assert fe._segdist(np.array([4.0, 3.0, 0]), a, b) == pytest.approx((3.0, 0.4))
    assert fe._segdist(np.array([-3.0, 4.0, 0]), a, b) == pytest.approx((5.0, 0.0))
    assert fe._segdist(np.array([13.0, 4.0, 0]), a, b) == pytest.approx((5.0, 1.0))


def test_short_elements_are_collapsed_into_one_node():
    fr = fe.Frame()
    for p in ([0, 0, 0], [0.5, 0, 0], [200.0, 0, 0], [200.0, 0.0, 0.99999], [400.0, 0, 0]):
        fr.add_node(p)
    fr.elems = [(0, 1, SEC, "s"), (1, 2, SEC, "a"), (2, 3, SEC, "s2"), (3, 4, SEC, "b")]
    fe._collapse_short(fr, 1.0)
    assert len(fr.nodes) == 3 and [(i, j, t) for i, j, _, t in fr.elems] == [(0, 1, "a"), (1, 2, "b")]
    assert fr.nodes[0] == pytest.approx([0.0, 0, 0]) or fr.nodes[0] == pytest.approx([0.5, 0, 0])


def test_step_axes_become_a_frame_with_merged_ends_tees_and_bends():
    # a 600 x 400 rectangle whose corners meet within 3 mm, a cross tube teeing onto the middle of the far side,
    # a tube ending 60 mm from a corner (a bend chord), and a tube far from everything
    axes = [((0, 0, 0), (600, 0, 0), 1.65), ((601, 2, 0), (600, 400, 0), None), ((600, 400, 0), (0, 400, 0), 1.65),
            ((0, 402, 0), (0, 1, 0), 1.65),
            ((300, 400, 0), (300, 200, 0), 1.24),                 # tees onto tube 0? no: starts on tube 2's span
            ((-60, 0, 0), (-60, -300, 0), 1.65),                   # its end is 60 mm from corner (0, 0): a bend
            ((5000, 0, 0), (5000, 300, 0), 1.65)]                  # unjoined: its own component
    od = [25.4, 25.4, 25.4, 25.4, 19.05, 25.4, 25.4]
    fr, rep = fe.from_step_axes(axes, od, max_el_mm=1000.0)
    assert rep["tubes"] == 7 and rep["tees"] == 1 and rep["bend_chords"] == 1
    assert rep["merged_ends"] == 8 and rep["unjoined_ends"] == 4          # tube 5's far end, 6's two, 7's far end
    assert rep["components"][0] > rep["components"][-1] and rep["dropped_tubes"] == ["T07"]
    assert rep["nodes"] == len(fr.nodes) and rep["elements"] == len(fr.elems)
    assert rep["beam_length_m"] == pytest.approx(fr.length_mm() / 1000)
    tags = [t for *_, t in fr.elems]
    assert tags.count("tee") == 0 and tags.count("bend") == 1        # the tee end lands on its node: no link needed
    walls = {t: s.wall_mm for *_, s, t in fr.elems}
    assert walls["T02"] == 1.65 and walls["T05"] == 1.24                # a missing wall reads as 1.65 mm
    host = [n for n in range(len(fr.nodes)) if np.linalg.norm(fr.nodes[n] - [300.0, 400.0, 0]) < 1e-6]
    assert len(host) == 1
    assert all(np.linalg.norm(p - [5000.0, 150.0, 0]) > 100 for p in fr.nodes)
    fr2, rep2 = fe.from_step_axes(axes[:4], od[:4])
    assert "dropped_tubes" not in rep2 and rep2["components"] == [len(fr2.nodes)]
    assert max(np.linalg.norm(fr2.nodes[j] - fr2.nodes[i]) for i, j, *_ in fr2.elems) <= 50.0 + 1e-9


def test_a_tee_link_is_added_when_the_end_stops_short_of_the_host():
    axes = [((0, 0, 0), (1000, 0, 0), 1.65), ((500, 20, 0), (500, 400, 0), 1.24)]
    fr, rep = fe.from_step_axes(axes, [25.4, 19.05], max_el_mm=2000.0)
    assert rep["tees"] == 1 and rep["unjoined_ends"] == 3                 # both ends of the host, the far end
    tee = [(i, j, s) for i, j, s, t in fr.elems if t == "tee"]
    assert len(tee) == 1 and tee[0][2].od_mm == 19.05 and tee[0][2].wall_mm == 1.24
    i, j, _ = tee[0]
    assert {tuple(np.round(fr.nodes[i])), tuple(np.round(fr.nodes[j]))} == {(500.0, 20.0, 0.0), (500.0, 0.0, 0.0)}
    near_end = [((0, 0, 0), (1000, 0, 0), 1.65), ((10, 20, 0), (10, 400, 0), 1.24)]  # lands at t = 0.01: not a tee
    assert fe.from_step_axes(near_end, [25.4, 19.05], bend_mm=10.0)[1]["tees"] == 0
    far = [((0, 0, 0), (1000, 0, 0), 1.65), ((500, 36, 0), (500, 400, 0), 1.24)]      # 36 mm off: beyond tee_mm
    assert fe.from_step_axes(far, [25.4, 19.05])[1]["tees"] == 0
    assert fe.from_step_axes(far, [25.4, 19.05], tee_mm=40.0)[1]["tees"] == 1
    gap = [((0, 0, 0), (600, 0, 0), 1.65), ((606, 0, 0), (900, 300, 0), 1.65)]        # ends 6 mm apart
    assert fe.from_step_axes(gap, [25.4, 25.4])[1]["merged_ends"] == 0
    assert fe.from_step_axes(gap, [25.4, 25.4], merge_mm=6.0)[1]["merged_ends"] == 2
    bend = [((0, 0, 0), (600, 0, 0), 1.65), ((680, 0, 0), (900, 300, 0), 1.65)]        # 80 mm apart: a bend
    assert fe.from_step_axes(bend, [25.4, 25.4])[1]["bend_chords"] == 1
    assert fe.from_step_axes(bend, [25.4, 25.4], bend_mm=79.0)[1]["bend_chords"] == 0


def _pinv_relief(fr, points, mass):
    """Inertia-relief flexibility by the pseudo-inverse: balance the load by inertia, solve, remove rigid motion."""
    n = 6 * len(fr.nodes)
    K = fr.stiffness()
    Md = np.zeros(n)
    for i, m in enumerate(mass):
        Md[6 * i:6 * i + 3] = m
    R = fr.rigid_modes()
    A = R.T @ (Md[:, None] * R)
    cols = [6 * p + d for p in points for d in range(3)]
    F = np.zeros((n, len(cols)))
    for c, d in enumerate(cols):
        F[d, c] = 1.0
    Fb = F - (Md[:, None] * R) @ np.linalg.solve(A, R.T @ F)
    U = np.linalg.pinv(K, rcond=1e-10) @ Fb
    U -= R @ np.linalg.solve(A, R.T @ (Md[:, None] * U))
    return U[cols, :]


def test_inertia_relief_matches_the_pseudo_inverse_for_an_unbalanced_load():
    fr = fe.Frame()
    pts = [[0.0, 0.0, 0.0], [800.0, 0.0, 10.0], [400.0, 300.0, 400.0], [300.0, -200.0, 150.0]]
    n = [fr.add_node(p) for p in pts]
    for i, j in ((0, 1), (1, 2), (2, 0), (3, 0), (3, 1), (3, 2)):
        fr.add_elem(n[i], n[j], SEC, max_el_mm=150.0)
    m = fr.lumped_mass(extra_kg=12.0)
    m[2] += 30.0                                         # a heavy node: the inertia that balances a load depends on it
    C = fr.inertia_relief_flexibility([n[0], n[3]], m)
    assert C == pytest.approx(_pinv_relief(fr, [n[0], n[3]], m), rel=1e-5, abs=1e-12)


def test_the_steps_sections_tee_splits_and_bends():
    axes = [((0, 0, 0), (1000, 0, 0), 1.65),                                 # host, two tees land on it
            ((300, 15, 0), (300, 400, 0), 1.24), ((700, -12, 0), (700, -400, 0), 0.9),
            ((1060, 0, 0), (1300, 200, 0), 2.0)]                             # its end is 60 mm from the host's end
    od = [25.4, 19.05, 15.875, 31.75]
    fr, rep = fe.from_step_axes(axes, od, max_el_mm=5000.0)
    assert rep["tees"] == 2 and rep["bend_chords"] == 1
    host = [(i, j) for i, j, s, t in fr.elems if t == "T01"]
    assert len(host) == 3                                                    # split at both tee nodes
    assert sum(np.linalg.norm(fr.nodes[j] - fr.nodes[i]) for i, j in host) == pytest.approx(1000.0)
    secs = {t: (s.od_mm, s.wall_mm) for *_, s, t in fr.elems if t.startswith("T")}
    assert secs == {"T01": (25.4, 1.65), "T02": (19.05, 1.24), "T03": (15.875, 0.9), "T04": (31.75, 2.0)}
    tee = sorted(((s.od_mm, s.wall_mm), round(float(np.linalg.norm(fr.nodes[j] - fr.nodes[i])), 6))
                 for i, j, s, t in fr.elems if t == "tee")
    assert tee == [((15.875, 0.9), 12.0), ((19.05, 1.24), 15.0)]
    bend = [(s.od_mm, s.wall_mm) for *_, s, t in fr.elems if t == "bend"]
    assert bend == [(31.75, 2.0)] or bend == [(25.4, 1.65)]


def test_mutual_bends_make_one_chord_and_short_links_survive_the_collapse():
    axes = [((0, 0, 0), (500, 0, 0), 1.65), ((560, 0, 0), (900, 300, 0), 1.65),        # ends 60 mm apart, each other's
            ((0, 2000, 0), (500, 2000, 0), 1.65), ((560, 2000, 0), (900, 2300, 0), 1.65)]   # nearest: one chord each
    fr, rep = fe.from_step_axes(axes, [25.4] * 4, max_el_mm=5000.0)
    assert rep["bend_chords"] == 2 and [t for *_, t in fr.elems].count("bend") <= 2
    near = [((0, 0, 0), (1000, 0, 0), 1.65), ((500, 1.5, 0), (500, 400, 0), 1.24)]     # 1.5 mm off the axis
    fr, rep = fe.from_step_axes(near, [25.4, 19.05], max_el_mm=5000.0)
    assert [t for *_, t in fr.elems].count("tee") == 1                       # longer than the 1 mm collapse


def test_an_end_near_the_host_end_is_a_bend_not_a_tee():
    axes = [((0, 0, 0), (1000, 0, 0), 1.65), ((990, 10, 0), (990, 400, 0), 1.24)]     # lands at t = 0.99
    fr, rep = fe.from_step_axes(axes, [25.4, 19.05], max_el_mm=5000.0)
    assert rep["tees"] == 0 and rep["bend_chords"] == 1


def test_small_elements_fixed_nodes_and_attachment_sections():
    fr = fe.Frame()
    a, b = fr.add_node([0, 0, 0]), fr.add_node([0.5, 0, 0])
    fr.add_elem(a, b, SEC)
    assert len(fr.elems) == 1
    fr2, a2, b2, d2 = _cantilever([1, 0, 0], 300.0, 100.0)
    C1 = fr2.condensed_compliance([b2], {a2: range(6)})
    mid = 2                                          # an interior subdivision node (0 is the root, 1 the tip)
    C2 = fr2.condensed_compliance([b2], {a2: range(6), mid: range(6)})
    assert not np.allclose(C1, C2)
    u = fr2.solve({b2: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]}, {a2: range(6), mid: range(6)})
    assert C2[1, 1] == pytest.approx(u[b2, 1])
    tab = fe.Section(10.0, 2.0)
    k = fr2.attach([150.0, -20.0, 80.0], [a2], tab, max_el_mm=None)
    assert fr2.elems[-1][2] == tab and fr2.elems[-1][:2] == (k, a2)


def test_a_chain_of_bends_keeps_every_chord_after_a_mutual_pair():
    axes = [((0, 0, 0), (100, 0, 0), 1.65), ((120, 0, 0), (220, 0, 0), 1.65), ((240, 0, 0), (340, 0, 0), 1.65)]
    fr, rep = fe.from_step_axes(axes, [25.4] * 3, max_el_mm=5000.0)
    assert rep["bend_chords"] == 2 and rep["components"] == [len(fr.nodes)]
    assert [t for *_, t in fr.elems].count("bend") == 2
