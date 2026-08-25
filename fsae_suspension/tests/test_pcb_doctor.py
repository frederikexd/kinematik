# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""PCB Doctor — parse a real board file, diagnose real-life failures, name the
component, patch the traces in place, and verify the patched file re-parses."""

import math
import re
import unittest

from suspension.pcb_doctor import (
    parse_kicad_pcb, parse_board, sniff_format, demo_kicad_pcb,
    auto_assign_net_currents, diagnose, apply_fixes, fix_report_md,
    prescribe_trace, required_width_mm, vias_needed, find_diff_pairs,
    board_svg, clearance_required_mm, declare_net_current,
    trace_ampacity_a, NetAssignment, NODE_WELD_MM, PcbZone,
    PcbSegment, analyze_net)
from suspension.pcb_altium_binary import (
    layer_from_code, is_available as _bin_available)
from suspension.pcb_altium import (
    parse_altium_ascii, demo_altium_pcb, altium_layer_to_kicad,
    is_altium_binary, OLE_MAGIC)
from suspension.interfaces import Severity


def _demo_setup(fan_a=8.0):
    board = parse_kicad_pcb(demo_kicad_pcb())
    assignments = auto_assign_net_currents(board, ledger=None)
    fan = board.net_id("FAN_PWR")
    declare_net_current(assignments, fan, fan_a)
    hv = board.net_id("HV_INV_SENSE")
    assignments[hv]["voltage_v"] = 400.0
    return board, assignments


class TestParser(unittest.TestCase):
    def test_parses_everything(self):
        board = parse_kicad_pcb(demo_kicad_pcb())
        self.assertIn("FAN_PWR", board.nets.values())
        self.assertGreaterEqual(len(board.segments), 10)
        self.assertEqual(len(board.vias), 2)
        refs = {fp.ref for fp in board.footprints}
        self.assertTrue({"J1", "U1", "C1", "F1", "U2", "J2"} <= refs)
        self.assertIn("In1.Cu", board.copper_layers)
        self.assertAlmostEqual(board.board_thickness_mm, 1.6)

    def test_rejects_non_board(self):
        with self.assertRaises(ValueError):
            parse_kicad_pcb("(schematic (net 1))")

    def test_width_spans_point_at_the_width(self):
        board = parse_kicad_pcb(demo_kicad_pcb())
        for s in board.segments:
            a, b = s.width_span
            self.assertEqual(float(board.text[a:b]), s.width_mm)


class TestPrescriber(unittest.TestCase):
    def test_width_grows_with_current_and_shrinks_with_copper(self):
        w1 = required_width_mm(5.0, 20.0, 1.0, external=True)
        w2 = required_width_mm(10.0, 20.0, 1.0, external=True)
        w3 = required_width_mm(5.0, 20.0, 2.0, external=True)
        self.assertGreater(w2, w1)
        self.assertLess(w3, w1)
        # inner layers need more copper than outer
        self.assertGreater(required_width_mm(5.0, 20.0, 1.0, external=False), w1)

    def test_prescription_shape(self):
        p = prescribe_trace(8.0, dT_c=20.0, length_mm=120.0)
        self.assertEqual(len(p["rows"]), 6)
        self.assertGreaterEqual(p["vias_per_transition"], 2)
        self.assertGreater(vias_needed(8.0, 0.3, 20.0), 1)

    def test_clearance_table(self):
        self.assertAlmostEqual(clearance_required_mm(12), 0.1)
        self.assertAlmostEqual(clearance_required_mm(400), 2.5)
        self.assertGreater(clearance_required_mm(600), 2.5)


class TestDiagnosis(unittest.TestCase):
    def test_finds_the_planted_failures(self):
        board, assignments = _demo_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        checks = " | ".join(f.check for f in rep.findings)
        self.assertIn("trace ampacity — FAN_PWR", checks)
        self.assertIn("via bottleneck — FAN_PWR", checks)
        self.assertIn("fusing margin — FAN_PWR", checks)
        self.assertIn("component — C1", checks)      # cap on hot copper
        self.assertIn("component — F1", checks)      # 5 A fuse on an 8 A net
        self.assertIn("HV clearance — HV_INV_SENSE", checks)
        self.assertIn("HV coupling — CAN", checks)
        self.assertIn("diff pair skew — CAN", checks)
        self.assertTrue(any(f.severity == Severity.FAIL for f in rep.findings))
        self.assertTrue(rep.fixes)

    def test_quiet_board_is_ok(self):
        board, assignments = _demo_setup(fan_a=0.5)
        for nid in list(assignments):
            declare_net_current(assignments, nid, 0.2, voltage_v=5.0)
        rep = diagnose(board, assignments)
        hard = [f for f in rep.findings
                if f.severity == Severity.FAIL and "open" not in f.check]
        self.assertFalse(hard, [f.check for f in hard])

    def test_diff_pair_detection(self):
        board = parse_kicad_pcb(demo_kicad_pcb())
        pairs = find_diff_pairs(board)
        self.assertTrue(any(base.upper().startswith("CAN") for base, _, _ in pairs))

    def test_ir_drop_nodal_analysis_runs(self):
        board, assignments = _demo_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        nr = rep.net_reports[board.net_id("FAN_PWR")]
        self.assertIsNotNone(nr["worst_r_ohm"])
        self.assertGreater(nr["worst_r_ohm"], 0.0)
        self.assertFalse(nr["open_groups"])   # demo fan net is fully connected


class TestAutoFix(unittest.TestCase):
    def test_patched_file_reparses_with_wider_traces(self):
        board, assignments = _demo_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        self.assertTrue(applied)
        board2 = parse_kicad_pcb(patched)
        fan = board2.net_id("FAN_PWR")
        old_min = min(s.width_mm for s in board.segments_of(board.net_id("FAN_PWR")))
        new_min = min(s.width_mm for s in board2.segments_of(fan))
        self.assertGreater(new_min, old_min)
        # patched geometry clears ampacity at the same current
        assignments2 = dict(assignments)
        rep2 = diagnose(board2, assignments2)
        amp2 = [f for f in rep2.findings
                if f.check.startswith("trace ampacity — FAN_PWR")
                and f.severity == Severity.FAIL]
        self.assertFalse(amp2)
        # only widths changed: same net list, same segment count
        self.assertEqual(board.nets, board2.nets)
        self.assertEqual(len(board.segments), len(board2.segments))

    def test_diff_pair_members_never_auto_widened(self):
        board, assignments = _demo_setup()
        can = board.net_id("CAN_H")
        declare_net_current(assignments, can, 6.0)  # absurd: forces a finding
        rep = diagnose(board, assignments)
        pair_fixes = [fx for fx in rep.fixes if fx.nid == can]
        self.assertTrue(pair_fixes)
        self.assertTrue(all(not fx.auto for fx in pair_fixes))

    def test_fix_report_and_svg_render(self):
        board, assignments = _demo_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        md = fix_report_md(board, rep, applied, assignments)
        self.assertIn("Widths rewritten", md)
        self.assertIn("FAN_PWR", md)
        svg = board_svg(board, report=rep)
        self.assertTrue(svg.startswith("<svg"))
        self.assertIn("#ff3333", svg)   # failing copper is haloed


# =========================================================================== #
#  Altium: a second front-end, not a second Doctor
# =========================================================================== #
def _alt_setup(fan_a=8.0):
    board = parse_board(demo_altium_pcb(), "demo_ecu_board.PcbDoc")
    assignments = auto_assign_net_currents(board, ledger=None)
    declare_net_current(assignments, board.net_id("FAN_PWR"), fan_a)
    assignments[board.net_id("HV_INV_SENSE")]["voltage_v"] = 400.0
    return board, assignments


class TestSettingACurrentIsDeclaringIt(unittest.TestCase):
    """The MISSING rule creates a trap: set `a["current_a"] = 8.0` and leave the
    `assumed` flag standing, and the net keeps reporting "current not declared"
    while visibly holding the number you just gave it. Documentation does not
    reliably prevent that, so the type does — every write path clears the flag.
    """

    def _fresh(self):
        b = parse_kicad_pcb(demo_kicad_pcb())
        return b, auto_assign_net_currents(b, ledger=None), b.net_id("FAN_PWR")

    def test_plain_item_assignment_declares(self):
        b, a, fan = self._fresh()
        self.assertFalse(a[fan].declared)
        a[fan]["current_a"] = 8.0
        self.assertTrue(a[fan].declared)
        rep = diagnose(b, a)
        self.assertFalse([f for f in rep.findings
                          if f.check == "current not declared — FAN_PWR"])
        self.assertTrue([f for f in rep.findings
                         if f.check.startswith("trace ampacity — FAN_PWR")])

    def test_update_declares_too(self):
        """dict.update bypasses __setitem__ unless overridden — and the UI's
        restore-remembered-edits path goes through update()."""
        _, a, fan = self._fresh()
        a[fan].update({"current_a": 8.0})
        self.assertTrue(a[fan].declared)

    def test_helper_and_direct_write_agree(self):
        _, a1, fan = self._fresh()
        _, a2, _ = self._fresh()
        a1[fan]["current_a"] = 8.0
        declare_net_current(a2, fan, 8.0)
        self.assertEqual(a1[fan].declared, a2[fan].declared)
        self.assertEqual(a1[fan]["current_a"], a2[fan]["current_a"])

    def test_the_auto_assigner_can_still_guess(self):
        """The one path that must NOT declare: the assigner's own guesses."""
        _, a, fan = self._fresh()
        self.assertFalse(a[fan].declared)
        self.assertTrue(a[fan]["current_a"] > 0)

    def test_a_hand_built_plain_dict_counts_as_declared(self):
        """Callers outside this module build plain dicts; absence of the flag
        means someone stated the number, so they must get real verdicts."""
        b = parse_kicad_pcb(demo_kicad_pcb())
        fan = b.net_id("FAN_PWR")
        rep = diagnose(b, {fan: {"net": "FAN_PWR", "current_a": 8.0,
                                 "voltage_v": 13.5}})
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("current not declared")])

    def test_survives_a_deep_copy(self):
        """Streamlit hands these round through session state."""
        import copy
        _, a, fan = self._fresh()
        a2 = copy.deepcopy(a)
        self.assertIsInstance(a2[fan], NetAssignment)
        a2[fan]["current_a"] = 8.0
        self.assertTrue(a2[fan].declared)
        self.assertFalse(a[fan].declared, "the copy must not alias the original")


class TestUndeclaredCurrentIsMissingNotFail(unittest.TestCase):
    """The noise-floor rule. Assigning a default current and then failing the
    board against that default is a guess wearing the costume of a finding; on
    a real board most nets are unmatched, so it buries the real findings under
    invented ones and teaches people to ignore the tool."""

    def test_no_ledger_gives_missing_and_no_fixes(self):
        board = parse_kicad_pcb(demo_kicad_pcb())
        asg = auto_assign_net_currents(board, ledger=None)
        rep = diagnose(board, asg)
        self.assertTrue([f for f in rep.findings
                         if f.check.startswith("current not declared")])
        # nothing may be auto-re-traced off a guessed current
        self.assertEqual([fx for fx in rep.fixes if fx.auto], [])
        amp = [f for f in rep.findings
               if f.check.startswith(("trace ampacity", "fusing margin"))]
        self.assertEqual(amp, [], "guessed currents must not produce verdicts")

    def test_declaring_a_current_restores_the_verdict(self):
        board, assignments = _demo_setup(fan_a=8.0)     # declares FAN_PWR
        rep = diagnose(board, assignments)
        fan = [f for f in rep.findings if f.check.endswith("FAN_PWR")]
        self.assertTrue([f for f in fan
                         if f.check.startswith("trace ampacity")])
        self.assertFalse([f for f in fan
                          if f.check.startswith("current not declared")])
        self.assertTrue([fx for fx in rep.fixes if fx.auto])

    def test_geometry_checks_still_run_without_any_current(self):
        """Copper opens and clearance do not depend on current and must never
        be suppressed by a missing declaration."""
        txt = ('(kicad_pcb (version 20240108)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "SIG")\n'
               ' (footprint "a" (layer "F.Cu") (at 0 0)\n'
               '   (property "Reference" "J1")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
               ' (footprint "b" (layer "F.Cu") (at 50 0)\n'
               '   (property "Reference" "J2")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
               ' (segment (start 0 0) (end 10 0) (width 0.25)'
               ' (layer "F.Cu") (net 1))\n)')
        b = parse_kicad_pcb(txt)
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertTrue([f for f in rep.findings
                         if f.check.startswith("copper open")])

    def test_ampacity_inverts_required_width(self):
        for amps in (0.5, 2.0, 8.0):
            w = required_width_mm(amps, 20.0, 1.0, True)
            self.assertAlmostEqual(
                trace_ampacity_a(w, 20.0, 1.0, True), amps, places=3)


class TestSafetyAsymmetryHolds(unittest.TestCase):
    """The property the whole module is built around, tested directly.

    Every individual tolerance has its own tripwire, but a tripwire per constant
    only catches the constant someone thought to guard. This checks the
    *behaviour* instead: a board with a genuine, unambiguous break must still
    come back open, no matter which knob was touched to get there. If someone
    widens a tolerance, adds a new welding rule, or loosens containment, this is
    what fails.
    """

    def _clearly_broken_board(self):
        """Two pads, two stubs, a 5 mm gap between them, nothing else. There is
        no reading of this board on which those pads are connected."""
        return parse_kicad_pcb(
            '(kicad_pcb (version 20241229)\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
            ' (net 0 "") (net 1 "SIG")\n'
            ' (footprint "a" (layer "F.Cu") (at 0 0)\n'
            '   (property "Reference" "J1")\n'
            '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
            ' (footprint "b" (layer "F.Cu") (at 20 0)\n'
            '   (property "Reference" "J2")\n'
            '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
            ' (segment (start 0 0) (end 7 0) (width 0.25)'
            ' (layer "F.Cu") (net 1))\n'
            ' (segment (start 12 0) (end 20 0) (width 0.25)'
            ' (layer "F.Cu") (net 1))\n)')

    def test_a_real_break_is_never_waved_through(self):
        b = self._clearly_broken_board()
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertTrue(
            [f for f in rep.findings if f.check.startswith("copper open")],
            "\n\nSTOP. A board with a 5 mm gap in its only net just came back "
            "CLEAN.\n"
            "Some connectivity rule -- a tolerance, a weld, a pour, a via "
            "reach -- is now joining copper that is really apart. That is the "
            "one failure direction this tool is not allowed to have: it sends "
            "a dead board to a fab.\n"
            "See SAFETY_CONTRACT in suspension/pcb_doctor.py. Fix the rule, "
            "not this test.")

    def test_reported_resistance_is_never_lowered_by_a_pour(self):
        """Trace-only resistance is an upper bound on the real thing. Anything
        that lowers it makes IR drop look better than it is, which under-reports
        brown-out -- the same asymmetry, on the resistance side."""
        b = parse_kicad_pcb(demo_kicad_pcb())
        nid = b.net_id("FAN_PWR")
        base = analyze_net(b, nid)["worst_r_ohm"]
        b.zones.append(PcbZone(net=nid, layer="F.Cu",
                               outline=[(-50, -50), (200, -50),
                                        (200, 200), (-50, 200)]))
        b.zone_nets.add(nid)
        after = analyze_net(b, nid)["worst_r_ohm"]
        if base is not None and after is not None:
            self.assertGreaterEqual(
                after, base - 1e-12,
                "\n\nSTOP. A copper pour just LOWERED the reported "
                "resistance.\n"
                "Pours are meant to join the connectivity graph and stay out "
                "of the resistance solve, precisely so every resistance stays "
                "an upper bound. A smaller number here under-reports "
                "brown-out. See SAFETY_CONTRACT in suspension/pcb_doctor.py.")


class TestViaAnnulusConnects(unittest.TestCase):
    """A track does not have to hit a via's exact centre. A via is an annulus of
    copper and routers habitually stop a little short: measured on a real
    4-layer board, an inner-layer track ended 97 um from a 350 um via — on the
    via pad, but outside the 50 um endpoint weld — so the net split across
    layers and read "copper open" while being perfectly routed. That single
    cause was behind 49 of the corpus's 67 remaining false alarms."""

    def _board(self, offset_mm, via_size=0.35):
        """Pad on F.Cu -> via -> pad on B.Cu, with the bottom track ending
        `offset_mm` short of the via centre."""
        return parse_kicad_pcb(
            '(kicad_pcb (version 20241229)\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
            ' (net 0 "") (net 1 "SIG")\n'
            ' (footprint "a" (layer "F.Cu") (at 0 0)\n'
            '   (property "Reference" "J1")\n'
            '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
            ' (footprint "b" (layer "B.Cu") (at 20 0)\n'
            '   (property "Reference" "J2")\n'
            '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
            ' (segment (start 0 0) (end 10 0) (width 0.25)'
            ' (layer "F.Cu") (net 1))\n'
            f' (segment (start {10 + offset_mm} 0) (end 20 0) (width 0.25)'
            ' (layer "B.Cu") (net 1))\n'
            f' (via (at 10 0) (size {via_size}) (drill 0.15)'
            ' (layers "F.Cu" "B.Cu") (net 1))\n)')

    def _opens(self, board):
        rep = diagnose(board, auto_assign_net_currents(board, ledger=None))
        return [f for f in rep.findings if f.check.startswith("copper open")]

    def test_track_landing_on_the_annulus_is_connected(self):
        """97 um short of a 350 um via: on the pad, outside the weld."""
        self.assertFalse(self._opens(self._board(0.097)))

    def test_track_reaching_the_exact_centre_still_works(self):
        self.assertFalse(self._opens(self._board(0.0)))

    def test_a_track_well_clear_of_the_via_is_still_open(self):
        """The reach is the via's own copper radius, taken from the file — not
        a tolerance to widen. A track 2 mm away is not touching anything."""
        self.assertTrue(self._opens(self._board(2.0)))

    def test_reach_scales_with_the_via_not_a_constant(self):
        """The same 0.3 mm offset is connected on a big via and open on a tiny
        one, because the annulus is a fact from the file."""
        self.assertFalse(self._opens(self._board(0.3, via_size=0.8)))
        self.assertTrue(self._opens(self._board(0.3, via_size=0.2)))


class TestPourConnectivity(unittest.TestCase):
    """A pour is real copper and really does join pads. But a pour is also a
    sheet, and any resistance invented for it would make the IR drop look
    *smaller* than trace-only does — the dangerous direction, since it
    under-reports brown-out. So pours join the connectivity graph and are kept
    out of the resistance solve."""

    SQ = [(0, 0), (20, 0), (20, 20), (0, 20)]

    def _board(self):
        """Two pads on one net, each with a stub, joined only by a pour."""
        txt = ('(kicad_pcb (version 20241229)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "SIG")\n'
               ' (footprint "a" (layer "F.Cu") (at 2 2)\n'
               '   (property "Reference" "J1")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
               ' (footprint "b" (layer "F.Cu") (at 18 18)\n'
               '   (property "Reference" "J2")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
               # a stub off each pad, so the net counts as routed but the two
               # stubs come nowhere near each other
               ' (segment (start 2 2) (end 3 2) (width 0.25)'
               ' (layer "F.Cu") (net 1))\n'
               ' (segment (start 18 18) (end 17 18) (width 0.25)'
               ' (layer "F.Cu") (net 1))\n)')
        return parse_kicad_pcb(txt)

    def test_a_pour_joins_pads_nothing_else_reaches(self):
        b = self._board()
        nid = b.net_id("SIG")
        self.assertTrue([f for f in diagnose(
            b, auto_assign_net_currents(b, ledger=None)).findings
            if f.check.startswith("copper open")], "precondition: open first")
        b.zones.append(PcbZone(net=nid, layer="F.Cu", outline=self.SQ))
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("copper open")],
                         "the pour covers both pads, so the net is joined")

    def test_a_pad_outside_the_pour_is_not_joined(self):
        """Point-in-polygon must actually be tested — a pour that swallowed
        every pad on its net would be a false all-clear machine."""
        b = self._board()
        nid = b.net_id("SIG")
        far = [(100, 100), (110, 100), (110, 110), (100, 110)]
        b.zones.append(PcbZone(net=nid, layer="F.Cu", outline=far))
        # geometry only — adding to zone_nets would trip the blanket exemption
        # and mask the containment test this case exists for
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertTrue([f for f in rep.findings
                         if f.check.startswith("copper open")])

    def test_a_pour_on_another_layer_does_not_join(self):
        b = self._board()
        nid = b.net_id("SIG")
        b.zones.append(PcbZone(net=nid, layer="B.Cu", outline=self.SQ))
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertTrue([f for f in rep.findings
                         if f.check.startswith("copper open")])

    def test_pours_never_lower_the_reported_resistance(self):
        """The safety property. Trace-only resistance is an upper bound on the
        real thing; a pour must not be allowed to shrink it."""
        b = self._board()
        nid = b.net_id("SIG")
        b.segments.append(PcbSegment(net=nid, layer="F.Cu", width_mm=0.25,
                                     start=(3, 2), end=(17, 18)))
        base = analyze_net(b, nid)["worst_r_ohm"]
        b.zones.append(PcbZone(net=nid, layer="F.Cu", outline=self.SQ))
        withpour = analyze_net(b, nid)["worst_r_ohm"]
        if base is not None and withpour is not None:
            self.assertAlmostEqual(base, withpour, places=9)

    def test_polygon_containment(self):
        z = PcbZone(net=1, layer="F.Cu", outline=self.SQ)
        self.assertTrue(z.contains((10, 10)))
        self.assertFalse(z.contains((30, 10)))
        self.assertTrue(z.contains((20.02, 10), tol=0.05))   # on the edge


class TestCoincidentEndpointsAreWelded(unittest.TestCase):
    """Routers emit coordinates that disagree in the last micron, and rounding
    to a fixed grid drops the two sides of a joint into different buckets. On a
    real 4-layer board endpoints 11 um apart split a fully routed net into three
    islands and reported it "copper open" — 8.0% of routed nets corpus-wide,
    down to 4.7% once coincident endpoints are welded."""

    def _net(self, gap_mm):
        """Two collinear tracks meeting with `gap_mm` between their ends."""
        return ('(kicad_pcb (version 20241229)\n'
                ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
                ' (net 0 "") (net 1 "SIG")\n'
                ' (footprint "a" (layer "F.Cu") (at 0 0)\n'
                '   (property "Reference" "J1")\n'
                '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
                ' (footprint "b" (layer "F.Cu") (at 20 0)\n'
                '   (property "Reference" "J2")\n'
                '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG")))\n'
                f' (segment (start 0 0) (end 10 0) (width 0.25)'
                ' (layer "F.Cu") (net 1))\n'
                f' (segment (start {10 + gap_mm} 0) (end 20 0) (width 0.25)'
                ' (layer "F.Cu") (net 1))\n)')

    def _opens(self, gap_mm):
        b = parse_kicad_pcb(self._net(gap_mm))
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        return [f for f in rep.findings if f.check.startswith("copper open")]

    def test_a_micron_scale_gap_is_the_same_copper(self):
        """0.011 mm between two 0.25 mm traces is overlapping copper, not a
        break — and it is what real files actually contain."""
        self.assertFalse(self._opens(0.011))

    def test_a_real_gap_is_still_reported(self):
        """The tolerance must not weld genuine unrouted connections shut —
        that would be a false all-clear, the one error this tool must not make.
        A rats-nest gap is millimetres, two orders of magnitude clear of it."""
        self.assertTrue(self._opens(2.0))

    def test_the_tolerance_stays_small(self):
        """Tripwire. Guards against someone widening this to silence a false
        alarm. Routed traces are 100-500 um wide; above ~50 um the weld starts
        closing gaps that are really open, which turns a wasted afternoon into
        a dead board at the fab."""
        self.assertLessEqual(
            NODE_WELD_MM, 0.05,
            "\n\nSTOP. You have widened NODE_WELD_MM, almost certainly to "
            "silence a false 'copper open'.\n"
            "That trade is the wrong way round: this tool may cry wolf, but it "
            "may not wave a bad board through, and welding copper that is "
            "really apart does exactly that.\n"
            "The fix for a false open is a PHYSICAL reason the copper touches "
            "-- see the via-annulus rule, which reaches further than this and "
            "is safer because its distance comes from the via diameter in the "
            "file rather than from a number someone picked.\n"
            "Do not update this assertion. See SAFETY_CONTRACT in "
            "suspension/pcb_doctor.py.")


class TestPadsCarryTheirOwnCopperSide(unittest.TestCase):
    """A pad is not necessarily on its footprint's side. A card-edge connector
    is one top-side footprint whose fingers sit on both faces; designs also put
    an SMD pad on the far side of a top-side part. Inheriting the footprint's
    layer hides the trace that reaches such a pad and reports a routed net as
    "copper open" — this was the largest false-alarm source on a real 4-layer
    board (28 of 371 nets, all of them fine)."""

    EDGE = ('(kicad_pcb (version 20241229)\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
            ' (net 0 "") (net 1 "AD0")\n'
            ' (footprint "edge" (layer "F.Cu") (at 0 0)\n'
            '   (property "Reference" "BUS1")\n'
            '   (pad "A47" smd rect (at 10 0) (size 1 6)'
            '     (layers "B.Cu" "B.Mask") (net 1 "AD0")))\n'
            ' (footprint "u" (layer "F.Cu") (at 0 0)\n'
            '   (property "Reference" "U1")\n'
            '   (pad "1" smd rect (at 0 0) (size 1 1)'
            '     (layers "B.Cu" "B.Mask") (net 1 "AD0")))\n'
            ' (segment (start 0 0) (end 10 0) (width 0.2)'
            '  (layer "B.Cu") (net 1))\n)')

    def test_back_side_pad_on_a_front_side_footprint(self):
        b = parse_kicad_pcb(self.EDGE)
        pads = {p.number: p for f in b.footprints for p in f.pads}
        self.assertEqual(pads["A47"].layer, "B.Cu")
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("copper open")],
                         "a net routed on B.Cu to B.Cu pads is not open")

    def test_pad_spanning_both_sides_reaches_either(self):
        txt = self.EDGE.replace('(layers "B.Cu" "B.Mask")',
                                '(layers "*.Cu" "*.Mask")')
        b = parse_kicad_pcb(txt)
        for f in b.footprints:
            for p in f.pads:
                self.assertTrue(p.through, "*.Cu pads reach every layer")


class TestKicad10NetDialect(unittest.TestCase):
    """KiCad 10 dropped numeric net IDs: v5-9 wrote `(net 2 "VCC")` and declared
    every net up front, v10 writes `(net "VCC")` on each object and declares
    nothing. Parsing only the numeric form does not fail loudly on a v10 board —
    it drops EVERY segment and reports a board with no copper, which reads as an
    empty file rather than a bug."""

    V10 = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
           '\t(generator_version "10.0")\n'
           '\t(layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
           '\t(footprint "r"\n\t\t(layer "F.Cu")\n\t\t(at 0 0)\n'
           '\t\t(property "Reference" "R1")\n'
           '\t\t(pad "1" smd rect (at 0 0) (size 1 1) (net "VCC"))\n'
           '\t\t(pad "2" smd rect (at 10 0) (size 1 1) (net "GND"))\n\t)\n'
           '\t(segment (start 0 0) (end 10 0) (width 0.8) (layer "F.Cu")'
           ' (net "VCC"))\n)')

    def test_named_nets_are_interned(self):
        b = parse_kicad_pcb(self.V10)
        self.assertEqual(len(b.segments), 1, "v10 segments must not be dropped")
        self.assertEqual(b.net_name(b.segments[0].net), "VCC")
        self.assertIn("GND", b.nets.values())

    def test_pads_resolve_to_the_same_ids_as_segments(self):
        """A pad and a segment naming the same net must land on one id, or the
        net looks routed-but-padless and the open check goes haywire."""
        b = parse_kicad_pcb(self.V10)
        vcc = b.net_id("VCC")
        self.assertTrue(vcc)
        pads = [p for f in b.footprints for p in f.pads if p.net == vcc]
        self.assertEqual(len(pads), 1)
        self.assertEqual(b.segments[0].net, vcc)

    def test_numeric_form_still_works(self):
        b = parse_kicad_pcb(demo_kicad_pcb())
        self.assertEqual(b.net_name(b.net_id("FAN_PWR")), "FAN_PWR")
        self.assertEqual(len(b.segments), 12)


class TestFormatSniffing(unittest.TestCase):
    def test_content_beats_extension(self):
        """Teams rename exports constantly; an Altium board called board.txt
        is still an Altium board."""
        self.assertEqual(sniff_format(demo_kicad_pcb(), "whatever.txt"), "kicad")
        self.assertEqual(sniff_format(demo_altium_pcb(), "whatever.txt"),
                         "altium")

    def test_binary_pcbdoc_is_detected_and_unreadable_ones_say_why(self):
        """Native binary is now read, not refused outright — but a file that
        cannot be read must still fail with the ASCII export path attached,
        never a raw struct/OLE error the member cannot act on."""
        blob = OLE_MAGIC + b"\x00" * 1024
        self.assertTrue(is_altium_binary(blob))
        self.assertEqual(sniff_format(blob, "board.PcbDoc"), "altium_binary")
        with self.assertRaises(ValueError) as cm:
            parse_board(blob, "board.PcbDoc")
        self.assertIn("ASCII", str(cm.exception))

    def test_unknown_file_says_what_is_supported(self):
        with self.assertRaises(ValueError) as cm:
            parse_board("hello, world", "notes.txt")
        msg = str(cm.exception)
        self.assertIn("kicad_pcb", msg)
        self.assertIn("PcbDoc", msg)

    def test_layer_mapping(self):
        self.assertEqual(altium_layer_to_kicad("Top Layer"), "F.Cu")
        self.assertEqual(altium_layer_to_kicad("BOTTOM"), "B.Cu")
        self.assertEqual(altium_layer_to_kicad("MID3"), "In3.Cu")
        self.assertEqual(altium_layer_to_kicad("InternalPlane2"), "In2.Cu")
        # not copper — must never enter the ampacity mesh
        self.assertIsNone(altium_layer_to_kicad("TopOverlay"))
        self.assertIsNone(altium_layer_to_kicad("Mechanical1"))


class TestAltiumBinary(unittest.TestCase):
    """The native binary .PcbDoc is what Altium saves by default, so it is the
    file a member actually has. It is read for diagnosis but never patched."""

    def test_layer_codes(self):
        self.assertEqual(layer_from_code(1), "F.Cu")
        self.assertEqual(layer_from_code(32), "B.Cu")
        self.assertEqual(layer_from_code(3), "In2.Cu")
        self.assertEqual(layer_from_code(39), "In1.Cu")    # internal plane 1
        self.assertIsNone(layer_from_code(33))             # top overlay: silk
        self.assertIsNone(layer_from_code(0))

    def test_a_binary_board_is_never_patched(self):
        """Diagnosis is recoverable if the parse is wrong; a bad patch corrupts
        a file heading for a fab. So apply_fixes must refuse outright rather
        than write bytes into a format it only reads."""
        from suspension.pcb_doctor import PcbBoard, PcbSegment
        b = PcbBoard(fmt="altium_binary")
        b.patchable = False
        b.segments.append(PcbSegment(net=1, layer="F.Cu", width_mm=0.2,
                                     start=(0, 0), end=(1, 0)))
        with self.assertRaises(ValueError) as cm:
            apply_fixes(b, [])
        self.assertIn("ASCII", str(cm.exception))

    def test_a_truncated_binary_file_is_refused_not_reported_on(self):
        """A mis-parse yields confident nonsense, not an exception, so the
        reader checks its own output against reality before returning."""
        from suspension import pcb_altium_binary as _bin
        if not _bin_available():
            self.skipTest("olefile not installed")
        with self.assertRaises(ValueError):
            parse_board(OLE_MAGIC + b"\x00" * 4096, "broken.PcbDoc")


class TestAltiumParser(unittest.TestCase):
    def test_parses_everything(self):
        board = parse_board(demo_altium_pcb(), "demo.PcbDoc")
        self.assertEqual(board.fmt, "altium")
        self.assertEqual(board.length_unit, "mil")
        self.assertIn("FAN_PWR", board.nets.values())
        refs = {fp.ref for fp in board.footprints}
        self.assertTrue({"J1", "U1", "C1", "F1", "U2", "J2"} <= refs)
        self.assertEqual(len(board.vias), 2)

    def test_silkscreen_and_mechanical_are_not_copper(self):
        """The demo file carries a TopOverlay and a Mechanical1 track. If either
        reached the copper mesh it would fake a net and skew the geometry."""
        board = parse_board(demo_altium_pcb(), "demo.PcbDoc")
        self.assertEqual(len(board.segments), 12)
        self.assertTrue(all(s.layer in ("F.Cu", "B.Cu") for s in board.segments))

    def test_same_board_two_formats_one_diagnosis(self):
        """The Altium demo is the same ECU geometry as the KiCad one. Different
        file, different units, different Y axis — the findings must match, or
        the two front-ends have drifted apart."""
        kb, ka = _demo_setup(fan_a=8.0)
        ab, aa = _alt_setup(fan_a=8.0)
        krep, arep = diagnose(kb, ka), diagnose(ab, aa)
        self.assertEqual({f.check for f in krep.findings},
                         {f.check for f in arep.findings})
        self.assertEqual(krep.counts(), arep.counts())
        self.assertEqual(len(krep.fixes), len(arep.fixes))

    def test_geometry_survives_the_y_flip_and_unit_conversion(self):
        kb = parse_board(demo_kicad_pcb(), "d.kicad_pcb")
        ab = parse_board(demo_altium_pcb(), "d.PcbDoc")
        for name in ("FAN_PWR", "CAN_H", "LV_5V"):
            kl = sum(s.length_mm for s in kb.segments_of(kb.net_id(name)))
            al = sum(s.length_mm for s in ab.segments_of(ab.net_id(name)))
            self.assertAlmostEqual(kl, al, places=2, msg=name)

    def test_units_and_planes_and_loose_pads(self):
        """A metric export with an arc, an internal plane and a pad that
        belongs to no component — all of which real boards contain."""
        txt = ("|RECORD=Board|FILENAME=x.PcbDoc|\n"
               "|RECORD=Net|ID=0|NAME=GND|\n"
               "|RECORD=Net|ID=1|NAME=VBAT|\n"
               "|RECORD=Component|SOURCEDESIGNATOR=U9|COMMENT=Reg|"
               "LAYER=BOTTOM|X=10mm|Y=10mm|\n"
               "|RECORD=Pad|NAME=1|COMPONENT=0|LAYER=BOTTOM|NET=1|X=10mm|"
               "Y=10mm|XSIZE=1mm|YSIZE=1mm|HOLESIZE=0mm|\n"
               "|RECORD=Pad|NAME=T1|COMPONENT=-1|LAYER=MULTILAYER|NET=1|"
               "X=30mm|Y=10mm|XSIZE=2mm|YSIZE=2mm|HOLESIZE=0.8mm|\n"
               "|RECORD=Track|LAYER=MID2|NET=1|X1=10mm|Y1=10mm|X2=20mm|"
               "Y2=10mm|WIDTH=0.4mm|\n"
               "|RECORD=Polygon|LAYER=InternalPlane1|NET=0|\n")
        b = parse_altium_ascii(txt)
        self.assertEqual(b.length_unit, "mm")
        self.assertEqual(b.copper_layers, ["In1.Cu", "In2.Cu", "B.Cu"])
        self.assertEqual(b.native_layer("In2.Cu"), "Mid Layer 2")
        # net indices are 0-based against |RECORD=Net| order, GND first
        self.assertEqual(b.net_name(b.segments[0].net), "VBAT")
        self.assertIn(b.net_id("GND"), b.zone_nets)     # the plane's net
        self.assertEqual(b.footprints[0].layer, "B.Cu")
        loose = [p for f in b.footprints if f.ref == "?" for p in f.pads]
        self.assertEqual(len(loose), 1)
        self.assertTrue(loose[0].through)               # multilayer + a hole

    def test_internal_units_are_refused_not_silently_all_clear(self):
        """Bare numbers in Altium internal units read as mils inflate the board
        10,000x — and a 2540 mm trace is never undersized, so the board would
        come back ALL CLEAR without being checked. Refuse instead."""
        txt = ("|RECORD=Net|ID=0|NAME=VBAT|\n"
               "|RECORD=Track|LAYER=TOP|NET=0|X1=0|Y1=0|X2=10000000|Y2=0|"
               "WIDTH=100000|\n")
        with self.assertRaises(ValueError) as cm:
            parse_altium_ascii(txt)
        self.assertIn("internal", str(cm.exception).lower())

    def test_free_pads_reach_the_copper_graph_on_their_own_layer(self):
        """Altium allows pads belonging to no component. Lumping them into one
        synthetic top-side footprint hides every bottom-side one from the
        connectivity graph and fakes a 'copper open' on a good net."""
        txt = ("|RECORD=Net|ID=0|NAME=SIG|\n"
               "|RECORD=Pad|NAME=A|LAYER=BOTTOM|NET=0|X=0mm|Y=0mm|"
               "XSIZE=1mm|YSIZE=1mm|HOLESIZE=0mm|\n"
               "|RECORD=Pad|NAME=B|LAYER=BOTTOM|NET=0|X=10mm|Y=0mm|"
               "XSIZE=1mm|YSIZE=1mm|HOLESIZE=0mm|\n"
               "|RECORD=Track|LAYER=BOTTOM|NET=0|X1=0mm|Y1=0mm|X2=10mm|"
               "Y2=0mm|WIDTH=0.5mm|\n")
        b = parse_altium_ascii(txt)
        free = [p for f in b.footprints if f.ref == "?" for p in f.pads]
        self.assertEqual(len(free), 2)
        self.assertTrue(all(p.layer == "B.Cu" for p in free))
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("copper open")])

    def test_ampacity_message_is_a_string_not_a_tuple(self):
        """A stray trailing comma made every ampacity finding render as
        ("...",) in the UI and in the hand-off report."""
        board, assignments = _alt_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        amp = [f for f in rep.findings if f.check.startswith("trace ampacity")]
        self.assertTrue(amp)
        for f in amp:
            self.assertIsInstance(f.message, str)

    def test_net_ids_are_authoritative_not_file_order(self):
        """Real exports number nets from 0 OR from 1. Assuming file order
        shifts every net by one on the 1-based files: the diagnosis then runs
        on the wrong currents and names the wrong nets, while looking entirely
        plausible. The ID field settles it."""
        one_based = ("|RECORD=Net|ID=1|NAME=D+|\n"
                     "|RECORD=Net|ID=2|NAME=GND|\n"
                     "|RECORD=Track|LAYER=TOP|NET=2|X1=0mm|Y1=0mm|"
                     "X2=10mm|Y2=0mm|WIDTH=0.5mm|\n")
        b = parse_altium_ascii(one_based)
        self.assertEqual(b.net_name(b.segments[0].net), "GND")

        zero_based = ("|RECORD=Net|ID=0|NAME=D+|\n"
                      "|RECORD=Net|ID=1|NAME=GND|\n"
                      "|RECORD=Track|LAYER=TOP|NET=1|X1=0mm|Y1=0mm|"
                      "X2=10mm|Y2=0mm|WIDTH=0.5mm|\n")
        b2 = parse_altium_ascii(zero_based)
        self.assertEqual(b2.net_name(b2.segments[0].net), "GND")

    def _region_board(self, extra_keys):
        return ("|RECORD=Net|ID=0|NAME=SIG|\n"
                "|RECORD=Net|ID=1|NAME=GND|\n"
                f"|RECORD=Region|LAYER=TOP|NET=0|{extra_keys}KIND=0|\n"
                "|RECORD=Track|LAYER=TOP|NET=0|X1=0mm|Y1=0mm|X2=5mm|Y2=0mm|"
                "WIDTH=0.3mm|\n")

    def test_teardrops_and_cutouts_do_not_fake_a_pour(self):
        """A phantom pour SUPPRESSES a real copper-open finding — a false
        all-clear, the one failure direction that matters. Teardrops, keepouts
        and board cutouts are not pours whatever net they claim."""
        for keys in ("TEARDROP=TRUE|", "KEEPOUT=TRUE|", "ISBOARDCUTOUT=TRUE|"):
            b = parse_altium_ascii(self._region_board(keys))
            self.assertEqual(b.zone_nets, set(), keys)

    def test_a_real_netted_region_does_count_as_a_pour(self):
        """The other direction: excluding regions wholesale was a workaround
        for the net-indexing bug, and it cost real pours. A plain region
        attached to a net is copper on that net."""
        b = parse_altium_ascii(self._region_board(""))
        self.assertEqual(b.zone_nets, {b.net_id("SIG")})

    def test_pad_landing_mid_trace_is_connected(self):
        """Routing runs a track straight across a pad and carries on. The pad
        touches real copper but sits nowhere near a segment endpoint."""
        txt = ("|RECORD=Net|ID=0|NAME=SIG|\n"
               "|RECORD=Pad|NAME=A|LAYER=TOP|NET=0|X=0mm|Y=0mm|XSIZE=1mm|"
               "YSIZE=1mm|HOLESIZE=0mm|\n"
               "|RECORD=Pad|NAME=B|LAYER=TOP|NET=0|X=5mm|Y=0mm|XSIZE=1mm|"
               "YSIZE=1mm|HOLESIZE=0mm|\n"
               "|RECORD=Track|LAYER=TOP|NET=0|X1=-2mm|Y1=0mm|X2=9mm|Y2=0mm|"
               "WIDTH=0.3mm|\n")
        b = parse_altium_ascii(txt)
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("copper open")])

    def test_import_assumptions_are_recorded_not_hidden(self):
        b = parse_board(demo_altium_pcb(), "d.PcbDoc")
        blob = " ".join(b.notes).lower()
        self.assertIn("net references resolved", blob)
        self.assertIn("mil", blob)


class TestArcsAreCopper(unittest.TestCase):
    """An arc dropped on the floor is a hole in the connectivity graph, and the
    open-copper check would then call a perfectly good net dead on arrival."""

    def test_kicad_arc_joins_the_net(self):
        txt = ('(kicad_pcb (version 20240108)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "VBAT")\n'
               ' (footprint "a" (layer "F.Cu") (at 0 0)\n'
               '   (property "Reference" "J1")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "VBAT")))\n'
               ' (footprint "b" (layer "F.Cu") (at 10 10)\n'
               '   (property "Reference" "J2")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "VBAT")))\n'
               ' (arc (start 0 0) (mid 2.929 7.071) (end 10 10) (width 0.5)'
               ' (layer "F.Cu") (net 1))\n)')
        b = parse_kicad_pcb(txt)
        self.assertGreater(sum(1 for s in b.segments if s.from_arc), 2)
        rep = diagnose(b, auto_assign_net_currents(b, ledger=None))
        self.assertFalse([f for f in rep.findings
                          if f.check.startswith("copper open")])

    def test_altium_arc_is_chorded_at_the_right_length(self):
        txt = ("|RECORD=Net|ID=0|NAME=VBAT|\n"
               "|RECORD=Arc|LAYER=TOP|NET=0|X=0mm|Y=0mm|RADIUS=10mm|"
               "STARTANGLE=0|ENDANGLE=90|WIDTH=0.5mm|\n")
        b = parse_altium_ascii(txt)
        total = sum(s.length_mm for s in b.segments)
        true_len = 2 * math.pi * 10 / 4
        self.assertTrue(all(s.from_arc for s in b.segments))
        # chords cut the corner: short, but by well under a percent
        self.assertLess(total, true_len)
        self.assertGreater(total, true_len * 0.998)


class TestAltiumAutoFix(unittest.TestCase):
    def test_patch_writes_back_in_the_files_own_units(self):
        """Mending a mil-based board must not quietly convert it to metric —
        a width silently reinterpreted as mm is a 25x error."""
        board, assignments = _alt_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        self.assertTrue(applied)
        self.assertNotIn("WIDTH=6.55|", patched)      # not raw mm
        widths = re.findall(r"WIDTH=([^|]+)\|", patched)
        self.assertTrue(widths)
        self.assertTrue(all(w.endswith("mil") for w in widths), widths)
        # the widened fan traces are ~6.55 mm, i.e. ~258 mil, not "6.55"
        self.assertTrue(any(float(w[:-3]) > 200 for w in widths), widths)

    def test_patched_altium_board_reparses_wider_and_clears(self):
        board, assignments = _alt_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, _ = apply_fixes(board, rep.fixes)
        board2 = parse_board(patched, "demo.PcbDoc")
        fan = board2.net_id("FAN_PWR")
        old_min = min(s.width_mm
                      for s in board.segments_of(board.net_id("FAN_PWR")))
        self.assertGreater(min(s.width_mm for s in board2.segments_of(fan)),
                           old_min)
        # only widths moved
        self.assertEqual(board.nets, board2.nets)
        self.assertEqual(len(board.segments), len(board2.segments))
        rep2 = diagnose(board2, assignments)
        self.assertFalse([f for f in rep2.findings
                          if f.check.startswith("trace ampacity — FAN_PWR")
                          and f.severity == Severity.FAIL])

    def test_one_arc_one_width_token_patched_once(self):
        """Many segments share an arc's single WIDTH token. Two edits at
        overlapping offsets would splice the file twice and corrupt it."""
        txt = ("|RECORD=Net|ID=0|NAME=VBAT|\n"
               "|RECORD=Arc|LAYER=TOP|NET=0|X=0mm|Y=0mm|RADIUS=10mm|"
               "STARTANGLE=0|ENDANGLE=90|WIDTH=0.2mm|\n")
        b = parse_altium_ascii(txt)
        asg = auto_assign_net_currents(b, ledger=None)
        declare_net_current(asg, b.net_id("VBAT"), 10.0)
        rep = diagnose(b, asg)
        patched, applied = apply_fixes(b, rep.fixes)
        self.assertGreater(len(applied), 1)           # many segments fixed
        self.assertEqual(patched.count("WIDTH="), 1)  # one token, written once
        b2 = parse_altium_ascii(patched)
        self.assertGreater(b2.segments[0].width_mm, 0.2)

    def test_report_names_the_source_eda_and_its_assumptions(self):
        board, assignments = _alt_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        md = fix_report_md(board, rep, applied, assignments)
        self.assertIn("Altium", md)
        self.assertIn("What the import had to assume", md)
        self.assertIn("FAN_PWR", md)

    def test_viewer_renders_an_altium_board(self):
        board, assignments = _alt_setup(fan_a=8.0)
        svg = board_svg(board, report=diagnose(board, assignments))
        self.assertTrue(svg.startswith("<svg"))
        self.assertIn("#ff3333", svg)


if __name__ == "__main__":
    unittest.main()
