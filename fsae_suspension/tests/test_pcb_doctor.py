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
from pathlib import Path

from suspension.pcb_doctor import (
    parse_kicad_pcb, parse_board, sniff_format, demo_kicad_pcb,
    auto_assign_net_currents, diagnose, apply_fixes, fix_report_md,
    prescribe_trace, required_width_mm, vias_needed, find_diff_pairs,
    board_svg, clearance_required_mm, declare_net_current,
    trace_ampacity_a, NetAssignment, NODE_WELD_MM, PcbZone,
    PcbSegment, analyze_net, apply_demo_declarations, DEMO_NET_DECLARATIONS)
from suspension.pcb_altium_binary import (
    layer_from_code, is_available as _bin_available)
from suspension.pcb_altium import (
    parse_altium_ascii, demo_altium_pcb, altium_layer_to_kicad,
    is_altium_binary, OLE_MAGIC)
from suspension.interfaces import Severity


def _demo_setup(fan_a=8.0):
    """The demo exactly as the UI loads it — same declarations, same path —
    so what these tests prove is what a member sees after one click."""
    board = parse_kicad_pcb(demo_kicad_pcb())
    assignments = auto_assign_net_currents(board, ledger=None)
    apply_demo_declarations(board, assignments,
                            overrides={"FAN_PWR": {"current_a": fan_a}})
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
        # inner layers rated as outer (IPC-2152 found buried traces no hotter), not IPC-2221's half
        self.assertAlmostEqual(required_width_mm(5.0, 20.0, 1.0, external=False), w1)

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
        """The demo's FAN_PWR needs 6.55 mm. On F.Cu that would run over GND, so those segments are refused and
        turned manual with the reason; on B.Cu there is room and it is widened. The fixed file has no new clash."""
        from suspension.pcb_doctor import widening_conflict
        board, assignments = _demo_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        self.assertTrue(applied)
        refused = [fx for fx in rep.fixes if not fx.auto and "would leave" in fx.note]
        self.assertTrue(refused)
        self.assertTrue(all("'GND'" in fx.note and "reroute" in fx.note for fx in refused))
        board2 = parse_kicad_pcb(patched)
        for fx in applied:
            self.assertGreater(board2.segments[fx.seg_index].width_mm, fx.old_width_mm)
            self.assertIsNone(widening_conflict(board2, fx.seg_index, board2.segments[fx.seg_index].width_mm))
        for fx in refused:                                   # left exactly as they were
            self.assertEqual(board2.segments[fx.seg_index].width_mm, fx.old_width_mm)
        # only widths changed: same net list, same segment count
        self.assertEqual(board.nets, board2.nets)
        self.assertEqual(len(board.segments), len(board2.segments))
        self.assertFalse([f for f in diagnose(board2, assignments).fixes if f.auto])   # nothing left to auto-fix

    def test_a_widening_with_room_clears_the_finding(self):
        b = parse_kicad_pcb('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "PWR")'
                            ' (segment (start 0 0) (end 30 0) (width 0.2) (layer "F.Cu") (net 1)))')
        asg = auto_assign_net_currents(b, ledger=None)
        asg[b.net_id("PWR")] = {**asg.get(b.net_id("PWR"), {}), "current_a": 5.0, "voltage_v": 12.0,
                                "declared": True, "assumed": False, "source": "test"}
        rep = diagnose(b, asg)
        patched, applied = apply_fixes(b, rep.fixes)
        self.assertTrue(applied)
        rep2 = diagnose(parse_kicad_pcb(patched), asg)
        self.assertFalse([f for f in rep2.findings if f.check.startswith("trace ampacity") and f.severity == Severity.FAIL])

    def test_two_nets_widened_side_by_side_cannot_close_the_gap(self):
        from suspension.pcb_doctor import FIX_MIN_CLEARANCE_MM, TraceFix
        b = parse_kicad_pcb('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "A") (net 2 "B")'
                            ' (segment (start 0 0) (end 30 0) (width 0.2) (layer "F.Cu") (net 1))'
                            ' (segment (start 0 1.1) (end 30 1.1) (width 0.2) (layer "F.Cu") (net 2)))')
        fixes = [TraceFix(net="A", nid=1, seg_index=0, old_width_mm=0.2, new_width_mm=1.0, layer="F.Cu"),
                 TraceFix(net="B", nid=2, seg_index=1, old_width_mm=0.2, new_width_mm=1.0, layer="F.Cu")]
        patched, applied = apply_fixes(b, fixes)                 # each alone is clear (0.5 mm gap), both are not
        self.assertEqual([fx.net for fx in applied], ["A"])
        self.assertFalse(fixes[1].auto)
        self.assertIn("once the other fixes are in", fixes[1].note)
        b2 = parse_kicad_pcb(patched)
        gap = 1.1 - b2.segments[0].width_mm / 2 - b2.segments[1].width_mm / 2
        self.assertGreaterEqual(gap, FIX_MIN_CLEARANCE_MM - 1e-9)
        self.assertEqual(b.segments[0].width_mm, 0.2)            # the diagnosed board object is untouched

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
        self.assertIn("a fix will widen", svg)   # failing copper glows


# =========================================================================== #
#  Altium: a second front-end, not a second Doctor
# =========================================================================== #
def _alt_setup(fan_a=8.0):
    board = parse_board(demo_altium_pcb(), "demo_ecu_board.PcbDoc")
    assignments = auto_assign_net_currents(board, ledger=None)
    apply_demo_declarations(board, assignments,
                            overrides={"FAN_PWR": {"current_a": fan_a}})
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
        txt = demo_kicad_pcb()
        b = parse_kicad_pcb(txt)
        self.assertEqual(b.net_name(b.net_id("FAN_PWR")), "FAN_PWR")
        # every segment in the file is read — counted off the text, so the
        # assertion follows the demo instead of pinning its current size
        self.assertEqual(len(b.segments), txt.count("(segment "))


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
        # the same routed copper the KiCad demo carries, not one track more
        self.assertEqual(len(board.segments),
                         len(parse_kicad_pcb(demo_kicad_pcb()).segments))
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
        from suspension.pcb_doctor import widening_conflict
        board, assignments = _alt_setup(fan_a=8.0)
        rep = diagnose(board, assignments)
        patched, applied = apply_fixes(board, rep.fixes)
        board2 = parse_board(patched, "demo.PcbDoc")
        self.assertTrue(applied)
        for fx in applied:                                      # wider, in the file's units, and clear of every net
            self.assertGreater(board2.segments[fx.seg_index].width_mm, fx.old_width_mm)
            self.assertIsNone(widening_conflict(board2, fx.seg_index, board2.segments[fx.seg_index].width_mm))
        # only widths moved
        self.assertEqual(board.nets, board2.nets)
        self.assertEqual(len(board.segments), len(board2.segments))
        self.assertFalse([f for f in diagnose(board2, assignments).fixes if f.auto])

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
        self.assertIn("a fix will widen", svg)
        self.assertIn("Top Layer", svg)          # the Altium user's own names


# =========================================================================== #
#  The demo, as a member meets it: one click, no ledger, a board that looks real
# =========================================================================== #
def _pad_box(p):
    """Axis-aligned copper box of a pad (the demo has no rotated pads)."""
    w, h = p.size
    return (p.at[0] - w / 2, p.at[1] - h / 2, p.at[0] + w / 2, p.at[1] + h / 2)


def _box_gap(a, b):
    dx = max(b[0] - a[2], a[0] - b[2], 0.0)
    dy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return math.hypot(dx, dy)


def _seg_box_gap(sg, box):
    """Edge-to-edge gap from a trace to a pad box; 0 when they touch."""
    x0, y0, x1, y1 = box
    (ax, ay), (bx, by) = sg.start, sg.end
    best = 1e9
    n = max(1, int(math.hypot(bx - ax, by - ay) / 0.05))
    for k in range(n + 1):
        px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
        dx = max(x0 - px, 0.0, px - x1)
        dy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(dx, dy))
    return max(best - sg.width_mm / 2.0, 0.0)


def _pad_side(fp, p):
    return None if p.through else (p.layer or fp.layer or "F.Cu")


class TestDemoWorksOutOfTheBox(unittest.TestCase):
    """The reported bug: a fresh session clicked Demo and got "0 width fix(es)
    ready" under a board advertised as carrying three planted failures. The
    ledger of a fresh session declares no peak current, so the fan feed was —
    correctly — MISSING, and the demo's headline failure never appeared."""

    def _fresh_session(self, text, name):
        from suspension import interfaces as I
        led = I.IntegrationLedger.from_dict(I.blank_ledger().as_dict())
        board = parse_board(text, name)
        asg = auto_assign_net_currents(board, ledger=led)
        apply_demo_declarations(board, asg)
        return board, diagnose(board, asg)

    def test_blank_ledger_demo_has_fixes_to_apply(self):
        for text, name in ((demo_kicad_pcb(), "d.kicad_pcb"),
                           (demo_altium_pcb(), "d.PcbDoc")):
            with self.subTest(name):
                board, rep = self._fresh_session(text, name)
                self.assertTrue([fx for fx in rep.fixes if fx.auto])
                checks = {f.check for f in rep.findings}
                for c in ("trace ampacity — FAN_PWR", "via bottleneck — FAN_PWR",
                          "HV clearance — HV_INV_SENSE", "diff pair skew — CAN",
                          "component — C1"):
                    self.assertIn(c, checks)

    def test_declarations_are_labelled_and_editable(self):
        board = parse_kicad_pcb(demo_kicad_pcb())
        asg = apply_demo_declarations(
            board, auto_assign_net_currents(board, ledger=None))
        fan = asg[board.net_id("FAN_PWR")]
        self.assertTrue(fan.declared)
        self.assertIn("demo", fan["source"])
        self.assertEqual(fan["current_a"],
                         DEMO_NET_DECLARATIONS["FAN_PWR"]["current_a"])
        # a voltage alone must not promote a guessed current to a declaration
        self.assertFalse(asg[board.net_id("HV_INV_SENSE")].declared)
        self.assertEqual(asg[board.net_id("HV_INV_SENSE")]["voltage_v"], 400.0)

    def test_a_real_board_is_untouched_by_the_demo_path(self):
        txt = ('(kicad_pcb (version 20240108)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "VBAT")\n'
               ' (segment (start 0 0) (end 5 0) (width 0.3) (layer "F.Cu") (net 1)))\n')
        b = parse_kicad_pcb(txt)
        asg = auto_assign_net_currents(b, ledger=None)
        before = {k: dict(v) for k, v in asg.items()}
        apply_demo_declarations(b, asg)
        self.assertEqual(before, {k: dict(v) for k, v in asg.items()})


class TestDemoBoardIsBuildable(unittest.TestCase):
    """The viewer draws every pad, so the demo's copper has to be copper a fab
    would accept: its planted faults are physics a DRC passes, never shorts.
    The previous demo had a CAN connector whose two pins overlapped, a 400 V
    sense trace running through a CAN pad, and the 5 V rail starting on the
    fan net's pad — invisible while only footprint origins were drawn."""

    BOARDS = ((demo_kicad_pcb, "d.kicad_pcb"), (demo_altium_pcb, "d.PcbDoc"))

    def test_no_pads_of_different_nets_touch(self):
        for gen, name in self.BOARDS:
            b = parse_board(gen(), name)
            pads = [(fp, p) for fp in b.footprints for p in fp.pads]
            for i, (fa, pa) in enumerate(pads):
                for fb, pb in pads[i + 1:]:
                    if pa.net == pb.net:
                        continue
                    sa, sb = _pad_side(fa, pa), _pad_side(fb, pb)
                    if sa and sb and sa != sb:
                        continue
                    with self.subTest(name, a=f"{fa.ref}.{pa.number}",
                                      b=f"{fb.ref}.{pb.number}"):
                        self.assertGreater(
                            _box_gap(_pad_box(pa), _pad_box(pb)), 0.15)

    def test_no_trace_crosses_another_nets_pad(self):
        for gen, name in self.BOARDS:
            b = parse_board(gen(), name)
            for sg in b.segments:
                for fp in b.footprints:
                    for p in fp.pads:
                        side = _pad_side(fp, p)
                        if p.net == sg.net or (side and side != sg.layer):
                            continue
                        with self.subTest(name, net=b.net_name(sg.net),
                                          pad=f"{fp.ref}.{p.number}"):
                            self.assertGreater(_seg_box_gap(sg, _pad_box(p)),
                                               0.1)

    def test_every_trace_end_lands_on_its_own_net(self):
        """No trace ends in mid-air: each endpoint sits on a pad or via of its
        net, or on another segment of its net (a T-junction)."""
        for gen, name in self.BOARDS:
            b = parse_board(gen(), name)
            for i, sg in enumerate(b.segments):
                for end in (sg.start, sg.end):
                    on_pad = any(
                        p.net == sg.net and _box_gap(
                            _pad_box(p), (end[0], end[1], end[0], end[1])) == 0
                        for fp in b.footprints for p in fp.pads)
                    on_via = any(v.net == sg.net and math.dist(v.at, end) < 1e-6
                                 for v in b.vias)
                    on_seg = any(j != i and o.net == sg.net and
                                 o.layer == sg.layer and
                                 (math.dist(o.start, end) < 1e-6
                                  or math.dist(o.end, end) < 1e-6)
                                 for j, o in enumerate(b.segments))
                    with self.subTest(name, net=b.net_name(sg.net), end=end):
                        self.assertTrue(on_pad or on_via or on_seg)


class TestViewerDrawsPads(unittest.TestCase):
    """The screenshot that prompted this: traces ending in mid-air beside an
    8-px footprint marker, because pads were never drawn at all."""

    def test_every_pad_is_drawn_with_its_net(self):
        b = parse_kicad_pcb(demo_kicad_pcb())
        svg = board_svg(b)
        for fp in b.footprints:
            for p in fp.pads:
                self.assertIn(f"<title>{fp.ref}.{p.number} · {p.net_name}"
                              f"</title>", svg)

    def test_no_glow_and_no_dimming_without_fixes(self):
        """Emphasis only when there is something to emphasise: a report with
        no fixes (and no report at all) must draw the board at full strength."""
        b, asg = _demo_setup()
        rep = diagnose(b, asg)
        self.assertTrue(rep.fixes)                    # the demo has some…
        rep.fixes = []                                # …so take them away
        for svg in (board_svg(b), board_svg(b, report=rep)):
            self.assertNotIn("a fix will widen", svg)
            self.assertNotIn('opacity="0.35"', svg)
        self.assertIn('opacity="0.35"', board_svg(b, report=diagnose(b, asg)))

    def test_fix_glow_is_not_a_layer_colour(self):
        from suspension import pcb_doctor as pdr
        self.assertNotIn(pdr._FIX_GLOW.lower(),
                         {c.lower() for c in pdr._LAYER_COLORS.values()})

    def test_names_are_escaped(self):
        txt = ('(kicad_pcb (version 20240108)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "A<B&C")\n'
               ' (footprint "x" (layer "F.Cu") (at 0 0)\n'
               '   (property "Reference" "U<1>")\n'
               '   (pad "1" smd rect (at 0 0) (size 1 1) (net 1 "A<B&C")))\n'
               ' (segment (start 0 0) (end 5 0) (width 0.3) (layer "F.Cu") (net 1)))\n')
        svg = board_svg(parse_kicad_pcb(txt))
        self.assertNotIn("U<1>", svg)
        self.assertIn("U&lt;1&gt;", svg)
        self.assertIn("A&lt;B&amp;C", svg)

    def test_rotated_pad_orientation_is_read_and_drawn(self):
        """KiCad writes a pad's absolute orientation as the third number of its
        (at …); a 90° part whose pads were drawn unrotated would lie across
        its own traces."""
        txt = ('(kicad_pcb (version 20240108)\n'
               ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
               ' (net 0 "") (net 1 "SIG")\n'
               ' (footprint "x" (layer "F.Cu") (at 10 10 90)\n'
               '   (property "Reference" "R9")\n'
               '   (pad "1" smd roundrect (at -1 0 90) (size 1 2) (net 1 "SIG")))\n'
               ' (segment (start 10 11) (end 20 11) (width 0.3) (layer "F.Cu") (net 1)))\n')
        b = parse_kicad_pcb(txt)
        pad = b.footprints[0].pads[0]
        self.assertEqual(pad.angle_deg, 90.0)
        self.assertEqual(pad.shape, "roundrect")
        self.assertIn('transform="rotate(-90', board_svg(b))


if __name__ == "__main__":
    unittest.main()


class TestRound3NetResistance(unittest.TestCase):
    """The round-3 net-resistance changes (docs/ELECTRICAL_BENCHMARK.md), each on a board small enough to check by
    hand. Copper: 1 oz, at 60 °C (the nodal solve's temperature)."""

    @staticmethod
    def _r(width_mm, length_mm):
        from suspension.electronics import Trace
        return Trace(name="t", net="t", owner_subsystem="e", width_mm=width_mm, copper_oz=1.0,
                     length_mm=length_mm).resistance_ohm(temp_c=60.0)

    @staticmethod
    def _board(body, pads=('(pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG"))',
                           '(pad "1" smd rect (at 20 0) (size 1 1) (net 1 "SIG"))')):
        fps = "".join(f' (footprint "f{i}" (layer "F.Cu") (at 0 0) (property "Reference" "J{i}") {p})\n'
                      for i, p in enumerate(pads, 1))
        return parse_kicad_pcb('(kicad_pcb (version 20241229)\n (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
                               ' (net 0 "") (net 1 "SIG")\n' + fps + body + ")")

    def _worst(self, b):
        return analyze_net(b, b.net_id("SIG"))["worst_r_ohm"]

    def test_track_on_a_pad_is_not_counted(self):
        """A 20 mm track from pad centre to pad centre: the 0.5 mm under each 1 mm pad carries no current."""
        b = self._board(' (segment (start 0 0) (end 20 0) (width 0.25) (layer "F.Cu") (net 1))\n')
        self.assertAlmostEqual(self._worst(b) / self._r(0.25, 19.0), 1.0, delta=0.03)

    def test_long_chain_is_the_sum_of_its_tracks(self):
        """200 tracks in series: the solve must give their sum (numpy's pinv had lost a quarter on such chains)."""
        segs = "".join(f' (segment (start {0.1 * k} 0) (end {0.1 * (k + 1)} 0) (width 0.25) (layer "F.Cu") (net 1))\n'
                       for k in range(200))
        b = self._board(segs)
        self.assertAlmostEqual(self._worst(b) / self._r(0.25, 19.0), 1.0, delta=0.03)

    def test_short_arc_chords_are_not_welded_shut(self):
        """Chords 40 um long are shorter than the 50 um endpoint weld; they still carry their resistance."""
        segs = "".join(f' (segment (start {0.04 * k} 0) (end {0.04 * (k + 1)} 0) (width 0.25) (layer "F.Cu") (net 1))\n'
                       for k in range(500))
        b = self._board(segs)
        self.assertAlmostEqual(self._worst(b) / self._r(0.25, 19.0), 1.0, delta=0.03)

    def test_thin_track_inside_a_wide_one_carries_nothing(self):
        """A 0.2 mm track left on top of a 1 mm track adds no copper; it must not halve the resistance."""
        b = self._board(' (segment (start 0 0) (end 20 0) (width 1) (layer "F.Cu") (net 1))\n'
                        ' (segment (start 2 0) (end 18 0) (width 0.2) (layer "F.Cu") (net 1))\n'
                        ' (segment (start 2 0.1) (end 2 0) (width 1) (layer "F.Cu") (net 1))\n'
                        ' (segment (start 18 0.1) (end 18 0) (width 1) (layer "F.Cu") (net 1))\n')
        self.assertAlmostEqual(self._worst(b) / self._r(1.0, 19.0), 1.0, delta=0.05)

    def test_same_numbered_pads_are_separate_copper(self):
        """A switch's two pin-1 legs are joined inside the part, not by copper: no shortcut, still connected."""
        pads = ('(pad "1" smd rect (at 0 0) (size 1 1) (net 1 "SIG"))',
                '(pad "1" smd rect (at 10 0) (size 1 1) (net 1 "SIG")) (pad "1" smd rect (at 10 5) (size 1 1)'
                ' (net 1 "SIG")) (pad "2" smd rect (at 10 10) (size 1 1) (net 0 ""))',
                '(pad "1" smd rect (at 20 5) (size 1 1) (net 1 "SIG"))')
        b = self._board(' (segment (start 0 0) (end 10 0) (width 0.25) (layer "F.Cu") (net 1))\n'
                        ' (segment (start 10 0) (end 10 5) (width 0.25) (layer "F.Cu") (net 1))\n'
                        ' (segment (start 10 5) (end 20 5) (width 0.25) (layer "F.Cu") (net 1))\n', pads)
        r = analyze_net(b, b.net_id("SIG"))
        self.assertFalse(r["open_groups"])
        self.assertAlmostEqual(r["worst_r_ohm"] / self._r(0.25, 22.0), 1.0, delta=0.03)   # 24 mm less 0.5 mm under each pad end

    def test_pad_copper_offset_from_its_drill(self):
        """(drill 1 (offset 2 0)) moves the copper 2 mm from the hole, turned with the pad."""
        b = self._board("", ('(pad "1" thru_hole rect (at 0 0 90) (size 3 1) (drill 1 (offset 2 0))'
                             ' (layers "*.Cu") (net 1 "SIG"))',))
        p = b.footprints[0].pads[0]
        self.assertAlmostEqual(math.hypot(*p.copper_offset), 2.0, places=6)
        self.assertAlmostEqual(abs(p.copper_offset[1]), 2.0, places=6)

    def test_via_without_a_drill_takes_the_board_default(self):
        b = parse_kicad_pcb('(kicad_pcb (version 20241229) (setup (via_drill 0.45))\n'
                            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal)) (net 0 "") (net 1 "SIG")\n'
                            ' (via (at 0 0) (size 0.8) (layers "F.Cu" "B.Cu") (net 1)))')
        self.assertAlmostEqual(b.vias[0].drill_mm, 0.45)


class TestBuiltBoardTolerance(unittest.TestCase):
    """The board from the fab is not the file: thin copper in the brown-out check, a measurement plan for bring-up."""

    def test_measurement_plan_band_brackets_the_estimate(self):
        b = parse_kicad_pcb(demo_kicad_pcb())
        from suspension.pcb_doctor import bringup_measurements
        rows = bringup_measurements(b)
        self.assertTrue(rows)
        for r in rows:
            lo, hi = r["band_mohm"]
            self.assertLess(lo, r["expected_mohm"])
            self.assertGreater(hi, r["expected_mohm"])

    def test_thinner_copper_assumption_raises_the_drop(self):
        b = parse_kicad_pcb(demo_kicad_pcb())
        asg = auto_assign_net_currents(b, ledger=None)
        apply_demo_declarations(b, asg)

        def worst(rep):
            import re as _re
            vals = [float(m) for f in rep.findings if f.check.startswith("IR drop")
                    for m in _re.findall(r"drops (\\d+\\.\\d+) V", f.message)]
            return max(vals) if vals else 0.0
        nominal = worst(diagnose(b, asg, copper_finish=1.0))
        thin = worst(diagnose(b, asg, copper_finish=0.8))
        if nominal:
            self.assertAlmostEqual(thin / nominal, 1.0 / 0.8, places=2)


class TestPourRound3Reading(unittest.TestCase):
    """Pour estimate round 3 (benchmark plan addendum 8): what KinematiK reads beyond a pad's anchor and a bare fill."""

    def test_custom_pad_primitives_turned_with_the_pad(self):
        b = parse_kicad_pcb(
            '(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "K")'
            ' (footprint "led" (layer "F.Cu") (at 10 10 -90) (fp_text reference "D1" (at 0 0))'
            '  (pad "1" smd custom (at -0.4 0.5 270) (size 0.1 0.1) (layers "F.Cu") (net 1 "K")'
            '   (options (clearance outline) (anchor circle))'
            '   (primitives (gr_poly (pts (xy -0.15 0.25) (xy -0.15 -0.15) (xy 0 -0.15) (xy 0.2 0.05) (xy 0.2 0.25))'
            '    (width 0.0001) (fill yes))))))')
        p = b.footprints[0].pads[0]
        self.assertEqual(p.primitives[0][0], "poly")
        xs = sorted(round(x, 3) for x, _y in p.primitives[0][1])
        ys = sorted(round(y, 3) for _x, y in p.primitives[0][1])
        # KiCad 7's own polygon for this pad spans x 9.85..10.25 (pad at 10.5, 9.6 rotated) — checked in pcbnew
        self.assertAlmostEqual(xs[-1] - xs[0], 0.4, places=3)
        self.assertAlmostEqual(ys[-1] - ys[0], 0.35, places=3)

    def test_kicad5_fill_is_stroked_and_lowers_the_pour_resistance(self):
        from suspension import pcb_pour as P
        zone = ('(zone (net 1) (net_name "P") (layer "F.Cu") (min_thickness 0.254)'
                ' (polygon (pts (xy 0 -1) (xy 10 -1) (xy 10 1) (xy 0 1)))'
                ' (filled_polygon (pts (xy 0.127 -0.873) (xy 9.873 -0.873) (xy 9.873 0.873) (xy 0.127 0.873))))')
        fps = ''.join(f'(module m{i} (layer F.Cu) (at {x} 0) (fp_text reference J{i} (at 0 0))'
                      f' (pad 1 smd rect (at 0 0) (size 0.5 1.5) (layers F.Cu) (net 1 P)))' for i, x in ((1, 0.5), (2, 9.5)))
        txt = lambda v: f'(kicad_pcb (version {v}) (layers (0 F.Cu signal)) (net 0 "") (net 1 P) {fps} {zone})'
        old, new = parse_kicad_pcb(txt(20171130)), parse_kicad_pcb(txt(20221018))
        self.assertAlmostEqual(old.zones[0].stroke_mm, 0.254)
        self.assertEqual(new.zones[0].stroke_mm, 0.0)
        r_old = P.pour_resistance(old, old.net_id("P"))["r_ohm"]
        r_new = P.pour_resistance(new, new.net_id("P"))["r_ohm"]
        # the stroke adds half of 0.254 mm on each side of a 1.746 mm wide strip: about 2.0 / 1.746 wider
        self.assertAlmostEqual(r_new / r_old, 2.0 / 1.746, delta=0.04)


class TestPourGridCheck(unittest.TestCase):
    """Pour estimate round 4 (plan addendum 10): the estimate checks its own grid and says whether it converged."""

    def test_a_plain_strip_converges_and_says_so(self):
        from suspension import pcb_pour as P
        fps = ''.join(f'(footprint "m{i}" (layer "F.Cu") (at {x} 0) (property "Reference" "J{i}")'
                      f' (pad "1" smd rect (at 0 0) (size 0.5 2) (layers "F.Cu") (net 1 "P")))' for i, x in ((1, 0.25), (2, 9.75)))
        b = parse_kicad_pcb('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "P") ' + fps +
                            ' (zone (net 1) (net_name "P") (layer "F.Cu") (fill yes (thermal_bridge_width 0.5))'
                            ' (filled_polygon (layer "F.Cu") (pts (xy 0 -1) (xy 10 -1) (xy 10 1) (xy 0 1)))))')
        r = P.pour_resistance(b, b.net_id("P"))
        self.assertTrue(r["converged"])
        self.assertLessEqual(r["grid_change_pct"], P.GRID_TOL_PCT)
        self.assertLess(r["h_mm"], P.pour_grid_mm(b, b.net_id("P")))          # the reported value is the finer solve
        # 9 mm of 2 mm wide 1 oz copper between the pads at 60 C: rho L / (w t)
        from suspension.electronics import Trace
        want = Trace(name="t", net="t", owner_subsystem="e", width_mm=2.0, copper_oz=1.0,
                     length_mm=9.0).resistance_ohm(temp_c=60.0)
        self.assertAlmostEqual(r["r_ohm"] / want, 1.0, delta=0.02)

    def test_no_room_for_a_finer_grid_is_reported_as_unverified(self):
        from suspension import pcb_pour as P
        fps = ''.join(f'(footprint "m{i}" (layer "F.Cu") (at {x} 0) (property "Reference" "J{i}")'
                      f' (pad "1" smd rect (at 0 0) (size 0.5 2) (layers "F.Cu") (net 1 "P")))' for i, x in ((1, 0.25), (2, 9.75)))
        b = parse_kicad_pcb('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "P") ' + fps +
                            ' (zone (net 1) (net_name "P") (layer "F.Cu") (fill yes (thermal_bridge_width 0.5))'
                            ' (filled_polygon (layer "F.Cu") (pts (xy 0 -1) (xy 10 -1) (xy 10 1) (xy 0 1)))))')
        h = P.pour_grid_mm(b, b.net_id("P"))
        old = P.MAX_CELLS_PER_LAYER
        P.MAX_CELLS_PER_LAYER = int(20.0 / (h * h) * 1.5)                      # room for h, not for h / 2
        try:
            r = P.pour_resistance(b, b.net_id("P"))
        finally:
            P.MAX_CELLS_PER_LAYER = old
        self.assertFalse(r["converged"])
        self.assertIsNone(r["grid_change_pct"])
        self.assertAlmostEqual(r["h_mm"], h)


class TestEmptyPours(unittest.TestCase):
    """A pour with no stored fill (plan addendum 11)."""
    HEAD = ('(kicad_pcb (version 20241229) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "A") (net 2 "B")'
            ' (footprint "f" (layer "F.Cu") (at 0 0) (property "Reference" "J1")'
            '  (pad "1" smd rect (at 1 1) (size 1 1) (layers "F.Cu") (net 1 "A"))'
            '  (pad "2" smd rect (at 8 1) (size 1 1) (layers "F.Cu") (net 1 "A")))')
    EMPTY = ' (zone (net 1) (net_name "A") (layer "F.Cu") (polygon (pts (xy 0 0) (xy 10 0) (xy 10 2) (xy 0 2))))'
    FILLED_B = (' (zone (net 2) (net_name "B") (layer "F.Cu") (polygon (pts (xy 0 5) (xy 10 5) (xy 10 7) (xy 0 7)))'
                ' (filled_polygon (layer "F.Cu") (pts (xy 0 5) (xy 10 5) (xy 10 7) (xy 0 7))))')
    KEEPOUT = (' (zone (net 0) (net_name "") (layer "F.Cu") (keepout (tracks not_allowed))'
               ' (polygon (pts (xy 20 0) (xy 30 0) (xy 30 2) (xy 20 2))))')

    def test_empty_pour_in_a_filled_board_is_not_copper(self):
        b = parse_kicad_pcb(self.HEAD + self.EMPTY + self.FILLED_B + self.KEEPOUT + ")")
        self.assertEqual([z.net for z in b.zones if z.net], [2])
        self.assertNotIn(1, b.zone_nets)
        self.assertEqual(sum("no filled copper" in n for n in b.notes), 1)
        self.assertIn("nets: A)", " ".join(b.notes))

    def test_board_saved_without_any_fill_keeps_outlines_and_says_so(self):
        b = parse_kicad_pcb(self.HEAD + self.EMPTY + self.KEEPOUT + ")")
        self.assertEqual([z.net for z in b.zones if z.net], [1])
        self.assertIn(1, b.zone_nets)
        self.assertTrue(any("saved without being filled" in n for n in b.notes))


class TestRefinedNetResistance(unittest.TestCase):
    """Plan addendum 12: the sheet solve between the 1-D model's worst pair, and what the brown-out check does with it."""

    def _board(self, body):
        return TestRound3NetResistance._board(body)

    def test_a_plain_track_is_refined_and_agrees_with_rho_l_over_a(self):
        from suspension.pcb_doctor import refined_net_resistance
        b = self._board(' (segment (start 0 0) (end 20 0) (width 0.25) (layer "F.Cu") (net 1))\n')
        r = refined_net_resistance(b, b.net_id("SIG"))
        self.assertEqual(r["source"], "sheet solve")
        self.assertAlmostEqual(r["r_ohm"] / TestRound3NetResistance._r(0.25, 19.0), 1.0, delta=0.02)
        self.assertAlmostEqual(r["r_ohm"] / r["r_1d_ohm"], 1.0, delta=0.04)

    def test_falls_back_to_the_1d_value_when_the_net_is_too_large(self):
        from suspension import pcb_doctor as D
        b = self._board(' (segment (start 0 0) (end 20 0) (width 0.25) (layer "F.Cu") (net 1))\n')
        old = D.REFINE_MAX_CELLS
        D.REFINE_MAX_CELLS = 100
        try:
            r = D.refined_net_resistance(b, b.net_id("SIG"))
        finally:
            D.REFINE_MAX_CELLS = old
        self.assertEqual(r["source"], "1-D")
        self.assertEqual(r["r_ohm"], r["r_1d_ohm"])

    def test_brownout_bound_is_never_below_the_1d_bound(self):
        """The check takes the larger of 1-D × 1.10 and sheet × 1.02, so refining can only make it stricter."""
        from suspension import pcb_doctor as D
        self.assertGreaterEqual(D.NET_R_UPPER_FACTOR, 1.10)
        self.assertGreater(D.SHEET_UPPER_FACTOR, 1.0)
        src = Path(D.__file__).read_text(encoding="utf-8")
        self.assertIn('r_up = max(r_up, ref_["r_ohm"] * SHEET_UPPER_FACTOR)', src)


class TestBrownoutThroughAPour(unittest.TestCase):
    """Plan addendum 13: a load fed through a pour is checked through the pour, and an unsolved pour is said out loud."""
    #  Two pads 60 mm apart joined only by a 0.6 mm wide filled strip; a third pad next to the first joined by a track.
    BOARD = ('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "VCC")'
             ' (footprint "f" (layer "F.Cu") (at 0 0) (property "Reference" "J1")'
             '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VCC")))'
             ' (footprint "f" (layer "F.Cu") (at 3 0) (property "Reference" "C1")'
             '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VCC")))'
             ' (footprint "f" (layer "F.Cu") (at 60 0) (property "Reference" "U1")'
             '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VCC")))'
             ' (segment (start 0 0) (end 3 0) (width 1.0) (layer "F.Cu") (net 1))'
             ' (zone (net 1) (net_name "VCC") (layer "F.Cu") (fill yes (thermal_bridge_width 0.5))'
             '  (filled_polygon (layer "F.Cu") (pts (xy -0.5 -0.3) (xy 60.5 -0.3) (xy 60.5 0.3) (xy -0.5 0.3)))))')

    def _findings(self, **patch):
        from suspension import pcb_doctor as D
        b = parse_kicad_pcb(self.BOARD)
        asg = {b.net_id("VCC"): {"current_a": 10.0, "voltage_v": 5.0}}
        old = {k: getattr(D, k) for k in patch}
        for k, v in patch.items():
            setattr(D, k, v)
        try:
            rep = diagnose(b, asg)
        finally:
            for k, v in old.items():
                setattr(D, k, v)
        return [f for f in rep.findings if f.check.startswith("IR drop")]

    def test_the_far_load_fed_by_the_pour_is_what_gets_checked(self):
        from suspension import pcb_doctor as D
        b = parse_kicad_pcb(self.BOARD)
        nid = b.net_id("VCC")
        one_d = D.analyze_net(b, nid)["worst_r_ohm"]                 # the 3 mm track between J1 and C1 only
        pour = D.pour_net_resistance(b, nid)
        self.assertEqual(sorted(pour["pair"]), ["J1.1", "U1.1"])
        self.assertGreater(pour["r_ohm"], 10 * one_d)                # 60 mm of 0.6 mm strip against 3 mm of 1 mm track
        fs = self._findings()
        self.assertEqual([f.severity.name for f in fs], ["FAIL"])
        self.assertIn("U1.1", fs[0].message)
        self.assertIn("(through the pour)", fs[0].message)

    def test_a_pour_too_large_to_solve_is_reported_as_not_checked(self):
        fs = self._findings(REFINE_MAX_CELLS=50)
        warn = [f for f in fs if "NOT checked" in f.message]
        self.assertEqual(len(warn), 1)
        self.assertEqual(warn[0].severity.name, "WARN")
        self.assertIn("covers only the pads its tracks join", warn[0].message)
        self.assertFalse(any("conservative" in f.message for f in fs))

    def test_an_unconverged_pour_estimate_gets_the_wider_margin(self):
        from suspension import pcb_doctor as D
        self.assertGreaterEqual(D.POUR_UNVERIFIED_FACTOR, 1.10)
        self.assertGreater(D.POUR_UNVERIFIED_FACTOR, D.SHEET_UPPER_FACTOR)


class TestVerifiedFix(unittest.TestCase):
    """A patched board must never carry a finding the original did not have, or a worse one."""

    def test_demo_patched_board_is_never_worse(self):
        from suspension.pcb_doctor import _worse_findings, fix_board
        for text, name in ((demo_kicad_pcb(), "d.kicad_pcb"), (demo_altium_pcb(), "d.PcbDoc")):
            with self.subTest(name):
                board = parse_board(text, name)
                asg = apply_demo_declarations(board, auto_assign_net_currents(board, ledger=None))
                rep0 = diagnose(board, asg)
                out = fix_board(board, asg, report=rep0)
                self.assertIsNotNone(out["text"])
                b2 = parse_board(out["text"], name)
                b2.copper_oz = board.copper_oz
                self.assertEqual(_worse_findings(rep0, diagnose(b2, asg)), [])
                self.assertTrue(out["remaining"])                # FAN_PWR on F.Cu has no room: said, not hidden
                self.assertTrue(all(f.severity.value in ("fail", "warning") for f in out["remaining"]))

    def test_a_widening_into_an_hv_gap_is_refused(self):
        """LV default spacing is not enough next to 400 V: IPC-2221 B4 applies to the fix as it does to the check.
        10 A needs 8.2 mm (heating and the 2x fusing margin); with the 400 V net 6 mm away that leaves 1.65 mm,
        under the 2.5 mm B4 wants but well over the 0.2 mm LV default, so only an HV-aware fixer refuses it."""
        from suspension.pcb_doctor import FIX_MIN_CLEARANCE_MM, clearance_required_mm, fix_board
        b = parse_kicad_pcb('(kicad_pcb (version 20221018) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "PWR") (net 2 "HV")'
                            ' (segment (start 0 0) (end 30 0) (width 0.2) (layer "F.Cu") (net 1))'
                            ' (segment (start 0 6.0) (end 30 6.0) (width 0.5) (layer "F.Cu") (net 2)))')
        asg = auto_assign_net_currents(b, ledger=None)
        asg[1] = {**asg.get(1, {}), "current_a": 10.0, "voltage_v": 12.0, "declared": True, "assumed": False, "source": "t"}
        asg[2] = {**asg.get(2, {}), "current_a": 0.1, "voltage_v": 400.0, "declared": True, "assumed": False, "source": "t"}
        need_hv = clearance_required_mm(400.0)
        out = fix_board(b, asg)
        refused = [fx for fx in out["manual"] if fx.net == "PWR"]
        self.assertTrue(refused)
        self.assertIn("'HV' (needs 2.5 mm)", refused[0].note)
        lv_gap = 6.0 - refused[0].new_width_mm / 2 - 0.25
        self.assertTrue(FIX_MIN_CLEARANCE_MM < lv_gap < need_hv)     # an LV-only check would have written it
        b2 = parse_kicad_pcb(out["text"]) if out["text"] else b
        self.assertGreaterEqual(6.0 - b2.segments[0].width_mm / 2 - 0.25, need_hv - 1e-9)

    def test_a_fix_that_makes_another_finding_worse_is_withdrawn(self):
        from unittest import mock

        from suspension import pcb_doctor as D
        board = parse_board(demo_kicad_pcb(), "d.kicad_pcb")
        asg = apply_demo_declarations(board, auto_assign_net_currents(board, ledger=None))
        rep0 = D.diagnose(board, asg)
        real = D.diagnose
        calls = {"n": 0}

        def worse_after_patch(b, a, **kw):
            r = real(b, a, **kw)
            calls["n"] += 1
            r.findings.append(D.Finding(check="invented — FAN_PWR", severity=D.Severity.FAIL,
                                        subsystems=["electrics"], message="'FAN_PWR' got worse"))
            return r
        with mock.patch.object(D, "diagnose", worse_after_patch):
            out = D.fix_board(board, asg, report=rep0)
        self.assertGreater(calls["n"], 0)
        self.assertIsNone(out["text"])                           # nothing safe to hand back
        self.assertTrue(all(not fx.auto for fx in rep0.fixes if fx.net == "FAN_PWR"))
        self.assertTrue(any("withdrawn" in fx.note for fx in rep0.fixes))


class TestFixHonoursBoardRules(unittest.TestCase):
    def test_net_class_clearance_wider_than_the_default_is_kept(self):
        from suspension.pcb_doctor import TraceFix, apply_fixes, widening_conflict
        txt = ('(kicad_pcb (version 20211014) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "A") (net 2 "B")'
               ' (net_class Default "" (clearance 0.5) (add_net A) (add_net B))'
               ' (segment (start 0 0) (end 30 0) (width 0.2) (layer "F.Cu") (net 1))'
               ' (segment (start 0 1.0) (end 30 1.0) (width 0.2) (layer "F.Cu") (net 2)))')
        b = parse_kicad_pcb(txt)
        self.assertEqual(b.clearance_mm, {1: 0.5, 2: 0.5})
        #  0.8 mm leaves 0.5 mm... to 0.1 mm half-width: gap 1.0 - 0.4 - 0.1 = 0.5, allowed; 1.0 mm leaves 0.4, refused
        self.assertIsNone(widening_conflict(b, 0, 0.8))
        clash = widening_conflict(b, 0, 1.0)
        self.assertEqual((clash[0], round(clash[1], 3), clash[2]), ("B", 0.4, 0.5))
        fx = TraceFix(net="A", nid=1, seg_index=0, old_width_mm=0.2, new_width_mm=1.0, layer="F.Cu")
        _, applied = apply_fixes(b, [fx])
        self.assertEqual(applied, [])
        self.assertIn("needs 0.5 mm", fx.note)
