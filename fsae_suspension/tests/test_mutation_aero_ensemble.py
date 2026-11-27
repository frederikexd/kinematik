"""Characterization suite for suspension/aero/ensemble.py (mutation gate); see tests/_golden.py. The default
in-house roster and hand-made rosters: members that agree, disagree beyond the tolerance, fall one short of
min_members, did not converge, raise SolverUnavailable or another error (short and long messages), report side
force, pitch and balance or not, fused by mean and median, run and read back, written (including a failing
writer), the provenance with and without votes and spread, the spread and reduction helpers, and a matrix."""
import math
import os

import pytest

from suspension.aero import cfd
from suspension.aero import ensemble as E
from tests._golden import plain
from tests._golden_lazy import check_lazy


class Fake:
    def __init__(self, name, cl=-3.0, cd=1.2, side=None, pitch=None, bal=None, converged=True, err=None,
                 write_err=None):
        self.name, self.cl, self.cd, self.side, self.pitch, self.bal = name, cl, cd, side, pitch, bal
        self.converged, self.err, self.write_err, self.calls = converged, err, write_err, []

    def write_case(self, spec, sub):
        self.calls.append(["write", spec.case_name(), os.path.basename(sub)])
        if self.write_err:
            raise RuntimeError(self.write_err)

    def _res(self, spec, how):
        self.calls.append([how, spec.case_name(), os.path.basename(str(self._sub))])
        if isinstance(self.err, Exception):
            raise self.err
        return cfd.CoeffResult(attitude=spec.attitude, c_lift=self.cl, c_drag=self.cd, c_side=self.side,
                               c_pitch=self.pitch, aero_balance_front=self.bal, converged=self.converged)

    def run_case(self, spec, sub):
        self._sub = sub
        return self._res(spec, "run")

    def read_result(self, spec, sub):
        self._sub = sub
        return self._res(spec, "read")


class Nameless(Fake):
    def __init__(self, **kw):
        super().__init__("x", **kw)
        del self.name


def _er(er):
    d = plain(er.as_dict())
    f = er.fused
    d["fused"] = {k: plain(getattr(f, k)) for k in ("c_side", "c_pitch", "aero_balance_front", "force_monitor_range",
                                                    "notes")}
    d["prov"] = plain(f.provenance.__dict__) if f.provenance else None
    return d


ROSTERS = {
    "agree": lambda: [Fake("a", -3.0, 1.2, side=0.1, pitch=0.02, bal=0.45), Fake("b", -3.06, 1.23, side=0.2),
                      Fake("c", -2.97, 1.19, bal=0.47)],
    "disagree": lambda: [Fake("a", -3.0, 1.2), Fake("b", -3.6, 1.2)],
    "cd_disagree": lambda: [Fake("a", -3.0, 1.2), Fake("b", -3.0, 1.5)],
    "edge": lambda: [Fake("a", -2.0, 1.0), Fake("b", -2.1, 1.05)],
    "holes": lambda: [Fake("a"), Fake("b", err=cfd.SolverUnavailable("no licence")), Fake("c", converged=False),
                      Fake("d", err=ValueError("x" * 60)), Fake("e", cl=None)],
    "none": lambda: [Fake("a", err=cfd.SolverUnavailable("no licence")), Fake("b", converged=False)],
    "zero_mean": lambda: [Fake("a", 1e-13, 1.0), Fake("b", -1e-13, 1.0)],
    "single": lambda: [Fake("solo", -2.5, 1.1, side=0.3, pitch=0.1, bal=0.5)],
    "nameless": lambda: [Nameless(), Fake("n")],
    "triple": lambda: [Fake("a", -3.0, 1.2, side=0.1, pitch=0.01, bal=0.4), Fake("b", -3.0, 1.2, side=0.2, pitch=0.02,
                                                                             bal=0.45),
                       Fake("c", -3.0, 1.2, side=0.9, pitch=0.12, bal=0.7)],
    "at_tol": lambda: [Fake("a", -3.0, 1.0), Fake("b", -5.0, 1.0)],
    "err48": lambda: [Fake("a"), Fake("b", err=ValueError("y" * 48)), Fake("c", err=ValueError("z" * 49))],
}


def _cases(tmp):
    spec = cfd.CaseSpec(cfd.Attitude(pitch_deg=0.5, ride_height_mm=25.0, speed_ms=15.0), "car.stl")
    spec2 = cfd.CaseSpec(cfd.Attitude(yaw_deg=2.0), "car.stl")
    yield "spread", [E._spread_pct(v) for v in ([], [1.0], [1.0, None], [1.0, 1.1], [-2.0, -2.2, -1.8], [1e-13, -1e-13],
                                               [0.0, 0.0])]
    yield "reduce", [E._reduce(v, h) for v in ([], [None], [1.0, 2.0, 6.0], [3.0, None, 1.0]) for h in ("mean", "median",
                                                                                                    "other")]
    for name, mk in ROSTERS.items():
        for kw in ({}, {"reduction": "median"}, {"min_members": 3}, {"agreement_tol": 10.0}, {"min_members": 0},
                   {"agreement_tol": 50.0}):
            members = mk()
            sol = E.EnsembleTunnelSolver(members=members, **kw)
            er = sol.solve_detailed(spec, str(tmp))
            yield f"{name}.{sorted(kw.items())}", [_er(er), plain(sol.read_result(spec, str(tmp)).c_lift),
                                                   plain(sol.run_case(spec, str(tmp)).c_drag), sol._member_names,
                                                   [m.calls for m in members]]
    w = E.EnsembleTunnelSolver(members=[Fake("ok"), Fake("bad", write_err="disk full")])
    d = w.write_case(spec, str(tmp))
    assert w.write_case(spec, str(tmp)) == d
    yield "write", [os.path.relpath(d, str(tmp)), sorted(os.listdir(d)),
                    open(os.path.join(d, "bad", "WRITE_FAILED.txt")).read(), os.listdir(os.path.join(d, "ok"))]
    default = E.EnsembleTunnelSolver()
    yield "default", [default._member_names, default.reduction, default.agreement_tol, default.min_members,
                      default.turbulence_model, default.fidelity.value, E.EnsembleTunnelSolver.name,
                      E.DEFAULT_MEMBER_NAMES]
    yield "default.solve", _er(default.solve_detailed(spec, str(tmp)))
    yield "default.prov", [plain(default.provenance().__dict__), plain(default.provenance(2, 3.25, ["a", "b"]).__dict__),
                           plain(default.provenance(0, math.nan).__dict__)]
    custom = E.EnsembleTunnelSolver(turbulence_model="kEpsilon", fidelity=cfd.SolverFidelity.DES, mesh_params={"x": 1})
    yield "custom", [custom.turbulence_model, custom.fidelity.value, plain(custom.provenance().__dict__)]
    mat = E.EnsembleTunnelSolver(members=ROSTERS["agree"]())
    res = mat.solve_matrix([spec, spec2], str(tmp))
    mcalls = [m.calls[:] for m in mat.members]
    yield "matrix", [[_er(r) for r in res], [plain(r.c_lift) for r in E.fused_results(res)],
                     [_er(r) for r in mat.solve_matrix([spec2], str(tmp), run=False)], mcalls,
                     [m.calls for m in mat.members]]
    for bad in ({"reduction": "mode"}, {"members": []}):
        with pytest.raises(ValueError) as e:
            E.EnsembleTunnelSolver(**bad)
        yield f"bad.{sorted(bad)}", str(e.value)
    yield "outcome", [[E.MemberOutcome("x").ok, E.MemberOutcome("x").ran],
                      [E.MemberOutcome("x", cfd.CoeffResult(spec.attitude, -1.0, 1.0, converged=True)).ok]]


def test_every_roster_fused(tmp_path):
    check_lazy("aero_ensemble", _cases(tmp_path))
