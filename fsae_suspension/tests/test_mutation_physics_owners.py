"""Characterization suite for suspension/physics_owners.py (mutation gate); see tests/_golden.py. The whole register,
every lookup (and an unknown key), the defaults of a bare Quantity and the human-readable report."""
from suspension import physics_owners as P
from tests._golden import plain
from tests._golden_lazy import check_lazy


def _cases():
    yield "register", plain(list(P.PHYSICS_OWNERS))
    for q in P.PHYSICS_OWNERS:
        yield f"lookup.{q.key}", [P.owner_of(q.key), P.mirrors_of(q.key)]
    yield "lookup.unknown", [P.owner_of("nope"), P.mirrors_of("nope"), P.owner_of("")]
    yield "bare", plain(P.Quantity("k", "d", "o"))
    yield "report", P.unowned_report()
    yield "all", P.__all__
    real = P.PHYSICS_OWNERS
    try:                                  # no registered quantity has two delegates: show how they are joined
        P.PHYSICS_OWNERS = tuple(real) + (P.Quantity("k2", "two delegates", "o", delegates=("a", "b")),)
        yield "report.two", P.unowned_report()
    finally:
        P.PHYSICS_OWNERS = real


def test_every_owner_and_mirror():
    check_lazy("physics_owners", _cases())
