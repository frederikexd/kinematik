# ============================================================================
#  KinematiK — seal_p1.py
#
#  Seals P1 — "the shim prescription puts alignment in spec on the first try" —
#  for the front and rear corners of the 2027 car.
#
#  P1 is the right first claim for a car designed from scratch: it is binary,
#  it needs no hardpoint-band derivation, and nobody on the team knows the
#  answer yet. That last part is what makes it worth sealing.
#
#  BEFORE RUNNING:
#    1. Fill in CAMBER_BAND_DEG and TOE_BAND_DEG below. Do not accept my
#       defaults without checking them against YOUR tire (see the note there).
#    2. Read the failure text. If you would not publish those sentences, edit
#       them now — after sealing they are fixed.
#
#  Run:  python3 scripts/seal_p1.py            # dry run, prints, seals nothing
#        python3 scripts/seal_p1.py --seal     # seals and writes the JSON
#
#  Then COMMIT the JSON the same day. The git timestamp is the cheapest
#  possible proof that the band was fixed before the measurement existed.
# ============================================================================
from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from suspension import prevalidation as pv          # noqa: E402

# --------------------------------------------------------------------------- #
#  THE TWO NUMBERS. These are the whole point — get them right.
# --------------------------------------------------------------------------- #

#: Camber band, degrees.
#:
#: NOT grip-derived. Against the generic Pacejka curve, ±1.0° of camber error
#: costs ~0.04% of peak mu — the optimum is broad and flat, so deriving this
#: from grip would give a uselessly wide number.
#:
#: >>> RE-CHECK THIS AGAINST YOUR TTC-FITTED TIRE, not default_tire(). If your
#: >>> real curve is peakier, grip decides after all and this number changes.
#:
#: 0.25° = ~3x the honest repeatability of a digital angle gauge (~0.10°).
#: You cannot verify tighter than you can measure.
CAMBER_BAND_DEG = 0.25

#: Left-right camber delta. The constraint that actually matters: both corners
#: 0.3° off the SAME way is far less harmful than one 0.15° off the other way.
CAMBER_LR_DELTA_DEG = 0.20

#: Toe band, degrees. Toe is far more sensitive than camber for stability and
#: drag, so this is tighter despite the same instrument.
TOE_BAND_DEG = 0.10

AUTHOR = "Frederik Thio"
CAR = "2027 FSAE EV"
OUT_DIR = pathlib.Path(__file__).resolve().parents[1] / "prevalidation"

#: Which corner is protected if April runs short. Decide NOW, in the calm —
#: not at 1am three weeks before Michigan.
PROTECTED_CORNER = "front"


# --------------------------------------------------------------------------- #
def _method(corner: str) -> str:
    return (
        f"{corner.capitalize()} corner. Photogrammetry of as-built hardpoints "
        "against a shop-machined reference artifact, fed to "
        "kinematik_stochastic.alignment_prescription(). Shims set ONCE to that "
        "prescription, no adjustment. Camber read with a digital angle gauge "
        "on the machined locating fixture; toe read with machined toe plates. "
        "Measured before the corner goes on the car."
    )


def _failure_text(corner: str) -> str:
    return (
        f"The alignment prescription did not put the {corner} corner in spec on "
        "the first attempt. N further iterations were required; final camber "
        "error was X deg and toe error Y deg. Attribution is recorded "
        "separately: a miss caused by as-built geometry falling outside the "
        "range the prescription was solved over is a DIFFERENT finding from a "
        "miss caused by the prescription arithmetic itself, and the 2027 car "
        "is the team's first designed from scratch, with no prior shop "
        "capability data. Both attributions are reported."
    )


def build(corner: str) -> pv.Registration:
    reg = pv.Registration(
        id=f"elbee-2027-{corner}-p1",
        title=f"P1 — shim prescription first-time fit, {corner} corner",
        author=AUTHOR, car=CAR)

    reg.add_claim(pv.Claim(
        id="P1_camber_first_try", kind=pv.KIND_BINARY, predicted=True,
        unit="deg",
        statement=(
            f"Setting shims once to the kinematik_stochastic prescription puts "
            f"{corner} camber within ±{CAMBER_BAND_DEG:g}° of target, with "
            f"left-right delta ≤{CAMBER_LR_DELTA_DEG:g}°, on the first "
            f"attempt."),
        method=_method(corner),
        failure_text=_failure_text(corner),
        engine="kinematik_stochastic"))

    reg.add_claim(pv.Claim(
        id="P1_toe_first_try", kind=pv.KIND_BINARY, predicted=True,
        unit="deg",
        statement=(
            f"The same single shim-setting attempt puts {corner} toe within "
            f"±{TOE_BAND_DEG:g}° of target."),
        method=_method(corner),
        failure_text=_failure_text(corner),
        engine="kinematik_stochastic"))

    return reg


def main(argv) -> int:
    do_seal = "--seal" in argv

    print("=" * 74)
    print("P1 — shim prescription first-time fit")
    print("=" * 74)
    print(f"  camber band       ± {CAMBER_BAND_DEG:g}°")
    print(f"  L-R camber delta  ≤ {CAMBER_LR_DELTA_DEG:g}°")
    print(f"  toe band          ± {TOE_BAND_DEG:g}°")
    print(f"  iterations        zero — one attempt")
    print(f"  protected corner  {PROTECTED_CORNER} (the one kept if April runs short)")
    print()

    if CAMBER_BAND_DEG == 0.25 and TOE_BAND_DEG == 0.10:
        print("  ⚠  These are still the suggested defaults. Confirm them against")
        print("     YOUR TTC-fitted tire and your own measured gauge")
        print("     repeatability before sealing. A band you did not check is")
        print("     a band you cannot defend in a paper.")
        print()

    regs = [build("front"), build("rear")]

    for reg in regs:
        print("-" * 74)
        print(f"{reg.id}")
        for c in reg.claims:
            print(f"  [{c.id}]  {c.statement}")
        if do_seal:
            digest = reg.seal()
            OUT_DIR.mkdir(exist_ok=True)
            path = OUT_DIR / f"{reg.id}.json"
            pv.save(reg, str(path))
            print(f"  SEALED  {reg.sealed_at}  digest {digest}")
            print(f"  written {path.relative_to(OUT_DIR.parent)}")
        else:
            print("  (dry run — nothing sealed)")

    print()
    if do_seal:
        print("=" * 74)
        print("COMMIT THESE FILES NOW.")
        print("  git add prevalidation/ && git commit -m 'Seal P1 pre-registration'")
        print()
        print("The git timestamp is the proof that the band was fixed before")
        print("the measurement existed. A sealed file sitting uncommitted on")
        print("your laptop proves nothing to anyone.")
        print("=" * 74)
    else:
        print("Dry run. Re-run with --seal when the two numbers are yours.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
