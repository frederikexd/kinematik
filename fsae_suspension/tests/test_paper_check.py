# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The claim-by-claim paper check (scripts/paper_check.py) on the published manifests: every claim's recomputed value,
rounded as printed, is in the text the paper prints. The documents themselves are checked when the script is run on
them; here the values are checked against the quoted text."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import paper_check as pc                                                  # noqa: E402
from paper_claims import CLAIMS                                           # noqa: E402


def test_every_claim_holds_on_the_published_manifests():
    rows = pc.run(CLAIMS)
    bad = [r for r in rows if r["ok"] is False]
    assert not bad, pc.report(bad)
    skipped = [r for r in rows if r["ok"] is None]          # only the packaging claims need the STEP file
    assert len(skipped) <= 30


def test_claim_ids_are_unique_and_quotes_carry_numbers():
    ids = [c.id for c in CLAIMS]
    assert len(ids) == len(set(ids))
    assert all(any(ch.isdigit() for ch in c.quote) for c in CLAIMS)
