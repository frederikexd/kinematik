# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Re-run one published manifest on the current code and report whether its outputs are byte-identical to the
recorded ones (supplement S13.3). usage: python scripts/verify_run.py manifests/<name>.recorded.genesis.json"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from suspension.genesis_repro import GenesisManifest                                    # noqa: E402

if __name__ == "__main__":
    for p in sys.argv[1:]:
        m = GenesisManifest.from_json(Path(p).read_text())
        t = time.time()
        ok, diff = m.verify(m.run())
        print(m.name, 're-run identical:', ok, diff, 'secs', round(time.time() - t), flush=True)
