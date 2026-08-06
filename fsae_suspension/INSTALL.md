# Where these files go

Your Streamlit Cloud traceback showed:

```
/mount/src/kinematik/fsae_suspension/streamlit_app.py
```

The app lives in a **`fsae_suspension/` subdirectory**, not at the repo root.
Everything here is relative to that subdirectory:

```
kinematik/
└── fsae_suspension/          <-- HERE, not the repo root
    ├── streamlit_app.py      (overwrite)
    ├── requirements.txt      (overwrite)
    ├── suspension/
    │   ├── project.py            (overwrite)
    │   ├── status_dashboard.py   (overwrite)
    │   ├── report_figures.py     (new)
    │   ├── perf.py               (new)
    │   └── daq_sample.py         (new)
    ├── ui/
    │   └── daq_plan.py       (overwrite)
    ├── tests/                (new, additive)
    └── tools/                (new, additive)
```

Do NOT copy `CHANGES.md`, `INSTALL.md`, `samples/` or the `EXAMPLE_*.pdf` files
into the repo — reference only. Leave the stale `suspension/streamlit_app.py`
near-copy alone; the live file is `fsae_suspension/streamlit_app.py`.

## Confirm the right build is running

Every report header now carries a content hash of the running file:

```
Generated 2026-08-06 09:14 from KinematiK build c85c71ed · Electrics subsystem.
```

**This patch is `build c85c71ed`.** Anything else means the deployment is not
running this code — a completely different problem from the fix not working,
and worth ten seconds to rule out first.

Without generating a PDF:

```bash
python3 -c "import hashlib;print(hashlib.sha256(open('fsae_suspension/streamlit_app.py','rb').read()).hexdigest()[:8])"
```

Streamlit Cloud usually redeploys on push. If the hash does not change, use
**Manage app -> Reboot**.

## Then verify

```bash
cd fsae_suspension
pip install -r requirements.txt
python3 -m pytest tests/ -q -k "table_capture or doc_coverage or perf or daq_sample or report_figures or store_persistence or status_keys or handover_coverage"
python3 tools/audit_doc_coverage.py
```

Expect all green and `zero capture surface: 0 []`.
