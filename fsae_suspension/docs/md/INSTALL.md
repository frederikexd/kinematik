# Where these files go

Your Streamlit Cloud traceback showed:

```
/mount/src/kinematik/fsae_suspension/streamlit_app.py
```

The app lives in a **`fsae_suspension/` subdirectory**, not at the repo root.

```
kinematik/
└── fsae_suspension/          <-- HERE, not the repo root
    ├── streamlit_app.py      (overwrite)
    ├── requirements.txt      (overwrite)
    ├── suspension/
    │   ├── project.py            (overwrite)
    │   ├── status_dashboard.py   (overwrite)
    │   ├── history.py            (overwrite)
    │   ├── release_gate.py       (overwrite)
    │   ├── integration.py        (overwrite)
    │   ├── pt_integration.py     (overwrite)
    │   ├── report_figures.py     (new)
    │   ├── perf.py               (new)
    │   ├── rationale.py          (new)
    │   └── daq_sample.py         (new)
    ├── ui/
    │   └── daq_plan.py       (overwrite)
    ├── tests/                (new, additive)
    └── tools/                (new, additive)
```

Do NOT copy `CHANGES.md`, `INSTALL.md`, `samples/` or `EXAMPLE_*.pdf` into the
repo — reference only. Leave the stale `suspension/streamlit_app.py` near-copy
alone; the live file is `fsae_suspension/streamlit_app.py`.

## One thing to check on YOUR deployment

`ProjectStore.team_name` no longer defaults to a team name. Your existing
`project.json` almost certainly still contains `"team_name": "Elbee Racing"`,
and the resolver treats that exact string as *unset* — so your reports will now
be headed by your **workspace name** instead. If you want the old text back,
type it into **Handover → Team**; anything you type there wins.

Your Supabase row key is deliberately unchanged (`SupabaseBackend.LEGACY_ROW_KEY`).
It is not a display name, and renaming it would point your deployment at a
different, empty row.

## Confirm the right build is running

```
Generated 2026-08-06 09:14 from KinematiK build 3f1cfbaa · Electrics subsystem.
```

**This patch is `build 3f1cfbaa`.**

```bash
python3 -c "import hashlib;print(hashlib.sha256(open('fsae_suspension/streamlit_app.py','rb').read()).hexdigest()[:8])"
```

If the hash does not change after a push, use **Manage app → Reboot**.

## Then verify

```bash
cd fsae_suspension
pip install -r requirements.txt
python3 -m pytest tests/ -q -k "team_naming or rationale or table_capture or doc_coverage or perf or daq_sample or report_figures or store_persistence or status_keys or handover_coverage"
python3 tools/audit_doc_coverage.py
```
