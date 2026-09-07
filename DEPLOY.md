# Deployment and database setup


## Database setup

Run `suspension/analytics_hardening.sql` in Supabase once. Safe to re-run (drop-then-create, idempotent grants). This creates the analytics views

Then run these four, in this order. All are idempotent.

| # | File | What it fixes |
|---|---|---|
| 1 | `fix_null_role_guard.sql` | **Security.** Every admin guard read `if workspace_role(ws) not in ('owner','lead') then raise`. For a non-member `workspace_role()` returns NULL, and `NULL not in (...)` is NULL rather than TRUE — so the branch never fired and the guard passed. It rejected members and viewers correctly while letting a non-member through. Reproduced end to end: a non-member could mint an invite to a workspace they were not in, redeem it, and join. Run this first. |
| 2 | `fix_invite_permission.sql` | Members could not create invite links — the rule was widened on the client only, so the button showed and the server refused with a raw Postgres error. |
| 3 | `fix_feature_allowlist.sql` | 16 of 40 tabs were discarding every analytics event: `ae_feature_fk` requires each feature to exist in `known_features` and only 24 were seeded. Inserts batch atomically, so one bad row failed the whole batch. A trigger now auto-registers unknown features. |
| 4 | `add_workspace_analytics.sql` | Adds `workspace_id` plus the per-team views `v_workspace_weekly`, `v_workspace_adoption` and `v_feature_traction`. Ships with `suspension/analytics.py` — the column stays NULL until the app change deploys, and historical events cannot be back-filled because the link never existed. |

---

## Deploy order

1. Push `streamlit_app.py`, `project.py` and `coordinate_frames.py` together — the handover builder gained a `frame_tag` parameter that the app passes, so they are a matched set.
2. Push `suspension/analytics.py` with `streamlit_app.py` as before — still a matched pair.
3. Run `suspension/analytics_hardening.sql` in Supabase.
4. Push `suspension/pcb_doctor.py`, `suspension/pcb_altium.py` and `suspension/pcb_altium_binary.py` together — `parse_board()` dispatches to both readers, so they are a matched set. `olefile` must be in `requirements.txt` for the binary reader; without it a native `.PcbDoc` falls back to the ASCII export instructions rather than raising.
5. Confirm the build stamp in the Usage section matches the version at the top of this file, and the Streamlit runtime reads `>= 1.58.0`.

---

## Architecture

**Kinematics engine** — architecture-agnostic multibody solver (`suspension/topology.py`). Rigid bodies defined by points, constraint primitives (distance links, ball/pin coincidence, prismatic slider, planar, revolute, rack translation, beam-axle roll), assembled into a `Mechanism` and solved by branch-stable Levenberg–Marquardt sweep.

**Topology library** (`suspension/topologies.py`) — double wishbone, MacPherson strut, multi-link (3/4/5-link), trailing arm, semi-trailing arm, solid axle (Panhard or Watts), twist-beam, truck steer linkage, and `from_links` for experimental corners.

**Vehicle dynamics layer** — roll-centre migration, anti-dive/anti-squat, load transfer, grip balance, all topology-independent via `GenericKinematics` adapter (`suspension/adapter.py`).

**Coordinate frames** (`coordinate_frames.py`) — pure-Python frame registry and transform core. Every conversion routes `frame A → world → frame B` through one auditable path; all frames are proper rotations (det = +1), so points, forces, moments and angular rates share one transform and only points shift by the datum. Rotation senses are derived from the basis via the right-hand rule. Datums resolve live from the vehicle parameters (`a = L·(1 − weight_dist_front)` from static axle-load balance). Self-tested with exact identities, no fuzz: `python3 coordinate_frames.py`.

**Analytics** (`suspension/analytics.py`) — privacy-respecting usage tracking. Identity is a random per-session UUID (plus a browser cookie for return-visit counting); no IP addresses or device fingerprints are collected or stored. A member name is recorded only if the user types one in (opt-in). Telemetry never blocks the UI and a telemetry failure can never crash the app. Only three event types are written (session start, workflow complete, error); raw events are purged after 30 days.

Events also carry a `workspace_id` so usage is answerable per team rather than summed across all of them. Attribution is at the workspace level only — individuals stay as anonymous as they were. Because Streamlit runs the script top to bottom and the analytics init sits below the widget instrumentation, a run's feature events are emitted *before* the workspace is resolved; `set_workspace()` therefore back-fills events still in the queue, matching on session_id so a process serving several browsers can never stamp one team's event with another's workspace.

---
