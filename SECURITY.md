# Security policy

KinematiK is maintained by one person. Security reports are taken seriously and answered as fast as I can, but there is no paid security team behind this project.

## Reporting a vulnerability

Please do not open a public issue for a security problem.

- Use GitHub's private vulnerability reporting: the **Security** tab of this repository, then **Report a vulnerability**, or
- email kinematik.kng1q@8shield.net with "KinematiK security" in the subject.

Include what you found, how to reproduce it, and what an attacker could do with it. You will get an acknowledgement within 7 days and an update when it is fixed. If you want credit, you will be named in the changelog.

Good-faith research is welcome. Please only test against your own account and workspace, do not access or change other people's data, and do not run load or denial-of-service tests against the hosted app.

## Supported versions

Only the latest version on the `main` branch, which is what the hosted app runs, receives security fixes.

## What is in place

- **Accounts** are handled by Supabase Auth. Passwords are stored only as salted bcrypt hashes; KinematiK's code never stores or logs them.
- **Row-level security** is enabled on every table. On every table that holds team data (workspaces, members, invites, projects, save history, CAD parts and assemblies) it is also forced, so the database limits each user to their own workspaces and roles, independent of the app code.
- **Privileged database functions** (owner and lead oversight) re-check the caller's role before returning anything.
- **CAD files** sit in a private storage bucket, not a public one.
- **XML inside uploaded spreadsheets and documents** is parsed with `defusedxml`, which blocks XML-based attacks.
- **Voice memos are transcribed on the app's own server**; audio is not sent to an outside service.
- **Automated checks** run on every change: the test suite, CodeQL code scanning, `pip-audit` for known vulnerabilities in dependencies, and `bandit` for security issues in the source.
- **All traffic** between the browser, the app and the database uses HTTPS.

What is stored, where, and how to delete it: see [PRIVACY.md](PRIVACY.md).

## Files you upload are treated as untrusted

Every file a team uploads is parsed by code that assumes the file may be damaged, mislabeled or hostile.
What that code guarantees, and how it is tested:

- **No silent garbage.** A value that is not a finite number (`nan`, `inf`, `1e999`, JSON `NaN` / `Infinity`) is
  refused with a message naming the file, row and column where that is known. It never becomes part of a model.
  This covers board files, aero maps, run logs, dyno and fuel maps, frame CSVs, STEP tube imports, hardpoint
  tables, track and GPS paths, cone maps, speed traces, `.tir` files, tire coefficients, tire test data, spring
  bench logs, part meshes, reproducibility manifests and project bundles.
- **A message, not a traceback.** A file the parser cannot read raises one error type (`ValueError`) with a
  sentence for the person holding the file.
- **Bounded memory.** A zip upload is unpacked to at most 256 MB per member and 512 MB in all, read to the limit
  rather than trusting the size the archive declares.
- **How it is tested.** `tests/test_pcb_upload_fuzz.py` replays 800 seeded damaged board files on every run.
  `tests/test_untrusted_inputs.py` pins each refusal found by fuzzing about 30 entry points with more than 15 million damaged
  inputs (truncation, deleted and duplicated chunks, numbers replaced by `nan`, `inf`, zero, negative and
  overflowing values, random bytes). That fuzzing found more than 30 defects; each is fixed and has a test.
- **Not covered.** Numbers typed into the app are bounded by their input widgets, not by this layer. Binary CAD
  kernels (STEP tessellation, GLB) are third-party code; their failures are caught and reported, not prevented.
