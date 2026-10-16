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
