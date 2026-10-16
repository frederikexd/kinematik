# Privacy and data

KinematiK is a free design tool for Formula Student and Formula SAE teams, built and run by one person, Frederik Thio. This page says plainly what the app stores, where it lives, who can see it and how to remove it.

Questions or deletion requests: kinematik.kng1q@8shield.net

## What is stored

| Data | Why | Where |
| --- | --- | --- |
| Your email address | To sign you in and to show your team who is in a workspace | Supabase Auth |
| Your password | To sign you in | Supabase Auth, stored only as a salted bcrypt hash. KinematiK's own code never stores or logs your password, and nobody, including me, can read it |
| Files you upload (CAD models, spreadsheets, CSVs, documents) | To run the tools on your car and share files with your team | Inside your team's workspace in the Supabase database. Large CAD parts go in a private Supabase Storage bucket |
| Voice memos you record in notes | So your team can play them back | Inside the note, in your team's workspace (memos over about 2.5 MB are not saved). Transcription runs on the app's own server; the audio is not sent to any speech service |
| Project data (hardpoints, settings, results, notes, manifests) and save history (who saved what, and when) | So your work persists and your team can share it | Inside your team's workspace in the Supabase database |
| Usage events (which tabs and features are used, timings, errors, app version) | To find bugs and see which tools are useful | Supabase database, analytics table |
| A visitor id | To count unique and returning visitors | A cookie named `kinematik_vid`. If cookies are blocked, a SHA-256 hash of your IP address and browser type is used instead; the raw IP address is not stored |

An optional name and subteam that you type in yourself may be attached to usage events. Nothing else about you is collected. There is no advertising, no data is sold, and nothing is shared with anyone other than the two hosting providers that run the app (below).

## Where it lives

- **The app** runs on Streamlit Community Cloud.
- **Accounts, files and project data** are stored in a Supabase project hosted in the West US (North California) region. Supabase encrypts stored data at rest, and all traffic between your browser, the app and the database uses HTTPS (TLS).

## Who can see your data

- **Your team.** Access is enforced in the database itself with row-level security. A user can only read and write the workspaces they are a member of, with the role they were given (owner, lead, member or viewer). This holds even if the app's own code had a bug.
- **Workspace owners and leads** can see who is in their workspace (names and emails) and who saved what, and when.
- **Linked workspaces.** If your team links its workspace to a shared one, members of that shared workspace can see what is shared there.
- **The administrator.** As the person who runs the database, I have technical access to it through Supabase's admin console. I do not browse team data. I access it only to fix a problem, or when you ask me to.

## How long it is kept, and how to delete it

Your data is kept while your account exists. To delete your account and everything stored for it, email kinematik.kng1q@8shield.net from the address you signed up with. I will delete your account, your uploaded files and your project data within 30 days and confirm by email. Files you shared into a team workspace belong to that workspace, so a workspace owner can also remove them. Usage events that carry only a visitor id, with no name or account, are kept for 12 months for usage statistics.

If you would rather not create an account at all, the full code is open source. You can run KinematiK on your own computer from https://github.com/frederikexd/kinematik, and nothing leaves your machine.

## Changes

If this page changes, the date below changes with it, and the history is in the repository.

Last updated: 15 October 2026
