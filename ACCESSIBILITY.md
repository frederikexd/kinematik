# Accessibility

KinematiK should be usable by every engineer on a team, including people who use a screen reader, magnification, a keyboard only, or who have color vision deficiency. It is maintained by one person and has not been audited against WCAG 2.2; this page says honestly where it stands.

## Supported environments

- **Hosted app:** current desktop versions of Chrome, Edge, Firefox and Safari. The layout is built for screens at least 1280 px wide; phones and small tablets work for reading results but not for editing geometry.
- **Local install:** the app runs from this repository with Docker or pip (see [DEPLOY.md](DEPLOY.md)), so you can use your own browser, zoom level and assistive technology.
- **Without the interface:** every solver is a Python module under `fsae_suspension/suspension/`, and the scripts in `fsae_suspension/scripts/` produce results as text and files, so the engineering does not depend on the web interface.

## Known limitations

- **Charts:** Plotly and Matplotlib charts are not described for screen readers, and some curves are told apart by color alone.
- **3D views:** suspension and CAD views need a mouse or trackpad to rotate and zoom.
- **Theme:** the app uses a fixed dark theme with an amber accent; there is no light or high-contrast mode yet.
- **Preview time limit:** the preview before sign-up ends after five minutes and cannot be extended. Signing up, which is free, removes the limit.
- **Custom components:** some controls come from Streamlit and third-party components whose keyboard and screen-reader support the project does not control.

## Reporting a barrier

Open an issue with the **Accessibility barrier** template, or email kinematik.kng1q@8shield.net with "KinematiK accessibility" in the subject if you would rather not post publicly. Say which page or feature, what you were trying to do, and the browser and assistive technology you use. Barriers are treated as bugs, and you will get a reply within 7 days.
