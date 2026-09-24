# Contributing

## Branches
Create a short-lived branch from `main`:
- `feature/<name>`
- `fix/<name>`
- `test/<name>`
- `docs/<name>`

## Season / Status Rule
Every PR that changes architecture, hardware assumptions, algorithms, interfaces, or requirements must identify the information as one of:

- **CURRENT — 2026–27:** approved/current-season implementation or requirement
- **WORKING — 2026–27:** active candidate or development baseline, not yet frozen
- **LEGACY — 2025–26:** previous robot/design reference
- **HISTORICAL — 2024–25:** first-year robot/reference

Do not copy a legacy choice into current documentation without recording the decision in `docs/DECISIONS.md`.

## Pull Requests
- Keep each PR focused on one change.
- Build/test locally before requesting review.
- Explain hardware and cross-subsystem dependencies.
- Update `docs/INTERFACES.md` when a topic, message, frame, unit, rate, command, timeout, or failure contract changes.
- Update `docs/DECISIONS.md` when a working option becomes the current baseline.
- Do not commit generated ROS 2 build folders, logs, rosbags, credentials, or machine-specific files.

`main` should remain integration-ready and should describe the current season accurately.
