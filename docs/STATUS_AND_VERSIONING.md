# Status & Versioning

This repository is for **NASA Lunabotics 2026–27**.

## Status Labels
Use these labels in documentation and design discussion:

| Label | Meaning |
|---|---|
| **CURRENT — 2026–27** | Current-season requirement or approved implementation |
| **WORKING — 2026–27** | Active candidate/development baseline; not frozen |
| **LEGACY — 2025–26** | Previous robot/design; reference only |
| **HISTORICAL — 2024–25** | First-year robot/design; reference only |
| **SUPERSEDED** | Explicitly replaced by a later design/source |
| **TBD** | Not yet decided |

## Source / Chronology Rule
When information conflicts:
1. **2026–27 competition compliance:** current NASA 2026–27 Guidebook and official updates.
2. **2026–27 design:** current-season approved design decisions in this repository.
3. **2025–26 final robot:** final 2025–26 P&D / Systems Engineering Paper.
4. **2025–26 design rationale:** PDR, then SRR, then PMP.
5. **2024–25:** historical reference only.

The old competition guidebook, old code, and old trade-study winners must never be silently presented as current.

## Code Placement
- Current 2026–27 code belongs in normal source directories.
- If old code is imported only for comparison/reuse, place it under `legacy/2025-26/` or clearly mark its season.
- Before legacy code becomes current, document the decision in `docs/DECISIONS.md` and move/refactor it into the current source tree.
