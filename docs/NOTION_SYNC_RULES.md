# OFDM PRS / SS-TWR Notion synchronization rules

## Scope
This file governs updates to the existing OFDM PRS / SS-TWR research tracking system from Codex CLI. It does not grant authorization to modify other Notion content.

Notion workspace must be verified as `Jason Hi's Notion` before writing.
Research home: https://app.notion.com/p/4240edd7006d81879003fbd8b571bfe4

Authorized destination databases:
- Milestones: https://app.notion.com/p/4240edd7006d816aa677e875b765861f
- Experiments: https://app.notion.com/p/4240edd7006d815b9cb9d8c9666e610f
- Tasks: https://app.notion.com/p/4240edd7006d8128b493e0e6224a5e11
- References: https://app.notion.com/p/4240edd7006d81fb91d2d1d8763e320b

## Write policy
1. After completing a coding/testing task, inspect `git diff`, test command output, logs, and available RF data. Draft a concise update and show the exact Notion rows/properties to be created or changed.
2. Before any Notion create, update, delete, or move operation, ask for explicit user approval. Do not write if approval is absent; do not automatically retry writes after an approval failure.
3. Fetch the target database schema and relevant existing pages immediately before editing. Match by project, function, experiment identity, and date; update an existing record rather than create a duplicate. Do not change unrelated pages or databases.
4. Never delete, archive, move, overwrite large bodies of existing text, change access permissions, or publish/share Notion pages without separate express approval.
5. After any approved write, re-fetch the affected page and report the Notion URL and changed fields. If writing fails, report failure without claiming success.

## Evidence and status rules
- **Implemented** means source changes exist. It does NOT imply build success, QA success, RF verification, or performance accuracy.
- **Software verified** requires a named test command and successful result with version/commit. Unit or synthetic tests are never proof of RF accuracy.
- **RF verified** requires real hardware measurements with raw logs or a reproducible result, hardware and RF setup, date, waveform settings, reference/ground-truth distance, sample count and relevant error statistics. If these are unavailable, mark as `已回報` / `待重測` or explain missing evidence; do not use `已驗證`.
- **Milestone `已完成`** requires that its specific `完成條件` is met. For combined software/RF milestones, leave `進行中` until RF conditions are met. State partial progress in `備註`.
- **Experiment `已驗證`** requires evidence review; a verbal or README historical result is `已回報` and must keep provenance.
- Failed or skipped tests must be recorded explicitly. Do not hide failures or invent values, dates, build outputs, measurements, or commit IDs.
- Separate `integer range`, `corrected range`, and `positioning error`; do not mix their mean/std statistics.
- Do not claim that the custom OFDM PRS-like PHY is a standards-complete 3GPP NR PRS implementation.

## Existing project facts (baseline, not fresh verification)
- PHY geometry: 512 FFT, 64 CP, 127 Gold-spread OFDM symbols at an example 30 MS/s. 80-bit packet v2 with CRC.
- Timed POLL/RESPONSE SS-TWR, bidirectional fine-delay correction, static scheduled 1..8 anchor cycles and complete epoch output have been implemented according to README.
- One-anchor Phase B and two-anchor Phase C RF runs were *reported* successful; 3/4/8-anchor RF was not verified by the supplied materials.
- Online Position Solver, DS-TWR, and clock-skew correction were not implemented in the provided README.
- FFT-ZC accuracy remains experimental; existing historical measurements must not be presented as current version tests.

## Update mapping
- `研究里程碑`: Update fields `狀態`, `完成條件`, `備註` only for matching milestone. Avoid marking all project work done because one subcomponent builds.
- `實驗紀錄`: Record setup, results, raw log paths/links, versions, and reference distance. Use `已回報` for unverified imported or historical results and `待重測` for outstanding regressions.
- `研究待辦`: Add/update actionable tasks with reproducible acceptance criteria, priority, domain, and a due date only when genuinely supplied.
- `研究文獻`: Add references only when the citation can be verified; preserve bibliographic uncertainty and do not invent DOI URLs.

## End-of-task synchronization sequence
1. Summarize completed edits and `git diff --stat`.
2. Summarize tests as **passed**, **failed**, **not run**, or **inconclusive**. Include exact command and concise evidence.
3. Describe any USRP/RF tests separately from software tests.
4. Find existing Notion records and present a proposed update diff with evidence.
5. Request approval. Only after approval, write via Notion MCP and verify by re-reading.
