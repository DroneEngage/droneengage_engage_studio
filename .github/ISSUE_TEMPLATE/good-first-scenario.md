---
name: Good first scenario
about: A self-contained scenario/test contribution - a good first issue
title: "[scenario] "
labels: ["good first issue", "scenario"]
---

## What the scenario proves

<!-- one paragraph, the same "what it proves" style every tests/**/*.yaml
     header carries - ci/check_library.py enforces it -->

## Files

- `scenarios/<name>.yaml` or `tests/<feature>/<name>.yaml`
  (`./sim new <name>` scaffolds it; see `docs/SCENARIOS.md`)
- optional world: `worlds/<name>.yaml` (`docs/WORLDS.md`)
- optional package: `tests/packages/` (test-only) or `packages/` (shared)

## Acceptance

- [ ] `./sim validate <file>` passes
- [ ] `./sim run <file>` passes on a clean machine (quote the
      `N/M assertions passed` line)
- [ ] the header paragraph says what it proves
- [ ] human-driven scenarios pair a `<name>.md` runbook
