# scenarios/p4b — Phase-4B SAR acceptance scenarios

**Pending P4B-10.** The SAR end-to-end acceptance scenarios (A–I:
nominal coverage search, fault variants, seeded reruns against the
SAR `.deop` result artifacts) are intentionally absent — they depend on
the P4B-10 scenario definitions and result artifacts, which are out of
scope for the Phase-5 library task (TASK-P5-09).

This folder is the agreed landing point: P4B-10 adds `scenarios/p4b/*.yaml`
plus any SAR ops under `scenarios/ops/` without changes to the runner or
the library layout. Until then, the closest live coverage is:

- `scenarios/faults/operator_rejects.yaml` — the p4b08
  confirm/veto path (operator dismisses a `det.fire` target),
- `tests/p4b08_confirm.yaml` — the full confirmation-policy legacy test,
- `tests/p4f03_reboot.yaml` — pending-signal resync across a reboot.
