# examples/link_fault — how to run

The smallest useful scenario: one static comm-only unit on the fake
server gets its link severed for 20 s. The link monitor must see the
unit drop offline and come back. No FC, no operation — the
"hello world" of the fault surface.

## Run

```bash
./sim run scenarios/examples/link_fault.yaml
```

Expected PASS: the monitor saw s1 offline ≥ 5 s at least once, while
the de_comm process itself never died (it is the link, not the unit,
that dropped). teardown: none.

## Notes

- `transport: fake` is required — `fault: link_cut` is a message-level
  fault only the fake server (or a real server with `link_proxy: true`
  byte-level equivalents) supports.
- Copy this file when you want to probe a link-fault case interactively;
  the matching manual command on a live sim is
  `./sim ctl link_cut <unit> <seconds>`.
