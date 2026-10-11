# examples/mission_square — how to run

The smallest end-to-end flight: one copter on the **real** comm server
is deployed the `copter_square` package over the wire, armed, started,
and must journal "mission main completed". The green path every bigger
scenario builds on: deploy → start → completed on the real binaries.

## Run

```bash
./sim run scenarios/examples/mission_square.yaml
```

Expected PASS: exactly one `RUN ... started` journal line, one
`mission main completed`, and zero `comm line offline`. teardown: land.

## Notes

- `transport: real` exercises the real `droneengage_server` routing —
  spawned fresh for the run.
- The GUIDED-before-deploy ordering is deliberate: de_mavlink refuses
  nav loads in AUTO. `arm` waits out SITL's ~1–2 min GPS/EKF warm-up.
- `verify: {cli: tools/verify_wrap.js}` wraps the run with the standard
  result capture into `var/results/`.
