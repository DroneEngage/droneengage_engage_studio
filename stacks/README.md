# stacks/ — declarative dev stacks

One yaml per stack. `./sim up <name>` / `./sim down <name>` /
`./sim logs <name> [proc]` / `./sim servers` and `./sim list stacks`
pick every file up — no engine edit. Format:

```yaml
name: de_servers            # must equal the file name minus .yaml
up_timeout_s: 30            # per-proc port-wait budget (default 30)
open_url: "{webclient_url}" # optional: what --open-web opens and
                            # `sim status` shows for this stack
up_note: "..."              # optional line printed after a successful up
procs:                      # launch order - first entry starts first
  - name: de_auth           # proc name: pids/<name>.pid, logs/<name>.log
    kind: node              # node (default): exec <repo>/src/server.js
                            # npm: exec `npm start` (react-scripts dev
                            # server, port from $PORT/SIM_WEBCLIENT_URL)
    repo_env: DE_AUTH_DIR   # sim.local.env / env override key (optional)
    repo: droneengage_authenticator
                            # repo dir: with repo_env this is the
                            # fallback under $DE_CODE; without repo_env
                            # it is used as-is (absolute or under DE_CODE)
    port_env: some_port_key # optional env var that overrides the port
                            # read from the repo's server.config(.local)
```

Stacks are deliberately NOT sims: procs carry a `--<stack>` argv marker
so `simlib.pid_is_ours` stays False — `sim down`, `sim down --all`,
`sim run`'s teardown and `sim clean` can never sweep them. Only
`./sim down <stack>` stops a stack. State lives under `var/<stack
minus any de_ prefix>/`: `launch.<proc>.sh`, `pids/`, `logs/`.
