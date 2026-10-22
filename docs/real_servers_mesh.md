# Real servers and mesh topologies in the sim — design

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

Status: **design only**, nothing here is implemented yet.
Companion docs, written in parallel:

- `droneengage_server/wiki/PluginEvents.md`: read-only server event hooks
  and the sim plugin that streams them (task 1).
- `droneengage_simulation_land/docs/port_allocation.md`: the
  `port_allocation_management` registry and the per-sim manifest (task 2).

This doc was first written against the interfaces sketched before
either companion doc existed. It has since been reconciled with both
final docs; section 10 lists each original assumption, how it was
resolved, and the mismatches still open.

## 1. Goal and principles

1. **Real servers by default.** Every comm server, storage server and
   authenticator the sim runs is the production code from its own repo,
   started with a config the sim generates. Nothing is mocked unless the
   test is specifically about a server misbehaving.
2. **Faults live outside the servers.** Transport faults (stall, cut,
   delay, bandwidth) come from `engine/link_proxy.py`, which can be
   placed on any named connection. The servers themselves are never
   patched for a test.
3. **Observe through the product.** Assertions about server internals
   (storage reconnected, child re-authenticated) come from the server's
   own plugin events (task 1), never from scraping logs.
4. **The fake server becomes a narrow tool.** `fake_fleet_server.py` is
   kept only for things a real server can't be made to do: message-level
   loss (dropping a whole ws frame), clock skew (`clock_offset_ms`), and
   frames without timestamps (`noclock`).
5. **Today's sims don't change.** `server: {type: real|fake|external}`
   keeps working exactly as now (section 7).
6. **Mavlink3DMap stays optional.** Nothing here adds a dependency on it.

## 2. What the server code actually supports (verified)

These facts constrain the design. Each was checked against the code, not
the README.

| Fact | Where | Consequence for the sim |
|---|---|---|
| Parent role: `enable_super_server: true` opens a **separate wss listener** on `s2s_super_server_ip:s2s_super_server_port`, using the same `ssl_key_file`/`ssl_cert_file` | `src/server.js:186`, `js_parent_comm_server.js:_startServer` | A parent needs a second allocated port (`<name>.s2s`) |
| Child role: `enable_persistant_relay: true` dials `wss://s2s_relay_to_super_server_ip:port` with `rejectUnauthorized: false`; on close it retries **every 10 s, fixed** | `js_child_comm_server.js:45-60` | Mesh heal assertions need `within_s` ≥ ~15 |
| A server can be **both** parent and child; relay code handles both flags | `js_chat_relay.js:forwardMessage/forwardExternalMessage` | Multi-tier trees are expressible |
| Parent → child S2S auth is **always Ed25519**. The parent challenges every child, checks the signature against `s2s_trusted_server_keys[<child server_id>]`, and terminates on timeout or failure. There is **no shared-secret mode on this link** and no opt-out | `js_parent_comm_server.js:72-108`, `js_s2s_auth.js:fn_verify` | The sim must generate an Ed25519 key pair **per comm server, per run**, and give each parent its children's public keys |
| Child signs with `s2s_my_private_key` and `server_id` | `js_child_comm_server.js:71` | `server_id` must be unique per node; the sim derives it from the node name |
| Storage link: the comm server dials `wss://storage_server_host:storage_server_port`. The storage server runs either Ed25519 (`s2s_cert_enabled: true`, with `s2s_trusted_server_keys`) or shared-secret HMAC (`s2s_cert_enabled: false`, `s2s_shared_secret_file` or `de_s2s_shared_secret` env, **min 16 chars, refuses to start without one**) | `droneengage_storage_server/server.config`, `js_db_proxy_client.js` | One storage node can serve several comm servers. The sim writes one secret file per storage node and points every attached comm server at it |
| Storage server also opens a **dashboard** on `dashboard_port` (default enabled) | storage `server.config` | Disable it in sim (`dashboard_enable: false`), or it needs another port |
| Mission save/load are **system** messages (`ty: s`, mt 9010/9011/9012) handled by the comm server the GCS is connected to, which then calls **its own** storage link. System messages are not relayed over S2S | `js_chat_system_commands.js:111-115`, `js_chat_tasks.js` | Missions are stored by the server the client is attached to. A unit on a child with no `storage:` can't load missions. Sharing one storage node across the mesh is the realistic setup |
| Broadcast messages relay through the whole tree (`forwardExternalMessage`), with `_path` loop prevention | `js_chat_routing.js:148-197` | GCS ↔ units at any depth works for group/broadcast traffic |
| **Individual (one-to-one) messages that arrive from another server are delivered locally only and not relayed further** (`default:` branch: "One-to-one messages are not propagated further") | `js_chat_routing.js:183-188` | A GCS on the root can command a unit **one hop** away, but **not two hops** away (root → child → grandchild). See 2.1 |
| Relayed delivery is scoped by `_aid`/`_gid` (sender's account/group); if that account/group doesn't exist locally, it **falls back to deliver to everyone** | `deliverExternalBroadcast`, `js_andruav_chat_account_rooms.js:131` | Mixing an auth-mode parent (real account ID) with local-mode children (`local_server_account_id`, default `'1'`) "works" only through the fallback. The sim must set matching account IDs on purpose (section 5) |
| Local mode: `local_server_enabled: true` accepts de_comm without the authenticator; account = `local_server_account_id` (default `'1'`), group `'1'`, permissions `0xffffffff`. `ignore_auth_server: true` stops it dialing the authenticator | `js_chat_connection.js:58`, `js_andruav_comm_server.js:356` | Matches the shipped drone child config `deployment/server.s2s.drone.config` |
| Authenticator picks the comm server for a login: **the server already serving that account**, else the least-loaded online one. The client can't choose | `droneengage_authenticator/.../js_comm_server_manager.js:fn_selectServerforAccount` | Per-unit `via:` **can't** be done through one authenticator (section 5) |
| The sim's spawned authenticator runs `s2s_auth_enabled: False`, account type `single` | `simlib.py:gen_server` | Unchanged; comm servers still try S2S auth and that passes |

The shipped `deployment/server.s2s.super.config` and
`deployment/server.s2s.drone.config` are the reference pair: a parent with
auth, and a **child in local mode with `ignore_auth_server: true`**. The
design follows that shape.

### 2.1 The two-hop individual-message limitation

`GCS@root → unit@grandchild` fails for one-to-one commands (`tg = <party
id>`). The child receives the message from the root, finds no local party
with that ID, and drops it. Broadcasts (`_AGN_`, `_GD_`, group) do get
through.

The sim should **surface this, not hide it**:

- The loader **warns** when a unit sits two or more hops below the node
  the GCS bridge connects to.
- A scenario `tests/mesh/two_hop_individual.yaml` is checked in marked
  `expect: fail`, to document the current behaviour. It turns into a
  regression test if the server later relays individual messages along
  `_path`.
- This is a server behaviour finding, filed as a follow-up for the
  server. It is out of scope for this design.

## 3. YAML topology

### 3.1 `servers:` replaces `server:`

```yaml
servers:
  hq:                         # node name = server_id suffix, port-lease prefix
    type: comm                # comm | storage | auth | external | fake
    auth: hq_auth             # login through this auth node (omit = local mode)
    storage: db1              # attach a storage node
    gcs: true                 # the GCS bridge connects here (exactly one node)
  edge1:
    type: comm
    relay_to: hq              # child of hq: enable_persistant_relay -> hq.s2s
    storage: db1              # same storage as hq (realistic)
  hq_auth:
    type: auth                # droneengage_authenticator, account_storage_type single
  db1:
    type: storage             # droneengage_storage_server, fresh DB per run
    persist: false            # true: DB kept across `sim down/up` (restart tests)
```

Node types:

| type | Process | Notes |
|---|---|---|
| `comm` | `droneengage_server/src/server.js` | Parent automatically when any node or unit has `relay_to: <it>` (`enable_super_server`); child when it has `relay_to` (`enable_persistant_relay`); can be both |
| `storage` | `droneengage_storage_server` | Shared-secret mode by default (`s2s_cert_enabled: false`), with a per-run secret file. `ed25519: true` switches to cert mode using the comm servers' per-run public keys |
| `auth` | `droneengage_authenticator` | As today: `single` account, `s2s_auth_enabled: false` |
| `external` | none | A stack the user runs themself. Only valid as the `gcs:` node, and nothing may `relay_to` it unless the user's server trusts our key (section 3.4) |
| `fake` | `engine/fake_fleet_server.py` | Message-level loss, clock skew, `noclock`. Can't take part in a mesh or storage (it speaks neither S2S nor storage) |

Validation rules (all checked at load time; errors name the node):

- Exactly one node has `gcs: true`. If there's a single comm/fake/external
  node, it's the default.
- `relay_to` must name a `comm` node. No cycles. A parent and its child
  can't be the same node.
- `storage:` must name a `storage` node. `auth:` must name an `auth` node.
- A child with `auth:` is **refused** in phase 3 (section 5). Children
  run in local mode.
- `fake` and `external` nodes take no `relay_to`, `storage` or `auth`
  edges.

### 3.2 Units: `via:`

```yaml
units:
  - {name: c1, block: copter, via: hq}
  - {name: c2, block: copter, via: edge1}
```

- `via:` names the comm node this unit's de_comm connects to. If omitted,
  it's the `gcs:` node, which reproduces today's behaviour.
- `via:` an auth-mode node → de_comm logs in through that node's `auth:`
  (`use_local_comm_server: false`, `auth_ip/auth_port` from the auth
  node).
- `via:` a local-mode node → `use_local_comm_server: true`,
  `local_comm_server_ip/port` = that node's wss port, or the
  connection's link_proxy port (section 6).

### 3.3 Comm servers attached to a unit

```yaml
units:
  - name: c3
    block: copter
    server:                   # this unit runs its own comm server (onboard)
      relay_to: hq            # uplink: child of hq
      storage: db1            # optional
  - {name: c4, block: copter, via: c3}   # c4 joins the mesh through c3's server
```

- The node is named after the unit (`c3`) and is a `comm` node with
  `owner: c3`. It shows up in the topology like any other node, so
  `via: c3` and `relay_to: c3` work.
- **Lifecycle follows the unit.** The node's process is launched under
  the unit's owner (`var/run/<sim>/c3/server/`), started **before** the
  unit's de_comm, and stopped and restarted with the unit
  (`kill`/`restart unit: c3 proc: all` takes it down too). A new process
  name, `de_server`, joins the unit's process list in `unit_procs()`,
  ahead of `link_proxy`/`de_comm`.
- c3's own de_comm connects to its own server in local mode (the onboard
  path, `127.0.0.1`).
- Taking c3 down takes c4 offline from the GCS's point of view, which is
  the realistic onboard-relay failure.

### 3.4 S2S keys and IDs, generated per run

- `server_id` = `<sim>_<node>` (e.g. `copters2_hq`). Unique across
  parallel sims, so logs and `_path` stay readable.
- `var/run/<sim>/server/keys/<server_id>_{private,public}.pem`: Ed25519,
  generated with `crypto.generateKeyPairSync('ed25519')` through a small
  node helper (or `openssl genpkey -algorithm ed25519`), mode 0600. Never
  the repo's `ssl_local/DE_ServerComm_*` pair, which leaks across sims
  and into git.
- Each parent's `s2s_trusted_server_keys` = `{<child server_id>:
  <child public.pem>}` for its direct children.
- Each storage node: `var/run/<sim>/server/<storage>/s2s_shared_secret.key`
  (32 random bytes hex, 0600). Every attached comm server's
  `s2s_shared_secret_file` points at the same file.
- `external` parent: not supported in phase 3. It would need the user to
  add our public key to their server; a later `trust_key_out:` option
  could print it.

## 4. Changes in `engine/simlib.py`

### 4.1 `load_sim` / `load_server`

- `load_sim` accepts **either** `server:` (today) **or** `servers:`,
  never both. `server:` is normalised into a one-node `servers:` (section
  7), so everything downstream only ever sees the topology form.
- New `load_servers(spec, slot_base, sim_name) -> topo`:
  `{nodes: {name: node}, gcs: name, edges: [...]}`. Each node carries
  type, `server_id`, role flags, attachments, owner (`None` or unit
  name), and its **port lease names** (not numbers).
- `load_server()` stays as the per-node validator for the keys it already
  understands (`auth`, `account`, `unit_account`, `seed`,
  `clock_offset_ms`, `noclock`, `control_port`, `gcs_party`,
  `link_proxy`), so error messages for single-server sims don't change.
- `sim["server"]` is kept as an **alias for the `gcs:` node**, with the
  same dict shape as today. The GCS bridge, `ctl.py`, `launcher.up`,
  checks and scenario runner keep reading it unchanged until each is
  moved over.

### 4.2 Ports

- Today: `port`, `port+1` (control), `port+10` (auth), `port+11` (auth
  S2S), where `port = SERVER_PORT + slot_base`.
- New: every port is a named lease.
  Per port_allocation.md (reconciled, section 10): the topology loader
  builds `Request(name, proto, kind, prefer=, pin=)` objects and passes
  them all to **one** `alloc.transaction(sim=...)`. The results land in
  the flat `ports` map of `var/run/<sim>/sim.json`, and code reads them
  with `simlib.port(sim, name)`. `alloc.release(sim)` runs on
  `sim down`. Lease names (port_allocation.md's naming):

  | Lease | Used by |
  |---|---|
  | `server.<node>.wss` | comm `server_port`; units/GCS dial it (a unit-owned server is `server.<unit>.wss`) |
  | `server.<node>.s2s_listen` | parent's `s2s_super_server_port` (only if it has children) |
  | `bridge.<gcs node>.ctrl` | the GCS bridge's control port (today `port+1`) |
  | `auth.<node>.https`, `auth.<node>.s2s` | authenticator `server_port`, `s2s_ws_listening_port` |
  | `storage.<node>.wss` | storage `server_port` |
  | `<node>.events` | the optional live stream of the sim event plugin (section 6.3) |
  | `link.<edge>.proxy.data`, `link.<edge>.proxy.ctrl` | a link_proxy on a named connection |

- For a **single-server sim** the allocator must hand out the **same
  numbers as today** (`62008+slot_base` etc.) during migration, so
  `ctl`, studio and tests that still read `srv["port"]` don't break.
  port_allocation.md's phase 0 ("legacy-pin mode") does exactly this:
  the old formulas are leased as `pin`, so the numbers can't move.

### 4.3 `gen_server(sim)` → per-node generation

`gen_server` becomes a loop over nodes in launch order, calling one
generator per type. Each writes `var/run/<sim>/server/<node>/` (or
`var/run/<sim>/<unit>/server/` for a unit-owned node):

- **`_gen_storage(node)`**: load `STORAGE_REPO/server.config`; set
  `server_id`, `server_ip: 127.0.0.1`, `server_port`,
  `database.path` = `<dir>/storage.db` (or under
  `var/persist/<sim>/<node>/` when `persist: true`), `log_directory`,
  `dashboard_enable: false`, `s2s_cert_enabled: false`,
  `s2s_shared_secret_file`, ssl paths from the storage repo's
  `ssl_local/ssl_airgap`. New `sim.local.env` key: `DE_STORAGE_DIR` →
  `$DE_CODE/droneengage_storage_server`, with `STORAGE_REPO` resolved
  like `SERVER_REPO`/`AUTH_REPO`; `check_binaries` adds its
  `src/server.js` when needed.
- **`_gen_auth(node)`**: today's authenticator block, moved as is, ports
  from leases.
- **`_gen_comm(node)`**: today's `scfg.update` block plus:
  - `server_id`, `s2s_my_private_key` = per-run key
  - `local_server_enabled = not node.auth`, `ignore_auth_server = not node.auth`
  - `local_server_account_id` = the mesh account ID (section 5.2)
  - if it has children: `enable_super_server: true`,
    `s2s_super_server_ip: 127.0.0.1`, `s2s_super_server_port`,
    `s2s_trusted_server_keys`
  - if `relay_to`: `enable_persistant_relay: true`,
    `s2s_relay_to_super_server_ip/port` = the parent's `s2s` port, or the
    proxy port of that connection
  - if `storage`: `enable_storage_server: true`,
    `storage_server_host/port` (or the proxy port of that connection),
    `s2s_shared_secret_file`
  - `command_plugin: []` (as today), plus a `plugins:` entry for the sim
    event plugin when events are enabled (section 6.3): `{path:
    <abs path to engine/server_plugins/de_sim_events>, enabled: true,
    config: {file: <node dir>/events.jsonl, port: <node>.events}}`
    (PluginEvents.md §2.1, §5.1)
  - **absolute** `ssl_key_file`/`ssl_cert_file`. The parent's S2S listener
    resolves relative SSL paths from `src/comm_server/`, not the repo root
    (PluginEvents.md §5.4/§8). Today's `gen_server` already writes
    absolute paths; keep it that way.
- **`_gen_fake(node)`**: today's fake block, unchanged.
- **`external`**: today's `_bridge_script` only.

The GCS bridge script is generated once, for the `gcs:` node, exactly as
today.

### 4.4 `gen_unit`

The de_comm target (around `simlib.py:1673-1692`) changes from the global
`srv` to `node = topo.nodes[u.via]`:

- auth mode: `auth_ip/auth_port` from `node.auth`'s node
- local mode: `local_comm_server_port` = `edge.<u>-><via>.port` if that
  connection is proxied, else `server.<via>.wss`

`u["link_proxy"]` (today a per-unit flag aimed at the one server) becomes
the proxy on the `<unit>-><via>` connection. A unit-owned server
(section 3.3) adds `de_server` to that unit's generated processes.

### 4.5 Launch order, readiness, shutdown

Launch order is a topological sort of the server graph:

1. `storage` nodes: ready = wss port open **and** the
   `storage`-readiness log line or event (the first `server.start` event
   from storage needs the plugin host there too, so phase 1 uses the port
   only)
2. `auth` nodes: ready = `server_port` open (today's check)
3. `comm` nodes, **parents before children** (root first), so a child's
   first dial doesn't burn a 10 s retry. Ready =
   - wss port open (today), and
   - parents: `s2s` port open
   - every comm node: `server.listening` event (port check until P6)
   - parents: `s2s.listen` event
   - children: S2S link up = the **parent's** `s2s.child.auth_ok` with
     `child_id` = the child's `server_id`. The child side only sees
     `s2s.uplink.open` / `s2s.uplink.challenge_answered`; a child-side
     `s2s.uplink.auth_ok` needs the parent → child "ok" notice that
     PluginEvents.md proposes as a small server addition (its open
     question 3). Until events exist: a 20 s grace, then a warning, not a
     failure.
   - with storage: `storage.auth.ok` on the comm node. Storage nodes
     themselves have no plugin host (PluginEvents.md phase 6, "later"),
     so the storage process's own readiness stays a port check.
   - with auth: `auth.link.open`.
4. link_proxies on server ↔ server connections: they start **before**
   the node that dials through them
5. the GCS bridge on the `gcs:` node (control port, as today)
6. units: unit-owned `de_server` first, then (as today) fcb_proxy,
   link_proxy, de_comm, modules

Shutdown is the reverse order: units, bridge, children, parents, auth,
storage. `sim down` releases the sim's port leases
(`alloc.release(sim)`), and the allocator's stale-lease sweep reclaims
leases left by a killed `up` (port_allocation.md §3).

`launcher.up`'s single-server checks (`wait_port(srv["port"])`, the
`busy` list) become a loop over nodes. Error messages name the node:
`server edge1 did not open 62131 - see var/run/<sim>/server/edge1/logs/`.

## 5. Auth in a mesh

### 5.1 The problem

- One authenticator decides which comm server serves an account, and
  sticks to the server already serving it (section 2). Units can't ask
  for `edge1`.
- `link_proxy` + `auth: true` is refused today for the same reason: the
  authenticator gives de_comm the server address, so the sim can't put a
  per-unit proxy in between.

### 5.2 Options

| Option | How | For | Against |
|---|---|---|---|
| **A. Root authenticates, children run in local mode** | Root `comm` has `auth:`. Children and unit-owned servers run `local_server_enabled` + `ignore_auth_server`. Units on children connect in local mode | Matches the shipped `server.s2s.drone.config`, and an onboard drone server really works this way. No server changes. link_proxy works on every child-side connection | Units on children skip login. Account scoping needs matching IDs (below) |
| B. One authenticator per child | Each child gets its own `auth` node | Units log in everywhere | Unrealistic. Account IDs differ per authenticator, so relayed scoping always falls back to "everyone". Doubles the processes |
| C. Login hint in the authenticator (`preferred server_id`) | Product change: `fn_selectServerforAccount` honours a hint from the login card | True per-unit placement under one authenticator | Needs a server/auth change and a de_comm config field. Mesh routing with one account across servers is still unclear |
| D. Comm server advertises a proxy as its `public_host:port` | The authenticator returns the proxy address | link_proxy with auth works | Only one proxy for **all** units on that server, not per-unit |

### 5.3 Recommendation

**Option A** for phases 3–4, with two rules:

1. **Matching account IDs.** Every local-mode node in a mesh gets
   `local_server_account_id` = the root's account ID, so relayed
   `_aid`/`_gid` scoping finds the account and doesn't fall back to
   "everyone".
   - Root in local mode: use a fixed sim value, `sim_acc`. The group is
     `'1'` everywhere already.
   - Root in auth mode: the ID is whatever the spawned `single`
     authenticator assigns. **Open question:** read it from the root's
     account list (`accountDetails` in the comm → auth server info) once
     the GCS bridge is logged in, or pin it through an authenticator
     config field if one exists. Until that's settled, phase 3 meshes
     with an auth root are allowed but print a warning that relayed
     scoping uses the fallback.
2. **Per-unit proxies stay local-mode only.** With option A, every child
   connection is local-mode and can be proxied. For the auth root, a
   `link_proxy` on `unit -> hq` stays refused (today's rule), and option
   D is offered as `servers.hq.advertise_proxy: true` for a single
   server-wide proxy.

Option C should be filed as a **follow-up for the server and
authenticator**. It's the only path to "a unit logs in and is placed on
child X", and real deployments may want it.

## 6. Faults and observability

### 6.1 Named connections

Every connection in the topology has a name:

| Connection | Name | Proxy sits between |
|---|---|---|
| unit → comm | `c2->edge1` | de_comm and `edge1.wss` |
| child → parent | `edge1->hq` | `edge1`'s relay dial and `hq.s2s` |
| comm → storage | `hq->db1` | comm's DBProxyClient and `db1.wss` |
| comm → auth | `hq->hq_auth` | comm's `s2s_ws_target_port` and auth's S2S port |

The proxy is enabled per connection:

```yaml
servers:
  edge1: {type: comm, relay_to: hq, proxy: [relay]}    # proxy the uplink
  hq:    {type: comm, storage: db1, proxy: [storage, auth]}
units:
  - {name: c2, via: edge1, link_proxy: true}           # existing per-unit flag
```

(`proxy:` lists which of the node's outgoing connections get a proxy.
The existing per-unit `link_proxy:` and `server.link_proxy:` keep their
meaning for unit → server connections.)

`link_proxy.py` relays TLS bytes, so it works on the S2S and storage
links unchanged. Its control protocol keys by **party**, and for
server ↔ server connections the party is the connection name. The
`list` command reports `d:<edge>` when the link is up.

### 6.2 Scenario syntax

Today's link steps take `unit:`. They gain an `edge:` alternative (one
or the other, never both):

```yaml
timeline:
  - {t: 30, fault: link_cut,  edge: "edge1->hq", dur_s: 40}     # partition the mesh
  - {t: 30, fault: link_stall, edge: "hq->db1", seconds: 20}    # storage brown-out
  - {t: 30, fault: link_cut,  unit: c2, dur_s: 15}              # unchanged form
```

- `unit: c2` ≡ `edge: "c2-><c2.via>"`
- The runner resolves `edge` to the proxy's control port from the
  manifest (`link.<edge>.proxy.ctrl`).
- `./sim ctl link <party> ...` gains `./sim ctl edge <edge> ...`, which
  sends the same sub-lines to that proxy.
- `loss*` stays fake-only, exactly as today.

### 6.3 Server events as assertions

Per PluginEvents.md §5: the `de_sim_events` plugin (in this repo, under
`engine/server_plugins/de_sim_events/`) subscribes to `*` and appends one
JSON line per event to `var/run/<sim>/server/<node>/events.jsonl`. It can
also stream them live on `<node>.events`, used by the
`wait_server_event` timeline step. Each line is the plugin envelope
`{name, ts, seq, server_id, data}`. `match:` below filters on `data`
fields. The runner loads the files into `ctx["server_events"]` keyed by
node name.

The scenario runner tails those files. New checks in `engine/checks.py`:

| check | args | passes when |
|---|---|---|
| `server_event` | `node`, `event` (glob), `match: {k: v}`, `at_least`/`equals`/`at_most` | count of matching events |
| `server_event_within` | `node`, `event`, `after` (a timeline step `id`, a mark, or another event), `within_s` | a matching event follows the reference within the window |
| `server_event_order` | `node`, `first`, `then` | event A precedes event B |
| `no_server_event` | `node`, `event`, `match` | none occurred (e.g. no `s2s.challenge.fail`) |
| `server_link_uptime` | `node`, `link` (`uplink`/`storage`/`auth`), `min_pct` | open/close pairs cover ≥ `min_pct` of the run (an addition to PluginEvents.md's list) |

Plus the timeline step `wait_server_event: {server, name, where,
timeout}` from PluginEvents.md §5.2.

Examples:

```yaml
checks:
  # the child re-authenticated after the partition healed
  - {check: server_event_within, node: hq, event: "s2s.child.auth_ok",
     match: {child_id: copters2_edge1}, after: cut_mesh, within_s: 20}
  # storage re-authenticated after the brown-out. The storage link backs
  # off 1 s -> 60 s with +/-25 % jitter, so the window must cover a full
  # backoff step
  - {check: server_event_within, node: hq, event: "storage.auth.ok",
     after: storage_stall, within_s: 75}
  - {check: no_server_event, node: hq, event: "s2s.child.auth_fail"}
```

Event names and fields here are PluginEvents.md's final catalog (§1).

Without the event plugin (phase 2 and before), the runner falls back to
link_proxy `stats`/`list` and the GCS bridge's view, and server-event
checks are SKIPped with a reason, never failed.

## 7. Backward compatibility

`server:` is normalised into a one-node topology:

| Today | Becomes |
|---|---|
| `server: {type: real, auth: false, ...}` | `servers: {main: {type: comm, gcs: true, ...}}` |
| `server: {type: real, auth: true}` | `servers: {main: {type: comm, auth: main_auth, gcs: true}, main_auth: {type: auth}}` |
| `server: {type: fake, ...}` | `servers: {main: {type: fake, gcs: true, ...}}` |
| `server: {type: external, ...}` | `servers: {main: {type: external, gcs: true, ...}}` |
| per-unit `link_proxy:` | proxy on `<unit>->main` |
| scenario `transport: fake/real/{type: real, link_proxy: true}` | unchanged (the scenario loader writes `server:` as today) |

Guarantees, each covered by a regression test:

- Generated configs for every sim in `sims/` are **byte-identical**
  before and after phases 1–3, except the deliberate storage change. Only
  the node directory moves: `var/run/<sim>/server/` stays as the
  single-node location, so logs and pid paths don't move either.
- `srv["port"]`, `srv["control_port"]` and `srv["auth_port"]` have the
  same values for single-server sims (pinned leases, section 4.2).
- `./sim ctl`, scenario `link_*` steps with `unit:`, studio and `sim
  status` work without edits.
- `enable_storage_server` stays `False` unless the sim adds a storage
  node. Adding storage is opt-in.

## 8. Example sims and scenarios

### 8.1 Save a mission, restart, fetch it (`sims/storage_single.yaml`)

```yaml
name: storage_single
servers:
  main: {type: comm, storage: db1, gcs: true}
  db1:  {type: storage, persist: true}
units:
  - {name: c1, block: copter}
```

Scenario `tests/storage/mission_roundtrip.yaml`:

```yaml
timeline:
  - {t: 5,  do: mission_save, name: sq1, file: ../../plans/square.json}  # 9011 via the GCS bridge
  - {t: 10, do: restart, unit: all, proc: server}                        # comm + storage restart; DB persists
  - {t: 25, do: wait_online, unit: c1}
  - {t: 30, do: mission_load, name: sq1, expect_items: 6}                # 9010 round trip
checks:
  - {check: server_event, node: main, event: "storage.request",
     match: {op: SaveMission, result: ok}, at_least: 1}
```

This needs a new bridge verb (`mission_save` / `mission_load`), sending
9011/9010 and waiting for the reply. That's a small `gcs_bridge.py` and
`ctl` addition. `restart proc: server` must restart storage nodes too
when `persist: true`.

### 8.2 Parent + child, GCS on the parent commands a unit on the child (`sims/mesh_two_tier.yaml`)

```yaml
name: mesh_two_tier
servers:
  hq:    {type: comm, gcs: true, storage: db1}
  edge1: {type: comm, relay_to: hq, storage: db1, proxy: [relay]}
  db1:   {type: storage}
units:
  - {name: c1, block: copter, via: hq}
  - {name: c2, block: copter, via: edge1}
```

Scenario: `wait_online c2` (seen by the GCS on `hq`), then `arm`,
`takeoff`, `mode GUIDED` on c2. All of these are one-to-one messages one
hop away, so they're supported. Checks: `fc_mode c2 GUIDED`,
`telemetry_value c2 alt ≥ 8`, `no_server_event hq s2s.child.auth_fail`.

### 8.3 A unit-attached server relays for a second unit (`sims/mesh_onboard.yaml`)

```yaml
name: mesh_onboard
servers:
  hq: {type: comm, gcs: true}
units:
  - name: c3
    block: copter
    server: {relay_to: hq}       # c3's onboard comm server, child of hq
  - {name: c4, block: copter, via: c3}
```

c4 is one hop below `hq` (`c4 -> c3 -> hq`), so one-to-one commands
work. Scenario: `wait_online c4`, `arm c4`, then
`kill unit: c3 proc: all` → `link_offline c4 at_least 1`, then
`restart c3` → c4 online again within 30 s.

### 8.4 Partition the mesh link and heal it (`tests/mesh/partition_heal.yaml`)

On `mesh_two_tier`:

```yaml
timeline:
  - {t: 10, do: wait_online, unit: c2}
  - {t: 20, id: cut_mesh, fault: link_cut, edge: "edge1->hq", dur_s: 40}
  - {t: 70, do: wait_online, unit: c2, timeout: 30}
checks:
  - {check: link_offline, unit: c2, at_least: 1}             # GCS saw c2 drop
  - {check: server_event_within, node: hq, event: "s2s.child.auth_ok",
     match: {child_id: mesh_two_tier_edge1}, after: cut_mesh, within_s: 20}
  - {check: server_event, node: edge1, event: "s2s.uplink.dropped",
     at_least: 1}                                            # traffic lost while cut
  - {check: recovers_after_fault, within_s: 30}
  # c2 stayed connected to edge1 the whole time: it lost the GCS, not its server
  - {check: no_server_event, node: edge1, event: "client.close",
     match: {unit: c2}}                       # runner maps unit -> party_id
```

`within_s: 20` allows for the child's fixed 10 s reconnect.

## 9. Phased plan

| Phase | Delivers | Depends on | Acceptance criteria |
|---|---|---|---|
| **P1 Storage node, single server** | `servers:` not needed yet. Adds `server.storage: true` / `persist:` on today's `server: {type: real}`; `_gen_storage`, `DE_STORAGE_DIR`, `enable_storage_server: true`, a per-run shared secret; bridge verbs `mission_save` / `mission_load` | none (fixed port `port+12` for storage until P2, leased as a `pin` once port_allocation.md phase 0 lands) | 8.1 passes on `--seeds 1,2`; `sims/*` without `storage` generate byte-identical configs; two sims with storage run in parallel |
| **P2 Port leases** | `gen_server`/`gen_unit`/`launcher.up` read ports from the allocator; single-server numbers unchanged | **Task 2**: port_allocation.md phase 0 (allocator, registry, legacy-pin mode) and phase 1 (free allocation for new/side ports) | every existing suite passes unchanged; `./sim up` of two sims (one with storage) gets disjoint ports; `sim down` releases the leases |
| **P3 `servers:` topology + `via:`** | `load_servers`, per-node generators, per-run S2S keys, parent/child configs, launch order, `server:` normalisation, two-hop warning. The loader emits the `Request`s of port_allocation.md §5 ("How a `servers:` topology requests ports"); this is that doc's **phase 4** | P2 | 8.2 passes; the parent's log shows the child passing the S2S handshake; the old `server:` sims are byte-identical; validation errors for cycles, unknown nodes and auth-on-child |
| **P4 Unit-attached servers** | `units[].server`, process under the unit owner, lifecycle with the unit | P3 | 8.3 passes, including kill/restart of c3 |
| **P5 Proxies on server ↔ server connections + `edge:` syntax** | `proxy:` on nodes, `link.<edge>.proxy.*` leases, `ctl edge`, runner `edge:` resolution | P3 (and P2 for the leases) | 8.4 passes **using GCS-side checks only**; a storage stall makes 9011 time out and then succeed after recovery |
| **P6 Server-event assertions** | `de_sim_events` plugin wired into `_gen_comm`, `ctx["server_events"]`, the checks and `wait_server_event`. This **is** PluginEvents.md's phase 4 ("part of task 3"); build it once, here | **Task 1**: PluginEvents.md phases 1–3 (host, client/auth events, storage/S2S/relay events) in the comm server; and P3 | 8.4's `server_event_within` checks pass; with the plugin disabled they SKIP with a reason; readiness in 4.5 switches from grace period to events |
| P7 (optional) Auth-root account ID | resolves the open question in 5.3 | P3 | 8.2 with `hq.auth` delivers relayed messages without the scoping fallback (verified by a `server_event`/journal check, or a second account that must **not** receive them) |

P1 is independent and gives you mission storage right away. P2 and P3 are
the core. P6 is where task 1 pays off.

Combined dependency order across the three docs:

```
port_allocation P0 ─ P1 ─────────────┐
                                     ├─ mesh P2 ─ P3 (= port_allocation P4) ─ P4 ─ P5
mesh P1 (storage, independent) ──────┘                 │
PluginEvents P1 ─ P2 ─ P3 ─────────────────────────────┴─ mesh P6 (= PluginEvents P4)
```

## 10. Assumptions and reconciliation with the companion docs

This doc was first written before either companion doc existed. Both
were read afterwards (port_allocation.md and PluginEvents.md as of
2026-10-21), and this doc was changed to match them. Every assumption is
now resolved:

| # | Assumption (as first written) | Doc | Resolution |
|---|---|---|---|
| A1 | `alloc(sim, owner, name)` → int, plus a per-sim manifest | port_allocation.md | **Changed to match.** The API is `alloc.transaction(sim=…)` with `tx.alloc(name, proto, prefer/pin)`, then `lookup`/`release`/`mark_bound`. The manifest is the flat `ports` map in `var/run/<sim>/sim.json`, read through `simlib.port(sim, name)`. There's no separate `owner`; the name carries it |
| A2 | Pinned/preferred ports keep single-server numbers | port_allocation.md | **Matches.** Phase 0 leases the legacy formulas as `pin` |
| A3 | Release on `sim down`; dead-pid leases reclaimed | port_allocation.md | **Matches.** Release is by `(worktree, sim)`; the stale-lease sweep checks pid + start time |
| A4 | Unit-attached servers lease under the unit | port_allocation.md | **Matches by naming.** `server.c3.wss` is in that doc's own example; no owner concept needed |
| — | Lease names (`server.<n>.s2s`, `.ctl`, `server.<auth>.*`, `edge.<e>.*`) | port_allocation.md | **Renamed** to `server.<n>.s2s_listen`, `bridge.<n>.ctrl`, `auth.<n>.https/.s2s`, `storage.<n>.wss`, `link.<e>.proxy.data/.ctrl` |
| A5 | Event names | PluginEvents.md | **Renamed:** `s2s.parent.link.open` → `s2s.uplink.open`; `s2s.child.connected` → `s2s.child.auth_ok` (field `child_id`); `s2s.challenge.fail` → `s2s.child.auth_fail`; `storage.s2s_auth.ok` → `storage.auth.ok`; `storage.request {op, ok}` → `{op, result: ok}`; `client.close {party}` → `{party_id}`. Readiness uses `server.listening` and `s2s.listen` |
| A6 | Sim plugin through `command_plugin`, lines `{ts, node, event}` | PluginEvents.md | **Changed:** loaded through the new `plugins:` config block (not `command_plugin`). Envelope `{name, ts, seq, server_id, data}`. The plugin lives in this repo (`engine/server_plugins/de_sim_events/`) |
| A7 | Storage-server events may not exist | PluginEvents.md | **Confirmed absent** (their phase 6, "later"). Storage health is observed from the comm side (`storage.*`); the storage process's readiness is a port check |

**Mismatches left between the docs (to settle when implementing):**

1. **Child-side "authenticated" signal.** The parent emits
   `s2s.child.auth_ok`, but the child can't tell it was accepted unless
   the server adds a parent → child `{s2s_auth: 'ok'}` notice
   (PluginEvents.md open question 3). This doc only relies on the
   parent-side event, so the mesh can be built without that addition.
2. **Check names.** PluginEvents.md §5.2 lists `server_event`,
   `server_event_order`, `server_event_within`, `no_server_event`. This
   doc uses the same names and adds `server_link_uptime`. Agree on that
   one addition, or drop it.
3. **Phase ownership.** PluginEvents.md phase 4 (sim plugin + graders) and
   port_allocation.md phase 4 (topology requests) both say "with task 3".
   They are this doc's P6 and P3. Track each as **one** task, not two.
4. **Party IDs in `match:`.** Events carry `party_id` (the de_comm
   senderID), not the sim unit name. The runner should accept
   `match: {unit: c2}` and translate it to that unit's `party_id`, so
   scenarios don't hard-code IDs.
5. **Storage backoff.** PluginEvents.md §5.4 found the storage link backs
   off 1 s → 60 s with ±25 % jitter. Any `within_s` after a storage fault
   must be ≥ ~75 s, or the fault kept short (under ~10 s).

## 11. Open questions

1. **Auth-root account ID** for `local_server_account_id` on children
   (5.3): read at runtime, or pinned by authenticator config?
2. **Two-hop one-to-one relay** (2.1): accept it as a documented
   limitation, or file a server change so individual messages relay along
   the tree toward a known party?
3. **Option C** (authenticator login hint): is per-unit placement under
   one authenticator something real deployments need? If so, it's a
   product feature, not just sim plumbing.
4. Should a unit-owned server **always** get a storage link when the
   parent has one (to mirror an onboard cache), or only when declared?
5. `persist: true` storage: one `var/persist/<sim>/` per sim name, or per
   scenario run, and who cleans it?
