# storage_single — real storage server in the sim (SL-0079)

One copter on the real `droneengage_server` with a real
`droneengage_storage_server` (`server: {type: real, storage: true}`).

## Setup

- `DE_STORAGE_DIR` must resolve the storage repo
  (`$DE_CODE/servers/droneengage_storage_server` by default; override in
  `sim.local.env`). Run `npm install` in that repo once.
- The sim writes a per-run shared secret
  (`var/run/<sim>/server/storage_s2s_secret.key`, chmod 600) that both the
  comm server and the storage node read; the DB lives at
  `var/run/<sim>/server/storage.db` unless `persist: true` keeps it
  under `var/persist/<sim>/storage/` across `sim down`/`up`.

## Run

    ./sim up sims/storage_single.yaml
    ./sim ctl mission_save copter1 my_mission my_id '{"waypoints":[]}'
    ./sim ctl mission_load copter1 my_id
    ./sim ctl restart server de_server     # mission survives a comm restart
    ./sim ctl mission_load copter1 my_id
    ./sim down

## Ports

`storage.wss` is an ordinary allocator lease (`./sim ports` shows it);
the comm server dials it via `storage_server_host/port` in the generated
`de_server.config`.
