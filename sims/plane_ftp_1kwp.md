# plane_ftp_1kwp — how to run

Big-mission FTP read demo: ONE ArduPlane SITL + de_comm + de_mavlink on a
spawned real server, with a 1001-waypoint mission on the FC. Lets you
watch the MAVLink FTP mission read (`@MISSION/mission.dat`, behind the
`use_mavlink_ftp_mission` config switch) and its R-WP progress frames in
the WebClient.

## Setup (once)

- `sim.local.env` filled in (DE_MAVLINK_DIR, DE_COMM_DIR, DE_SERVER_DIR,
  ARDUPILOT_DIR).
- `droneengage_mavlink/bin/de_ardupilot` and
  `droneengage_communication_pro/bin/de_comm` built (`./build.sh` in each
  repo). Rebuilding while the sim is up only needs
  `./sim ctl -s plane_ftp_1kwp restart wptest de_mavlink` — the SITL keeps
  the mission.
- `packages/plan_survey_1001wp.json` exists. Regenerate with:

      python3 tools/gen_bigplan.py --count 1003 \
          --out packages/plan_survey_1001wp.json

  (`--count` counts takeoff + RTL too, so 1003 = 1001 waypoints.)

## Run

```bash
cd droneengage_studio

# 1. WebClient dev server (separate terminal, leave running)
cd ../droneengage_webclient_react
NODE_OPTIONS=--openssl-legacy-provider HTTPS=true \
  SSL_CRT_FILE=$PWD/ssl/localssl.crt SSL_KEY_FILE=$PWD/ssl/localssl.key \
  ./node_modules/.bin/react-scripts start
cd -

# 2. Bring the sim up (SITL + de_comm + de_mavlink + real server)
./sim up sims/plane_ftp_1kwp.yaml

# 3. Upload the 1001-wp mission to the FC (~2-3 min write)
./sim ctl -s plane_ftp_1kwp deploy packages/plan_survey_1001wp.json
#    prints "package NOT READY (timeout)" after 30 s — expected: the
#    SV_LOAD_TIMEOUT_MS cap documented in tests/capacity/plan_1kwp.yaml.
#    The FC write keeps going; watch it finish:
./sim ctl -s plane_ftp_1kwp wait_journal wptest "package READY|nav load failed" 400
#    FC-side confirmation in the de_mavlink log: "Flight plan received"

# 4. GCS deep link
./sim status -s plane_ftp_1kwp --open-web
```

Open the printed `webclient:` link, then on the mission panel click
**R-WP**. The button shows `R-WP N/1004` busy progress while the FTP
download streams, then the mission draws on the map. (1004 = the 1003
items + ArduPilot's home placeholder.)

## Verify the FTP path

```bash
./sim logs wptest de_mavlink | grep FTP
# -> "Mission Received Way points count: 1004 (FTP)"
```

Legacy fallback check: set `"use_mavlink_ftp_mission": false` in
`var/run/plane_ftp_1kwp/wptest/de_mavlink/de_mavlink.config.json`,
restart `de_mavlink`, R-WP again — same completion line without `(FTP)`
(the per-item `MISSION_REQUEST_INT` loop, visibly slower).

## Teardown

```bash
./sim down plane_ftp_1kwp
```

## Ports

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s plane_ftp_1kwp` lists what was actually bound. `slot_base: 95`
is identity only (plane sysid 95, distinct from the default slot-40
fleets).
