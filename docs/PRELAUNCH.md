# PRELAUNCH.md - the pre-launch gate

> DroneEngageStudio doc. Epic SL-0189.

Use this procedure before each release of the webclient, de_comm,
de_mavlink, Andruav or the servers. The purpose: do not break a critical
feature that works today.

## Why the sim is not enough

The sim runs all processes on one host. So it cannot find a firewall
fault, a wrong `public_host` or a bad ICE list. These faults stop the UDP
proxy and video in the field while every sim test stays green. The gate
adds tests on real hosts, across a real network, with real media.

## The hosts

| Host | Use |
|---|---|
| Unit Pi | Compiles the release, runs the test unit, runs the probes |
| Airgap Pi | Runs a true airgapped server from the real install scripts |
| Production machine A | de_auth and the webclient: live and mirror slot |
| Production machine B | de_comm server, storage and relay: live and mirror slot |
| Outside host | A host outside the LAN, for the view of a real user |

## The slots

Production has two slots on the same two machines: live and mirror. Each
slot keeps its own ports for all time: comm, storage, relay range and S2S.
Only the public de_auth API port (`server_port`) moves between the slots.
Units know only `auth_ip` and `auth_port`. All other values come from
de_auth. So tests on the mirror use the same firewall rules that the
mirror uses after the switch.

There is one coturn. It is not in a slot. A switch does not change it. A
coturn config change affects live at once. Test it on the airgap Pi
first. Then apply it as its own step.

## The stages

Run `./sim prelaunch --list` to see each feature and its checks.

| Stage | What it proves | Where it runs | Time |
|---|---|---|---|
| 0 | The config has no ICE, `public_host` or port fault | Any host | Seconds |
| 1 | Unit tests and the critical sim scenarios pass | Unit Pi | 45 min or less |
| install | The real artifacts install and upgrade, and all services start | Both Pis | Minutes |
| 2 | Ports, TLS, login and TURN work from another host | Unit Pi to airgap Pi | Minutes |
| 3 | MAVLink and video frames really arrive | Unit Pi to airgap Pi | Minutes |
| 4 | A human confirms what automation cannot judge | The human | 15 min or less |

## The procedure

1. Freeze the versions. Build the release with the de-release skill.
2. Run `./sim prelaunch --target airgap`, all stages. Fix each FAIL.
3. Install the release on the production mirror slot.
4. Run `./sim prelaunch --target mirror`, stages 0, 2 and 3, from the
   outside host. The test unit changes only `auth_port`.
5. Do the human checklist below. Record each item with `--signoff`.
6. Read the report. Apply the go or no-go rule.
7. Send the global message to users.
8. Switch the slots (SL-0205).
9. Run `./sim prelaunch --target live --canary` at once.
10. Keep the old slot unchanged until you approve the release.
11. Update the old slot. It is now the new mirror.

## The go or no-go rule

**Any P0 FAIL or SKIP means no-go.** A skipped check proves nothing. So it
counts the same as a failed check. A P1 FAIL does not block. The report
still shows it.

The human can override a no-go. The override and its reason go in the
report. A report for a different version does not count.

## Rollback rule

Roll back if the canary shows a P0 FAIL after the switch, or if the human
sees a P0 fault during the soak. A rollback is one more port swap. The
old slot stays unchanged until approval. Do not debug on live. Roll back
first. Then debug on the mirror.

## Human checklist (15 minutes or less)

Do these after stages 0 to 3 pass. Use a network that is not the server
LAN when you can, for example a phone hotspot.

| Check id | Time | What to do |
|---|---|---|
| `human.qgc` | 5 min | Connect a real QGroundControl to the UDP proxy port of a unit. See telemetry and change a mode. |
| `human.andruav_video` | 5 min | Open the video of a real Andruav phone in the webclient, from a phone hotspot network. |
| `human.fly_webclient` | 5 min | On SITL or on a bench unit, arm, take off and land from the webclient. Read the map, HUD and video. |

Record each result:

    ./sim prelaunch --target <t> --signoff human.qgc=pass

## Rules that keep the gate useful

1. **Each probe has a negative control.** Break the thing on purpose.
   The probe must go red. Do this on the airgap Pi only. Never add a
   firewall rule or restart a service on production for a test.
2. **Each incident adds a row.** A field fault that the gate did not
   catch adds a row or a check to `prelaunch/features.yaml`. Put it in
   the same change as the fix.
3. **No flaky test in the gate.** A flaky check teaches people to ignore
   the gate. Fix it or remove it.
4. **Test what ships.** Use the release artifacts, not dev builds.
5. **Credentials stay out of reports.**
6. **Production probes do not change state.** They never arm a vehicle
   and never write config.

## The checks on live

Run `./sim prelaunch --target live --canary` after each deploy and on a
timer, from a host outside the server network:

    */15 * * * * cd ~/de_code/droneengage_studio && ./sim prelaunch --target live --canary

A firewall or a certificate can change with no release. Only `--canary`
runs on a live target. The runner refuses other stages there. It uses
only the canary account. On the machines it runs only `ss -ltn` and `cat`
of named config files. A change from PASS to FAIL makes one notice,
through `canary.notify_cmd` in the target. A repeated FAIL makes no new
notice. A certificate with less than 14 days left makes a warning.

## The checks on the mirror

Set every null value in `prelaunch/targets/mirror.yaml` first. The mirror
adds three checks. The slots share no port or relay range. The mirror
auth sends clients to the mirror comm and not to live. The test unit
differs from production only in `auth_port`. The mirror shares CPU and
bandwidth with live. Keep video tests short. Run no load test. Never add
a firewall rule on these machines for a negative control. Run those on
the airgap Pi.

## The switch

`engine/prelaunch_promote.py` moves the public de_auth port between the
slots. It reads a plan file. The header of the script shows the format.
Write that file on your own machine. It names your paths and commands.

    python3 engine/prelaunch_promote.py --plan <plan> --version <v>

This is a dry run. It prints each step and each config change. It changes
nothing. Then send the global message to users. Then run:

    python3 engine/prelaunch_promote.py --plan <plan> --version <v> --message-sent --execute

To roll back:

    python3 engine/prelaunch_promote.py --plan <plan> --rollback --execute

The script stops when no GO report or override exists for the version.
Armed units do not stop it. It shows the count. The human warns users
first. During the swap, logins fail for a short time. The port has one
owner. Units on the old comm drop when it restarts. They reconnect to the
new slot.

Each machine holds a live-slot file. Deploy skills read it. They deploy
only to the other slot. A deploy to a fixed path can overwrite live.
