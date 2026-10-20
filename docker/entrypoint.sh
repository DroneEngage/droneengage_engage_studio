#!/bin/bash
# Container entrypoint: everything resolves inside the image
# (sim.local.env in the repo dir), so this is just a cd + passthrough
# into the sim CLI.
#
#   docker run -it --network host de-sim                 # the demo run
#   docker run -it --network host de-sim doctor          # env self-check
#   docker run -it --network host de-sim list            # scenario catalog
#   docker run -it --network host de-sim bash            # a shell
#
# --network host matters: the sim binds per-slot 127.0.0.1 ports (the
# printed viewer URL, SITL TCP links, the fake server) and expects them
# reachable from the user's browser.
cd "$(dirname "$0")/.." || exit 1
if [ "$#" -eq 0 ]; then
    set -- view scenarios/demos/world_canopy_viewer.yaml
elif [ "$1" = "bash" ] || [ "$1" = "sh" ]; then
    exec /bin/bash
fi
exec ./sim "$@"
