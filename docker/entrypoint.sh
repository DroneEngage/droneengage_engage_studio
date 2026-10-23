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
# Port-lease registry: unless the host's de_sim dir is mounted at
# /run/de_sim the registry stays container-private - warn once, then
# carry on (the bind probe still catches real clashes).
if command -v mountpoint >/dev/null 2>&1 && ! mountpoint -q /run/de_sim; then
    case "${SIM_PORT_REGISTRY:-/run/de_sim/ports.json}" in
        /run/de_sim|/run/de_sim/*)
            echo "port registry not shared with the host (see docker/README.md)" ;;
    esac
fi
if [ "$#" -eq 0 ]; then
    set -- view docker/canopy_quickstart.yaml
elif [ "$1" = "bash" ] || [ "$1" = "sh" ]; then
    exec /bin/bash
fi
exec ./sim "$@"
