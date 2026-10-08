#!/bin/bash
# de_sound launch wrapper (TASK-EV-SND): the engine launches
# '<bin>' -c '<cfg>' directly, and de_sound_module.py ships no shebang -
# exec python3 on the entry point instead.
# The module repo is $DE_SOUND_DIR when set, else the sibling checkout of
# this simulation_land repo (the usual all-repos-under-DE_CODE layout).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DE_SOUND_DIR="${DE_SOUND_DIR:-$(cd "$HERE/../../.." && pwd)/droneengage_sound_module}"
# -u: the sim reads the ready line ("Communicator Server Found") off the
# module's stdout log; buffered python output would never flush in time
exec python3 -u "$DE_SOUND_DIR/de_sound_module.py" "$@"
