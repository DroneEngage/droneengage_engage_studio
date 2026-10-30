"""simland_cli.py - console entry point (`sim-land`) of the installed
package. It runs the same `sim` front-end as ./sim: the checkout (or the
SIM_ROOT directory) holds the `sim` script, blocks/, modules/, sims/ and
the other data the engine reads relative to its root."""
import os
import runpy
import sys


def _find_sim():
    roots = [os.environ.get("SIM_ROOT"),
             os.path.dirname(os.path.dirname(os.path.abspath(__file__)))]
    for r in roots:
        if r and os.path.isfile(os.path.join(r, "sim")):
            return os.path.join(r, "sim")
    return None


def main():
    path = _find_sim()
    if not path:
        sys.exit("sim-land: no `sim` front-end found - run from an "
                 "editable install of the checkout (pip install -e .) "
                 "or set SIM_ROOT to the droneengage_studio directory")
    sys.argv[0] = path
    runpy.run_path(path, run_name="__main__")


if __name__ == "__main__":
    main()
