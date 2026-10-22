"""adapter.py - the vehicle-stack seam (docs/ADAPTER.md is the spec).

de.scenario/1's core (timeline, world, truth, checks) needs a narrow
surface from a vehicle stack:

  do_step(name, args, step, scn, ctx) -> bool   every `do:` verb

The stock implementation (de_adapter.DEAdapter, the `droneengage`
plugin of the `simland.adapters` entry-point group) is DroneEngage: op packages
(.deploy.json / .deop / compiled .json), the operator bot's GCS session
(deploy / prepare / start / abort / authorize / revoke_launch /
operator approve-deny-confirm-fire), and the ctl verb set behind
runner.run_step (mode / arm / land / wait_journal / fire / param /
link_* / fc ...). The fleet bus (1090) rides inside those ctl verbs -
it is not a separate seam.

A non-DE adapter implements the same do_step signature; everything the
core hands it (the ctx dict keys, world hooks, journal conventions) is
documented in docs/ADAPTER.md.
"""
import os


class VehicleAdapter:
    """Interface - what the core calls. docs/ADAPTER.md has the
    field-level contract (required inputs, hooks, optional faults)."""

    name = "abstract"

    def do_step(self, name, a, step, scn, ctx):
        """One `do:` step -> truthy on success. `a` is the step's args
        (t/on/do/fault/once already stripped); `step` is the raw step
        (an `on:` trigger's unit defaults live there)."""
        raise NotImplementedError


ENTRY_POINT_GROUP = "simland.adapters"
DEFAULT_ADAPTER = "droneengage"


def _load_adapter_class(name):
    """Adapter class registered as `name`: an installed plugin
    (entry-point group simland.adapters) first; the in-tree
    de_adapter for the stock one when the package is not installed
    (the ./sim checkout)."""
    try:
        from importlib.metadata import entry_points
        for ep in entry_points(group=ENTRY_POINT_GROUP):
            if ep.name == name:
                return ep.load()
    except Exception:
        pass
    if name == DEFAULT_ADAPTER:
        from de_adapter import DEAdapter
        return DEAdapter
    raise RuntimeError(f"unknown vehicle adapter {name!r} (no "
                       f"{ENTRY_POINT_GROUP} entry point)")


def new_adapter(ctx):
    """The stack the scenario runs on - DroneEngage by default;
    SIM_ADAPTER=<name> selects another registered plugin
    (docs/ADAPTER.md)."""
    return _load_adapter_class(
        os.environ.get("SIM_ADAPTER") or DEFAULT_ADAPTER)()


def __getattr__(attr):
    # back-compat: adapter.DEAdapter still resolves (lazy - de_adapter
    # imports this module)
    if attr == "DEAdapter":
        from de_adapter import DEAdapter
        return DEAdapter
    raise AttributeError(attr)
