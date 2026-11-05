"""prelaunch_stubs.py - stub probes for the runner's own tests."""


def ok(ctx):
    return "PASS", "stub pass"


def bad(ctx):
    return "FAIL", "stub fail"


def skip(ctx):
    return "SKIP", "stub skip"


def crash(ctx):
    raise RuntimeError("stub crash")


def leak(ctx):
    ctx.secrets.add("hunter2")
    return "FAIL", "login failed for code hunter2"
