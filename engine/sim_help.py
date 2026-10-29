"""sim_help.py - per-sim `--help` (SL-0121).

`render_help(path, verb)` prints how to run one sim/test without opening
any file: description, runbook, usage, declared `params:`, the shared
flags and an example. Every flag description comes from
engine/sim_commands.py (the central registry) - no flag text is
duplicated here.
"""

import os
import textwrap

import sim_commands
import simlib

# verbs whose positional argument is a sim/test and that accept the
# per-sim help trigger
SIM_HELP_VERBS = ("show", "up", "run", "try", "view", "studio")

_HELP_TOKENS = ("help", "--help", "-h")


def _first_line(text):
    for ln in (text or "").splitlines():
        if ln.strip():
            return ln.strip()
    return ""


def _rest_wrapped(text, width=78):
    """Description minus its first line, each paragraph wrapped."""
    lines = text.splitlines()[1:]
    out = []
    for ln in lines:
        if not ln.strip():
            out.append("")
        else:
            out.extend(textwrap.fill(ln.strip(), width).splitlines())
    # trim leading/trailing blanks
    while out and not out[0]:
        out.pop(0)
    while out and not out[-1]:
        out.pop()
    return "\n".join(out)


def _param_lines(decl):
    """One param per two lines:
      name           default: X   choices: a, b   (int)
                     <help>
    """
    out = []
    name_w = max((len(n) for n in decl), default=0) + 4
    for name in decl:
        p = decl[name] or {}
        head = "  " + name.ljust(name_w) + "default: " + str(p.get("default"))
        if p.get("choices"):
            head += "   choices: " + ", ".join(str(c) for c in p["choices"])
        ptype = p.get("type", "str")
        if ptype != "str":
            head += f"   ({ptype})"
        out.append(head)
        out.append("  " + " " * name_w + "  " + (p.get("help") or "").strip())
    return out


def _flag_lines(verb):
    """Verb's own flags then the shared ones, one line each - the text is
    the registry's, never reworded here."""
    cmd = sim_commands.COMMANDS.get(verb) or {}
    out = []
    for flag, help_text in (cmd.get("flags") or {}).items():
        out.append(f"  {flag:<14}  {help_text}")
    for flag, help_text in sim_commands.SHARED_FLAGS.items():
        out.append(f"  {flag:<14}  {help_text}")
    return out


def _usage_line(verb, name, has_params):
    cmd = sim_commands.COMMANDS.get(verb) or {}
    usage = (cmd.get("usage") or [f"./sim {verb} <sim>"])[0]
    # swap the registry's placeholder for this sim's name and add the
    # --set slot when the sim declares params
    line = (usage.replace("<sim.yaml|de_servers|de_webclient>", name)
                 .replace("<sim.yaml>", name)
                 .replace("<file.yaml>", name)
                 .replace("<file>", name)
                 .replace("<sim>", name))
    if has_params:
        parts = line.split("[", 1)
        line = parts[0].rstrip() + " [--set NAME=VALUE ...]" + \
            (" [" + parts[1] if len(parts) > 1 else "")
    return line


def _example_lines(spec, verb, name, params):
    ex = spec.get("example")
    if isinstance(ex, str):
        ex = [ex]
    if ex:
        return list(ex)
    # generate one: first param with choices wins, else the first param
    pick = None
    for n, p in params.items():
        if (p or {}).get("choices"):
            pick = (n, p["choices"][0])
            break
    if pick is None and params:
        n = next(iter(params))
        pick = (n, (params[n] or {}).get("default"))
    if pick is not None:
        return [f"./sim {verb} {name} --set {pick[0]}={pick[1]}"]
    return [f"./sim {verb} {name}"]


def render_help(path, verb="up"):
    """Human help for one sim/test file. Plain text, no colours - the
    caller prints it verbatim."""
    spec, _src = simlib._sim_spec(path)
    name = spec.get("name") or os.path.splitext(os.path.basename(path))[0]
    desc = (spec.get("description") or "").strip()
    params = spec.get("params") or {}

    out = [f"{name} - {_first_line(desc) or '(no description)'}"]
    rest = _rest_wrapped(desc)
    if rest:
        out += ["", rest]

    runbook = os.path.join(os.path.dirname(path), name + ".md")
    if os.path.isfile(runbook):
        rel = os.path.relpath(runbook, simlib.SIM_ROOT)
        out += ["", f"Runbook: {rel}"]

    out += ["", "Usage:", "  " + _usage_line(verb, name, bool(params))]

    out += ["", "Parameters:"]
    if params:
        out += _param_lines(params)
    else:
        out.append("  none")

    out += ["", "Common flags:"] + _flag_lines(verb)

    out += ["", "Example:"]
    out += ["  " + ex for ex in _example_lines(spec, verb, name, params)]

    return "\n".join(out)


def wants_sim_help(verb, args):
    """True when `args` (post _cli_flags) holds a help token AND a
    positional sim/test argument - `./sim up copters3 --help` - vs the
    bare `./sim up --help` command-help case (no positional)."""
    if verb not in SIM_HELP_VERBS:
        return False
    pos = [x for x in args if not x.startswith("-") and x not in _HELP_TOKENS]
    return bool(pos) and any(x in _HELP_TOKENS for x in args)
