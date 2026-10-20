# Discourse post draft — for the human to adapt/post

> Title: *Simulation land: rehearse drone missions like software —
> scenarios, seeded ground truth, POD grading*

Hi all — we've been building a public scenario playground for
ArduPilot-backed drone operations and it's ready for people to try.

**What it is.** One YAML file rehearses a whole mission on your desk:
the fleet (SITL copters/planes/rovers or a fast fake transport), the
mission package that would really deploy, a scripted operator
(approvals, confirms, vetoes), a timeline of steps, injected faults —
and an `assert:` block that grades the outcome the way a reviewer
grades a flight log.

**What's different from "run SITL and watch".** The world is explicit
ground truth: people, phones, canopy, buildings, decoys — all local-ENU
around an origin. The sim raycasts camera looks through the clutter,
models SDR/thermal signatures, and can tell you that your survey
covered 99 % of the block but only had a 0.9 probability of detecting
the person standing under the trees. Coverage and POD are graded
separately — we've written up *why* in
`docs/articles/pod-under-canopy.md`.

**Try it:**

```bash
./sim doctor                                     # environment check
./sim list scenarios                             # the demo gallery
./sim view scenarios/demos/world_canopy_viewer.yaml   # forest search,
                                     # live in a 3D viewer
./sim run scenarios/demos/battery_low_rtl.yaml   # battery dies,
                                     # the op's handler flies home
```

There's a monthly challenge too (`docs/CHALLENGE.md`): fixed world,
fixed seed, submit your `summary.json`, we replay the seed to verify
the score. And a course-assignment kit (`docs/EDU_ASSIGNMENT.md`) if
you teach this stuff.

**What we're asking:** does the scenario format say what you'd actually
want to assert about a mission? What fault would you inject that we
don't model? And if you try the Docker quickstart — that's the piece
still being finished; feedback on the host-machine path is what we
need most today.

*(Link to repo + docs/SCENARIOS.md to be filled in at post time.)*
