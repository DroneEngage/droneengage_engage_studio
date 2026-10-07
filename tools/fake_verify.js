#!/usr/bin/env node
// fake_verify.js - stands in for the P5-06 verifier CLI while it is not
// in the webclient tree: prints a JSON report shaped like the real one
// ({error_count, errors:[{rule}]}) so the scenario runner's verify gate
// can be exercised end to end. Exit 1 = findings, 0 = clean; the mode is
// picked from the operation file's basename ("clean" -> 0).
const base = (process.argv[2] || "").toLowerCase();
const clean = base.includes("clean");
const rep = clean
  ? { error_count: 0, errors: [], warnings: [] }
  : { error_count: 1,
      errors: [{ rule: "bad_lane_overlap",
                 msg: "lane 2 overlaps lane 1 over the river" }],
      warnings: [{ rule: "no_abort_plan", msg: "no abort mission" }] };
process.stdout.write(JSON.stringify(rep));
process.exit(clean ? 0 : 1);
