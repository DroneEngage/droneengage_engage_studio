# tests/sar — Phase-4B SAR acceptance scenarios

TASK-P4B-10: the reference search-and-rescue operation run end to end on
the multi-SITL harness. Results are recorded in
`Tasks/mission_planner/Phase-4B-Tasks/ACCEPTANCE.md`.

| file | spec | what it proves |
|---|---|---|
| `p4b10_a_nominal.yaml` | A | phone signal → 3 disjoint sectors → person detected + operator confirm → launch approval → drop within 15 m of X → OP_END + RTLs |
| `p4b10_b_auto_veto.yaml` | B | person `auto_with_veto 30` confirms itself ~30 s after the first detection; the launch approval is still operator-gated |
| `p4b10_c_searcher_lost.yaml` | C | s2's link is cut mid-sector; a survivor adopts its remaining lanes (`TAKEOVER`) and completes them |
| `p4b10_d_gcs_out.yaml` | D | GCS session gone after deploy: vehicle coordinator allocates; approvals wait and resolve when the GCS is back |
| `p4b10_e_loss30.yaml` | E | 30 % loss on every server link; same outcome as A, relaxed timing bounds |
| `p4b10_f_drop_denied.yaml` | F | drop approval denied: no release, ambulance comes home, target stays confirmed, no OP_END |
| `p4b10_g_no_autolaunch.yaml` | G | `sv_allow_auto_launch: false`: the ambulance refuses the start (journaled), never arms |
| `p4b10_h_decomm_restart.yaml` | H | s1's de_comm restarted mid-sector: package + dynamic segment restored, run resumes RUNNING, sector completes, no takeover |
| `p4b10_i_static_sources.yaml` | I | two static SDR masts confirm the phone (`k_sources_within k:2`); a static camera mast sees the person; statics never get sectors, drain on OP_END |
| `p4b10_j_approval_timeout.yaml` | J | drop approval raised while the GCS is gone: request pends across the outage, dies at `approval_timeout_s` (`APPROVAL_TIMEOUT`), release never dispatched, ambulance falls back to RTL |
| `p4b11_copter_hold.yaml` | P4B-11 a–g | owned mission hold (DE-Pilot op 5): GUIDED pin at 80 m, upload-start pin-alt regression, GCS/DE-Pilot/mode takeovers pause the run, de_comm restart keeps the pin, plane LOITER smoke |

Inputs:

- world: `worlds/sar_forest.yaml` (person_1 at X, phone_1 at X + 30 m N;
  the camera model only boxes `detect_cls: [person]` — a legacy box has no
  class, so a visible phone would become a second person target);
- ops: `Tasks/mission_planner/ops/sar_forest*.deop`, compiled into
  `packages/sar_forest*.{deploy,pkg}.json` with
  `tools/compile_op.js`:

```bash
node tools/compile_op.js ../Tasks/mission_planner/ops/sar_forest.deop --out-dir packages --pkg-prefix sar_forest --binding "cam_scout=scout1;sdr_scout=sdr1;searcher=s1,s2,s3;ambulance=amb1"
```

(`sar_forest_veto` takes the same binding; `sar_forest_static` binds
`cam_scout=scout1;sdr_mast=sdrm1,sdrm2;cam_mast=camm1;searcher=s1,s2,s3;ambulance=amb1`.)

Every sensor is a world stub (`stub_ai_box`, `stub_sdr`, `stub_payload`) —
none of these runs touches real camera / SDR / AI hardware.

Run one scenario at a time (they share the seed-derived port block):

```bash
./run_scenario.py tests/sar/p4b10_a_nominal.yaml
```
