# Performance Metrics Reference

> Draft notes for the user manual covering metrics and viewer features added in v3.5+.
> Style is plain-prose, ready to paste into `RainyDay_User_Manual_v0_2.docx` or a follow-on revision.

---

Each tracker run produces two tables and a summary box. The columns and warning lines documented here cover what's new in v3.5+ and what each number is telling you.

## Reading the trackSummary table

One row per track the tracker created.

| Column | Meaning |
|---|---|
| `TrkID` | Internal track identifier assigned by MATLAB |
| `Truth` | Which truth this track was associated with at the end of its life. `NaN` means the track was never confidently associated with any truth (typically a clutter track) |
| `Surv` | Did the track survive to the end of the scenario? |
| `Len` | Number of scans the track existed for |
| `Reported` | Was the track reporting consistently? `no` means there were gaps longer than the configured tolerance |
| `MeanTBR` / `MaxTBR` | Mean and maximum **Time Between Reports** in seconds. Lower is better — large gaps usually indicate a struggling track |
| `Diverged` | Did the track's position estimate diverge from truth by more than the divergence threshold? |
| `Swaps` | Number of times this track changed its truth assignment mid-life. A swap means the tracker got confused between targets |

## Reading the truthSummary table

One row per real target in the scenario.

| Column | Meaning |
|---|---|
| `TruID` | Truth identifier |
| `AssocTrk` | The track that spent the most cumulative time associated with this truth. **Note**: this is "longest-associated," not "currently-assigned." A track that briefly associated near the end can still appear here if no other track ever associated with the truth |
| `TotLen` | Number of scenario steps the truth existed for |
| `EstLen` | Number of steps before any track was first associated with the truth. Higher means slower establishment |
| `Tracked%` | **New in v3.5.** Estimated fraction of the truth's lifetime that was tracked, as an integer percent. Computed as `(TotLen − EstLen) / TotLen × 100`. This is an **upper bound**: it assumes no track breaks after establishment, so the real tracked fraction can be lower if `Breaks > 0`. For the authoritative view of who was tracking what scan-by-scan, see the platform-to-track assignment plot |
| `Breaks` | How many times the truth lost its track mid-life and had to re-establish |
| `MeanTBR` / `MaxTBR` | Mean/max time between reports for this truth |
| `Reported` | `no` means the truth had a gap of unreported scans |

## Reading the per-tracker summary box

```
  ┌─────────────────────────────────────────────
  │ JPDA + IMM  SUMMARY
  │ Track Swaps : CLEAN
  │ Position RMS: T1=716.0m  T2=773.6m  ...
  │ Velocity RMS: T1=250.6m/s  T2=248.3m/s  ...
  │ Quality     : T1=716m / 4.4% (Acceptable)  ...
  │ Est failure : Truth1 est@19/21 (90%)
  └─────────────────────────────────────────────
```

| Line | Meaning |
|---|---|
| `Track Swaps` | `CLEAN` if no track changed truth mid-life, otherwise the count |
| `Position RMS` | Per-truth root-mean-square position error in meters |
| `Velocity RMS` | Per-truth RMS velocity error in m/s |
| `Quality` | RMS as a percentage of max scenario range. Bands: < 1% Excellent, 1–3% Good, 3–5% Acceptable, > 5% Poor |
| `Est failure` | **New in v3.5.** Appears only when a truth's establishment took more than 25% of its lifetime. Read `Truth1 est@19/21 (90%)` as "Truth 1 wasn't picked up until scan 19 of 21 — 90% of the way through its lifetime." This warns of a tracker failure that swap count alone can miss |

## What "CLEAN with Est failure" tells you

If the summary shows `Track Swaps: CLEAN` and `Est failure: Truth1 est@19/21 (90%)`, the tracker did not visibly mishandle any track identity — but it also never effectively tracked Truth 1. It briefly associated with that truth near the end of its life and that was the only association.

This is a worse outcome than a swap, because a swap implies the tracker had both targets and confused them; a high `Est failure` value implies the tracker never had both targets in the first place. Always check the Est failure line alongside the swap count.

## Reviewing past runs

Each completed run is saved to `results/results_<scenario>_<timestamp>.mat`. Two ways to revisit:

- **From the GUI**: open the Validation & Documentation tab and click **"View a Saved Run..."**. Pick a `.mat` file; the per-tracker summary tables, summary box, and assignment plots will be reproduced.
- **From the MATLAB command line**:
  ```matlab
  viewSavedResults                          % opens a file picker
  viewSavedResults("path/to/results.mat")   % opens directly
  ```

The reviewer reproduces everything from the saved file. The 3D scenario animation is **not** replayed (it depends on the live tracker stepping through detections), but the platform-to-track assignment plots and swap analysis plots are fully reconstructed.

## Identifying which tracker a plot belongs to

When more than one tracker swaps, multiple "Track Swap Analysis" windows open at once. Each is now labeled with its tracker:

- Window title: `Track Swap Analysis — GNN + CV`
- First subplot title: `GNN + CV — Track-to-Truth Assignment Timeline`

The same applies to the assignment-plot tabs in the main tracker output window.

---

## Notes for the manual editor

Two things worth a reviewer's eye before publishing in the manual:

1. **The "longest-associated, not currently-assigned" caveat on `AssocTrk`** — this is MATLAB's behavior, not custom code, and it really does trip people up. The Swap scenario JPDA result demonstrates it: Truth 1's `AssocTrk = 38` even though Track 38 only existed for 3 scans, because no other track ever associated with Truth 1 at all. Worth keeping in the manual.

2. **The 25% Est failure threshold is currently hardcoded** in `runSingleScenario.m`. If a future revision makes it configurable (e.g., an `est_failure_threshold` field in the run JSON), the corresponding manual line would need updating, but stating "25%" verbatim is correct for v3.5.

## Source files (for cross-referencing in the manual)

| Feature | File |
|---|---|
| `Tracked%` column computation | `src/+trackbench/+tracking/runTracker.m` |
| `Est failure` warning line | `scripts/runSingleScenario.m` |
| Compact track summary printer | `src/+trackbench/+reporting/printCompactTrackSummary.m` |
| Compact truth summary printer | `src/+trackbench/+reporting/printCompactTruthSummary.m` |
| Saved-run viewer (command + GUI button) | `scripts/viewSavedResults.m`, `scripts/validationDocsGUI.m` |
| Tracker label on swap analysis figure | `src/+trackbench/+reporting/plotTrackSwapAnalysis.m` |
