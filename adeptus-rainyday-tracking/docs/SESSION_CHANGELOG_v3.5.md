# Session Changelog — For Manual v0.6

> Handoff notes from the 2026-05-25 session covering all v3.5 code/UX changes.
> Use this alongside `docs/PERFORMANCE_METRICS.md` when revising the manual from v0.5 → v0.6.

---

## TL;DR — What's new and visible to the user

1. **`Tracked%` column** added to every truthSummary table — estimated tracked fraction as integer percent
2. **`Est failure` warning line** added to per-tracker summary boxes — fires when establishment took > 25% of a truth's lifetime
3. **Compact table format** replaces MATLAB's default wide `disp()` output, fits in narrow deployed-app terminals
4. **"View a Saved Run..." button** on the Validation & Documentation tab, plus matching `viewSavedResults` MATLAB command
5. **Track Swap Analysis windows** now show their tracker label in the window title and the first subplot title
6. **Platform-to-Track Assignment plot** now segment-colors track lines so swaps are visible in the line itself (not just at the red X marker)
7. **All metric docs** drafted at `docs/PERFORMANCE_METRICS.md` — ready to paste into manual sections
8. **AutoTune overhaul (v3.5 "amazing" mode)** — 6-component normalized score, per-row coverage display, sensitivity analysis, score-component breakdown for the winner
9. **CompareTrackers** uses the same 6-component score with a winner-vs-runner-up component-by-component diff

---

## Code changes by file

| File | Change |
|---|---|
| `src/+trackbench/+tracking/runTracker.m` | Added `TrackedPct` column to truthSummary; passes tracker label to plotTrackSwapAnalysis |
| `src/+trackbench/+reporting/plotPlatformToTrackAssignment.m` | Per-scan segment coloring of track lines (truth transitions visible in line color); label format e.g. `T01 → Truth 2→1` |
| `src/+trackbench/+reporting/plotTrackSwapAnalysis.m` | New optional `label` parameter (4th arg) — when set, window title becomes `Track Swap Analysis — <label>` and first subplot title becomes `<label> — Track-to-Truth Assignment Timeline` |
| `src/+trackbench/+reporting/printCompactTrackSummary.m` | **NEW** — compact 70-char trackSummary printer |
| `src/+trackbench/+reporting/printCompactTruthSummary.m` | **NEW** — compact 80-char truthSummary printer; bridges legacy `TrackedFraction` to new `TrackedPct` for old .mat files |
| `scripts/runSingleScenario.m` | Calls package printers instead of `disp()`; blank line between tables; added `Est failure` line (25% threshold) |
| `scripts/viewSavedResults.m` | **NEW** — replays per-tracker tables, summary box, assignment plots, and swap analysis from any saved `.mat`. CLI and GUI-callable |
| `scripts/validationDocsGUI.m` | Added "View a Saved Run..." button alongside "Open results/ folder" in a 2-column sub-grid on row 7; added `launchViewer` helper |
| `config/runs/swap_demo.json` | **NEW** — demo scenario (ultimately the user used their own redrawn `Swap.json` for the Boeing demo) |
| `config/targets/waypoints/swap_demo.json` | **NEW** — paired waypoint file |
| `docs/PERFORMANCE_METRICS.md` | **NEW** — full prose reference for all new metrics, ready for manual integration |

---

## What's already drafted in PERFORMANCE_METRICS.md (don't redo)

The metrics doc already contains complete prose for:

- Every column of the compact trackSummary table
- Every column of the compact truthSummary table (including the legacy `AssocTrk` caveat about "longest-associated, not currently-assigned")
- The per-tracker summary box layout including the new `Est failure` line
- A "CLEAN with Est failure" worked example using the JPDA Swap scenario result
- `viewSavedResults` usage (GUI + CLI)
- Tracker labeling on Track Swap Analysis windows
- Source-file cross-reference table

The next chat should be able to lift sections directly into manual chapters with light editing rather than writing fresh prose.

---

## Manual sections likely needing updates (v0.5 → v0.6)

Without reading the v0.5 manual exhaustively, the sections probably affected:

- **Output / Results interpretation chapter** — needs the full new column reference and the Est failure explanation. Most material is in `docs/PERFORMANCE_METRICS.md`.
- **GUI tour / Validation & Documentation tab** — needs the new "View a Saved Run..." button documented. Screenshot replacement may be needed.
- **Running a scenario chapter** — the per-tracker summary box example output needs replacing with the new compact format (and the new `Est failure` line).
- **Visualization chapter** — Platform-to-Track Assignment plot description needs the new segment-coloring behavior noted. Track Swap Analysis section needs the new window-label behavior.
- **Troubleshooting / Demo notes** — the "CLEAN swap count with high Est failure" failure mode is worth a callout because it's counterintuitive and a real Boeing demo talking point.

---

## AutoTune & CompareTrackers overhaul — the "amazing" v3.5 changes

### What changed and why

The pre-v3.5 score had a structural blind spot: a tracker could win by ignoring half the targets. Specifically, the old formula was

```
Score = w1·(posRMS/maxRange) + w2·swapCount + w3·log1p(falseTracks) + w4·breakCount
```

If a tracker dropped Truth 1 entirely and tracked Truth 2 perfectly, it could outscore a tracker that handled both imperfectly — because the dropped truth contributes nothing to any of those four metrics. The JPDA result on `Swap.json` (described in the demo story above) showed this concretely: JPDA scored "better" than GNN by old metrics, while actually being the worst performer.

The new score adds two coverage terms, normalizes every component to `[0, 1]`, and makes the weights interpretable as a *proportion of the total score*:

```
Score = w1·posErr + w2·swap + w3·false + w4·break + w5·tracked + w6·estFail
```

Default weights `[0.25 0.15 0.10 0.10 0.25 0.15]` sum to 1.0. A perfect tracker scores 0.000; a catastrophic one scores 1.000. Coverage (`tracked` + `estFail`) makes up 40% of the score — enough to prevent the JPDA-style failure mode.

Each component is normalized before weighting:

| Component | Formula | Saturates at |
|---|---|---|
| `posErr` | `avgPosRMS / maxRange` | RMS = maxRange |
| `swap` | `swapCount / 5` | 5 swaps |
| `false` | `log1p(falseTracks) / log1p(20)` | 20 false tracks |
| `break` | `breakCount / 5` | 5 breaks |
| `tracked` | `(100 - worstTrackedPct) / 100` | 0% worst-tracked |
| `estFail` | `estFailures / numTruths` | every truth fails |

Legacy 4-element weight vectors are auto-padded with default coverage weights and renormalized, with a one-line warning. Old callers keep working.

### What you see on screen

Pass 1 and Pass 2 sweep rows now show **all six metrics per row**:

```
   #  | Gate  Volume   Beta     Extra              |  RMS    Sw  FT  Brk Trkd EstF | Score
  ----+----------------------------------------------+-------------------------------+-------
    1 |   30  1e+07   1e-12   conf=8 del=-5         |  1234   0   2   0  92%  0/2 | 0.187 *
```

The `*` marks each new best as before. `Trkd` is the worst-tracked percentage across truths; `EstF` is the `failed/total` count of establishment failures.

The Pass 1 winner report now prints a **score-component breakdown** showing how the total decomposes into weighted contributions, plus a small bar chart:

```
  Score: 0.187  (lower is better, 0 = perfect)
  Component contributions (raw × weight = weighted):
    posErr   0.012 × 0.25 = 0.003  
    swap     0.000 × 0.15 = 0.000  
    false    0.099 × 0.10 = 0.010  █
    break    0.000 × 0.10 = 0.000  
    tracked  0.080 × 0.25 = 0.020  ██
    estFail  0.000 × 0.15 = 0.000  
```

At a glance, you can see *which terms* drove the optimizer's pick — not just the composite number.

### Sensitivity analysis (after the sweep)

A new section ranks parameters by how much they actually moved the score during the sweep, with both Spearman correlation (monotonic effects) and influence (non-monotonic effects):

```
  Param        Correlation    Influence  Levels   Verdict
  -----        -----------    ---------  ------   -------
  Gate            +0.821         0.752      4     strong monotonic effect: ↑ score (higher = worse)
  Beta            -0.612         0.541      3     strong monotonic effect: ↓ score (higher = better)
  ConfThresh      +0.082         0.341      3     moderate non-monotonic effect
  Volume          +0.044         0.118      3     minor effect
  DelThresh       +0.012         0.024      3     negligible — consider removing from sweep
```

Two immediate uses:
1. **Trust calibration** — see whether the winner won because of parameters that matter, or just got lucky on a parameter that didn't
2. **Sweep design** — parameters with `negligible` verdict can be removed from future sweeps (or fixed at a known-good value) to make tuning faster

Column names are tracker-aware: GNN sees `ConfThresh`/`DelThresh`, JPDA sees `ConfProb`/`DelProb`, TOMHT sees `MaxBranches`/`ConfThresh`. No `Extra1`/`Extra2` ambiguity in the report.

### Saved JSON includes the breakdown

The `TUNING_NOTES` block now records the full score decomposition plus the raw metrics, so you can re-inspect an old tune without re-running:

```json
"TUNING_NOTES": {
  "method": "autoTuneTracker 2-pass sweep + sensitivity (v3.5)",
  "score": 0.187,
  "score_components": { "posErr": 0.012, "swap": 0.000, ... },
  "weights": [0.25, 0.15, 0.10, 0.10, 0.25, 0.15],
  "raw_metrics": { "posRMS": 1234, "worstTrackedPct": 92, "estFailures": 0, ... }
}
```

### CompareTrackers gets the same treatment

`compareTrackers.m` was updated in lockstep: same 6-component scoring, same per-row format (RMS / Sw / FT / Brk / Trkd / EstF), and a new **winner-vs-runner-up** component-by-component diff at the bottom. Now you can see *why* the winner won — it's almost always one specific component driving the gap.

### Honest limitations (still worth documenting)

This is the "good" autotuner, not the "optimal" one. Things it still doesn't do:

1. **Sparse 2D grid** — the smart-grid samples gate×volume, beta×extra1, etc. with other parameters fixed at baseline. Cross-interactions of 3+ parameters at their joint optima are invisible. A full Cartesian product would catch them; we settled for the 2D sweep to keep runtime bounded.
2. **No replication** — each combo runs once. Tracking is stochastic; the "best" could be a lucky draw. Standard practice is 3–5 reps per combo with averaged scores. Not implemented because it triples runtime.
3. **Two-pass independence assumption** — Pass 1 (tracker params) and Pass 2 (filter params) are sequential, not joint. The Pass 1 winner is locked while Pass 2 sweeps filter params; if the optimal filter for *another* Pass 1 row would have been better, we miss it. Joint sweep would be ideal but combinatorially expensive.

These are documented here so the next iteration knows what was deferred and why. The current state is rigorous about what it measures and transparent about what it samples.

### Files affected

| File | Change |
|---|---|
| `src/+trackbench/+analysis/computeTunerScore.m` | **NEW** — shared 6-component scoring function (used by both auto-tune and compare) |
| `src/+trackbench/+analysis/extractTunerMetrics.m` | **NEW** — reads coverage from truthSummary, back-compat with legacy `TrackedFraction` |
| `src/+trackbench/+analysis/analyzeSensitivity.m` | **NEW** — ranks sweep parameters by Spearman correlation + influence |
| `scripts/autoTuneTracker.m` | Wholesale rewrite of scoring and display; sensitivity report added at end; saved JSON includes full breakdown |
| `scripts/compareTrackers.m` | Same scoring system; new per-row format with coverage columns; winner-vs-runner-up component diff |

All three new package files are independent of MATLAB toolboxes — the Spearman rank correlation in `analyzeSensitivity` is hand-rolled to avoid a Statistics Toolbox dependency.

---

## The demo story (worth its own paragraph in the manual)

On the `Swap.json` scenario with all three trackers enabled:

| Tracker | Swaps | Truth 1 Tracked% | Est failure | What this means |
|---|---|---|---|---|
| GNN + CV | 1 | 62% | Truth1 est@8/21 (38%) | Mishandled the crossing — got confused and swapped |
| TOMHT + IMM | 1 | 62% | Truth1 est@8/21 (38%) | Same failure mode as GNN |
| JPDA + IMM | 0 (CLEAN) | **10%** | Truth1 est@19/21 (**90%**) | **Never effectively tracked Truth 1** — looks clean by swaps but actually worse |

This is the story that motivated adding both new metrics. Swap count alone makes JPDA look like the winner; the combination of `Tracked%` and `Est failure` exposes that it's the loser. Worth a worked-example callout in the manual.

---

## Behavior caveats worth knowing about

1. **`AssocTrk` semantics are MATLAB's, not custom.** The `AssociatedTrackID` column in truthSummary reports the track that spent the most cumulative time associated with the truth — not the currently-assigned track. If no track properly associated with a truth, the briefly-associated one still wins by default. Users misread this constantly. The manual should be explicit.

2. **The 25% Est failure threshold is hardcoded.** Lives in `scripts/runSingleScenario.m` at the `if tl > 0 && (el / tl) > 0.25` line. Not currently configurable via run JSON. If a future revision exposes it, the manual line will need updating; for now, state "25%" verbatim.

3. **`Tracked%` is an upper bound.** Computed as `(TotalLength − EstablishmentLength) / TotalLength × 100`. Assumes no track breaks after establishment. If `Breaks > 0`, the real tracked fraction is lower. The platform-to-track assignment plot is the authoritative scan-by-scan view; this column is a quick summary.

4. **Saved-run viewer doesn't replay the 3D scenario animation.** That requires the live tracker stepping through detections, which we don't have post-hoc. Assignment plots and swap analysis plots ARE reconstructed from saved data. Worth noting in the manual.

5. **OneDrive sync occasionally drops newly-created files.** Hit twice this session: `viewSavedResults.m` and the two `printCompact*Summary.m` files. The pattern: `create_file` reports success, the file isn't actually persisted, the next build is missing it. Workaround used: `write_file` instead. If the next chat creates new files via the Filesystem tool, verify they exist in the build location before rebuilding. Not a manual concern — internal note.

6. **Backward compatibility for old .mat files is built in.** `printCompactTruthSummary` reads either the new `TrackedPct` integer column or the legacy `TrackedFraction` float column. Saved runs from before the rename still display the same compact percentage. Users don't need to re-run anything.

---

## Build workflow reminder

After any code change:

```matlab
clear classes; clear all
addpath("scripts"); addpath(genpath("src"));
build_executable          % ~5 min — compiles mainMenu.exe
build_installer('web')    % ~30s — wraps EXE in web installer
```

`build_executable.m` auto-picks up new scripts via its `dir(fullfile(scriptsDir, '*.m'))` loop, and new package files via `addpath(genpath(srcDir))` + `-a srcDir`. No build-script edits are needed when adding new code — just rebuild.

---

## Open work / not done this session

These were on the project plan but not touched in this session:

- README v3.5.0+ update — version bump and changelog entry
- Codebase cleanup (the previously-planned 19-file archive to `scripts/legacy/`, 4-file deletion)
- Physics verification against published references
- Tracker editor substeps 2.4 (Globals editor) and 2.5 (validation polish)
- Post-demo Flight Data Manager feature

Mentioning these so the next chat knows what's NOT covered by this session's changes.
