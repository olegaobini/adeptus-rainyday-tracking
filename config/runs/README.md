# Run Files

Each .json here defines a **complete simulation**.
Pass the filename (no .json) to runSingleScenario:

```matlab
runSingleScenario("dasr_baseline")                %% PSR+SSR, rural, GNN+JPDA
runSingleScenario("demo_multi_5_tropicalStorm")   %% Multi-target + violent tropical rain
runSingleScenario("my_run")                       %% Your custom run
```

## Format
- **sensors**: list of sensor files from `config/sensors/<TYPE>/<file>`
- **targets**: target file from `config/targets/<PATTERN>/<file>`
- **terrain**: terrain file from `config/terrain/<TYPE>/<file>`
- **trackers**: list of tracker files from `config/trackers/<TYPE>/<file>`
- **compare_trackers** *(optional)*: list of tracker files to compare head-to-head via `compareTrackers("<runName>")`. Leave `[]` to skip.
- **degradation**: environment + weather toggles, all in one block:
  - `terrain_occlusion` — LOS checks against terrain heightmap
  - `horizon_masking` — 4/3 Earth radius model
  - `ground_clutter` — terrain-dependent false returns
  - `doppler_fade` — tangential targets fade in MTI clutter notch
  - `weather` — config from `config/weather/<TYPE>/<file>`, or `"none"` for clear
- **platforms**: moving platforms (aircraft, ship). Leave `{}` for all stationary.
- **cache**: `{ use_cached_detections, save_detections }` — set `use_cached_detections: true` to skip detection generation and reuse cached results (faster tracker tuning).
- **output**: visuals, save, diagnostics

See `run_template.json` for a fully documented example.

## Removed fields (v3.4.1)
- `rcs_range_filter` — RCS is now handled natively by `fusionRadarSensor` via `platform.Signatures` per R2025b best practice. External filter caused double-counting and has been removed.
