# Run Files

Each .json here defines a **complete simulation**.
Pass the filename (no .json) to runSingleScenario:

```matlab
runSingleScenario("dasr_baseline")    %% PSR+SSR, rural, GNN+JPDA
runSingleScenario("demo_mountain")    %% Boeing demo: 5 targets, mountain
runSingleScenario("my_run")           %% Your custom run
```

## Format
- **sensors**: list of sensor files from `config/sensors/<TYPE>/<file>`
- **targets**: target file from `config/targets/<pattern>/<file>`
- **terrain**: terrain file from `config/terrain/<type>/<file>`
- **trackers**: list of tracker files from `config/trackers/<TYPE>/<file>`
- **degradation**: `{ "enabled": true/false, "type": "rain" }`
- **platforms**: moving platforms (aircraft, ship). Omit `placeholder` field for no platforms.
- **output**: visuals, save, diagnostics

See `run_template.json` for a fully documented example.
