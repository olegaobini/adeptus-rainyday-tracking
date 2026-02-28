# Trackbench Config Workflow Guide

## Quick Start

**Run a single test:**
```matlab
>> runSingleScenario("scenarios/my_test")
```

**Run a parameter sweep:**
```matlab
>> runParameterSweep("sweeps/weather_study")
```

---

## Directory Structure

```
config/
├── default.json              System defaults (rarely touch)
├── templates/                Starting points for scenarios
├── scenarios/                Your test cases (CREATE THESE)
├── sweeps/                   Parameter studies (CREATE THESE)
└── components/               Reusable building blocks
    ├── sensors/
    └── weather/
```

---

## The Workflow in 3 Steps

### 1. Create a Scenario

**Option A: Use a template (recommended)**

```json
// config/scenarios/my_test.json
{
  "template": "standard_crossing",
  "overrides": {
    "weather": "heavy_rain",
    "trackers": ["gnn_cv", "tomht_cv"]
  }
}
```

This says:
- Start with `templates/standard_crossing.json`
- Change weather to heavy rain
- Test only GNN and TOMHT trackers

**Option B: Define everything inline**

```json
// config/scenarios/my_test.json
{
  "scenario": {
    "mode": "3D",
    "duration_s": 100
  },
  "sensors": ["tower_radar_v1"],
  "weather": "clear",
  "truth": {
    "targets": [
      { "path": "data/truths/crossing_pair.csv" }
    ]
  },
  "trackers": ["gnn_cv", "tomht_cv", "jpda_cv"]
}
```

### 2. Test Your Scenario

```matlab
>> runSingleScenario("scenarios/my_test")
```

**Output:**
- Plots appear showing trajectories, detections, tracks
- Metrics printed to console
- Results saved to `outputs/`

### 3. Create a Sweep (Optional)

Once your scenario works, vary parameters:

```json
// config/sweeps/weather_study.json
{
  "sweep_name": "weather_impact",
  "base_scenario": "scenarios/my_test.json",
  "sweep": {
    "mode": "single",
    "parameter": "weather",
    "values": ["clear", "light_rain", "heavy_rain", "fog", "storm"]
  }
}
```

**Run it:**
```matlab
>> runParameterSweep("sweeps/weather_study")
```

**Output:**
- Runs 5 simulations automatically
- Generates comparison plots
- Creates summary report in `outputs/`

---

## Config Hierarchy (What Overrides What)

```
default.json
    ↓  (fills in missing fields)
templates/*.json
    ↓  (overrides defaults)
scenarios/*.json
    ↓  (overrides template)
sweeps/*.json
    ↓  (varies scenario parameters)
```

**Lower levels win conflicts.**

---

## How to Reference Components

### Simple String Reference

```json
"weather": "heavy_rain"
```

Looks for `config/components/weather/heavy_rain.json`

### Reference with Overrides

```json
"weather": {
  "ref": "heavy_rain",
  "overrides": {
    "pd": 0.65
  }
}
```

Loads `heavy_rain.json` then changes `pd` value.

### Inline Definition

```json
"weather": {
  "profile": "custom",
  "pd": 0.70,
  "range_multiplier": 0.85
}
```

Define directly without referencing a file.

---

## Common Workflows

### Workflow 1: Test Existing Trackers on New Scenario

**Files to create:** 1

```json
// config/scenarios/my_scenario.json
{
  "template": "standard_crossing",
  "overrides": {
    "weather": "storm",
    "truth": {
      "targets": [{ "path": "data/truths/my_paths.csv" }]
    }
  }
}
```

```matlab
>> runSingleScenario("scenarios/my_scenario")
```

**Time:** 5 minutes

---

### Workflow 2: Compare Trackers Across Weather

**Files to create:** 2

**Scenario:**
```json
// config/scenarios/baseline.json
{
  "template": "standard_crossing",
  "overrides": {
    "trackers": ["gnn_cv", "tomht_cv", "jpda_cv"]
  }
}
```

**Sweep:**
```json
// config/sweeps/weather_comparison.json
{
  "sweep_name": "weather_study",
  "base_scenario": "scenarios/baseline.json",
  "sweep": {
    "mode": "single",
    "parameter": "weather",
    "values": ["clear", "light_rain", "heavy_rain"]
  }
}
```

```matlab
>> runParameterSweep("sweeps/weather_comparison")
```

**Time:** 10 minutes to set up, hours to run

---

### Workflow 3: Optimize Parameters

**Files to create:** 1

```json
// config/sweeps/gate_optimization.json
{
  "sweep_name": "find_best_gate",
  "base_scenario": "scenarios/baseline.json",
  "sweep": {
    "mode": "grid",
    "parameters": {
      "weather": ["clear", "heavy_rain"],
      "trackers.params.gate": [30, 35, 40, 45]
    }
  }
}
```

This creates 2 × 4 = 8 runs (all combinations).

```matlab
>> runParameterSweep("sweeps/gate_optimization")
```

---

## Sweep Modes

### Single Parameter Sweep

Test one parameter with multiple values:

```json
"sweep": {
  "mode": "single",
  "parameter": "weather",
  "values": ["clear", "rain", "storm"]
}
```

**Output:** 3 runs

---

### Grid Sweep (Cartesian Product)

Test all combinations:

```json
"sweep": {
  "mode": "grid",
  "parameters": {
    "weather": ["clear", "rain"],
    "trackers.params.gate": [35, 40, 45]
  }
}
```

**Output:** 2 × 3 = 6 runs

---

### List Sweep (Specific Combinations)

Hand-pick combinations:

```json
"sweep": {
  "mode": "list",
  "configs": [
    { "weather": "clear", "trackers.params.gate": 35 },
    { "weather": "rain", "trackers.params.gate": 45 },
    { "weather": "storm", "trackers.params.gate": 50 }
  ]
}
```

**Output:** 3 runs (exactly as specified)

---

## What Gets Loaded (Behind the Scenes)

When you run a scenario, the system:

1. **Reads your scenario file**
2. **Loads template** (if specified)
3. **Merges overrides** on top
4. **Resolves component refs** (loads `weather.json`, `sensors.json`, etc.)
5. **Fills in defaults** from `default.json`
6. **Validates** everything is correct
7. **Normalizes** units and coordinates
8. **Runs simulation**

**You only touch:** scenario files and sweep files
**System handles:** merging, validation, loading components

---

## Tips and Best Practices

### Naming Conventions

- **Scenarios:** Descriptive names like `crossing_heavy_rain.json`, `high_density_clear.json`
- **Sweeps:** Purpose-based like `weather_comparison.json`, `gate_optimization.json`
- **Components:** Versioned like `tower_radar_v1.json`, `storm_profile_v2.json`

### Start Simple

1. Copy an existing scenario that's close to what you want
2. Change 1-2 fields
3. Test it works
4. Then create sweeps

### Use Templates

Create templates for common scenario types:
- `templates/crossing.json` - Two aircraft crossing
- `templates/high_density.json` - Many targets
- `templates/maritime.json` - Ships and aircraft

Then scenarios just override what's different:
```json
{
  "template": "crossing",
  "overrides": { "weather": "storm" }
}
```

### Component Library

Build up a library over time:
- `components/sensors/` - Different radar configurations
- `components/weather/` - Weather profiles you test often
- `components/paths/` - Common trajectory patterns

Reuse them across scenarios.

---

## Troubleshooting

### "Missing required field: X"

**Problem:** Config is incomplete

**Fix:** Either add the field or make sure you're using a template that has it

```json
// Add missing field
{
  "scenario": { "mode": "3D", "duration_s": 100 }
}

// Or use template
{
  "template": "standard_crossing",
  "overrides": { ... }
}
```

---

### "Component not found: weather/my_weather"

**Problem:** Referenced component doesn't exist

**Fix:** Check the file exists at `config/components/weather/my_weather.json`

Or define inline:
```json
"weather": {
  "profile": "custom",
  "pd": 0.70
}
```

---

### "Sweep parameter does not exist: trackers.params.gate"

**Problem:** Trying to sweep a parameter that doesn't exist in the config

**Fix:** Check the parameter path is correct. Run single scenario first to see what's available.

---

## Quick Reference Card

| Task | Command |
|------|---------|
| Run single scenario | `runSingleScenario("scenarios/name")` |
| Run parameter sweep | `runParameterSweep("sweeps/name")` |
| Reference component | `"weather": "heavy_rain"` |
| Override component | `"weather": { "ref": "heavy_rain", "overrides": {...} }` |
| Inline definition | `"weather": { "pd": 0.7, ... }` |
| Use template | `"template": "standard_crossing"` |
| Single param sweep | `"mode": "single", "parameter": "weather"` |
| Grid sweep | `"mode": "grid", "parameters": {...}` |

---

## File Templates

### Minimal Scenario Template

```json
{
  "template": "standard_crossing",
  "overrides": {
    "weather": "clear"
  }
}
```

### Minimal Sweep Template

```json
{
  "sweep_name": "my_study",
  "base_scenario": "scenarios/my_test.json",
  "sweep": {
    "mode": "single",
    "parameter": "weather",
    "values": ["clear", "rain"]
  }
}
```

### Full Scenario Template (No Template)

```json
{
  "description": "Description of what this tests",
  "scenario": {
    "mode": "3D",
    "duration_s": 100,
    "frame": "NED"
  },
  "sensors": ["tower_radar_v1"],
  "weather": "clear",
  "truth": {
    "targets": [
      { "path": "data/truths/aircraft_1.csv" }
    ]
  },
  "trackers": ["gnn_cv", "tomht_cv"]
}
```

---

## Summary

**To run a test:**
1. Create/edit scenario file in `config/scenarios/`
2. Run `runSingleScenario("scenarios/your_file")`

**To sweep parameters:**
1. Create scenario (if you haven't)
2. Create sweep file in `config/sweeps/`
3. Run `runParameterSweep("sweeps/your_sweep")`

**That's it.** System handles everything else.

---

## Questions?

Check:
- `config/scenarios/` for example scenarios
- `config/templates/` for starting points
- `config/components/` for available components

Or run a simple example to see how it works:
```matlab
>> runSingleScenario("scenarios/clear_weather")
```
