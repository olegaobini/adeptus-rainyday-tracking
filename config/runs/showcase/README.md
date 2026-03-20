# Showcase Scenarios

These are the original catalog scenarios converted to modular run files.
Each is a proven, tested scenario from the Boeing demo.

```matlab
runSingleScenario("showcase/dasr_ideal")        %% DASR baseline
runSingleScenario("showcase/crossing_targets")  %% Track swap test
runSingleScenario("showcase/demo_ideal")        %% Boeing demo (5 targets, mountain)
runSingleScenario("showcase/fighter_intercept")  %% AESA+FLIR airborne
```

These use the same sensors/targets/terrain as the catalog originals
but through the modular system. Each component can be swapped independently.
