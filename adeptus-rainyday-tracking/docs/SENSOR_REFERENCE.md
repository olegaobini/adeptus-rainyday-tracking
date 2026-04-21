# Sensor Reference — buildSensor.m

## Quick Start

```matlab
% Create sensors
[psr, ~] = buildSensor(1, 'PSR');                        % Primary Search Radar
[ssr, ~] = buildSensor(2, 'SSR');                        % Secondary Surveillance Radar
[irst,~] = buildSensor(3, 'IRST');                       % IR Search & Track

% Override defaults
[psr, ~] = buildSensor(1, 'PSR', 'rpm', 15, 'pd', 0.85, 'rangeLimits', [0 150000]);

% Add to scenario
scenario = trackingScenario;
tower = platform(scenario, 'Sensors', {psr, ssr});
```

## All Sensor Types

### Radar Family (fusionRadarSensor)
| Type | Description | Range | RPM | Scan Mode |
|------|-------------|-------|-----|-----------|
| `PSR` | Primary Search Radar | 60 nm | 12.5 | Rotator |
| `SSR` / `MSSR` / `IFF` | Secondary Surveillance Radar | 120 nm | 12.5 | Rotator |
| `ASR` | Airport Surveillance Radar | 60 nm | 12.5 | Rotator |
| `ARSR` | Air Route Surveillance Radar | 250 nm | 5 | Rotator |
| `PAR` | Precision Approach Radar | 20 nm | — | Sector |
| `TWS` | Track-While-Scan phased array | 200 km | — | No scanning |
| `AESA` | Active Electronically Scanned Array | 300 km | — | Sector |
| `FIRE_CONTROL` | Fire control radar | 150 km | — | No scanning |
| `WEATHER` | Weather radar (NEXRAD-like) | 250 nm | 25 | Raster |
| `MARITIME` | Maritime / surface search radar | 40 nm | 24 | Rotator |
| `CUSTOM_RADAR` | Bare fusionRadarSensor | user | user | Rotator |

### Infrared Family (irSensor)
| Type | Description | Range | RPM | Scan Mode |
|------|-------------|-------|-----|-----------|
| `IRST` | IR Search & Track | 100 km | 60 | Rotator |
| `IR_STARING` | Staring IR sensor | 50 km | — | No scanning |
| `FLIR` | Forward-Looking Infrared | 30 km | — | Sector |
| `CUSTOM_IR` | Bare irSensor | user | user | No scanning |

### Sonar Family (sonarSensor)
| Type | Description | Range | RPM | Scan Mode |
|------|-------------|-------|-----|-----------|
| `ACTIVE_SONAR` | Active sonar (ping + listen) | 20 km | 6 | Rotator |
| `PASSIVE_SONAR` | Passive sonar (listen only) | 50 km | — | No scanning |
| `TOWED_ARRAY` | Towed array sonar | 80 km | — | Sector |
| `CUSTOM_SONAR` | Bare sonarSensor | user | user | No scanning |

### Lidar (monostaticLidarSensor)
| Type | Description | Range | Update Rate |
|------|-------------|-------|-------------|
| `LIDAR` | Monostatic lidar point cloud | 200 m | 10 Hz |
| `CUSTOM_LIDAR` | Bare monostaticLidarSensor | user | user |

### ADS-B
| Type | Description | Notes |
|------|-------------|-------|
| `ADSB_TX` | ADS-B Transponder | Attach to aircraft platform |
| `ADSB_RX` | ADS-B Receiver | Attach to ground station |

### Custom
| Type | Description |
|------|-------------|
| `CUSTOM` | Returns a template struct showing required interface |

## Common Parameters (all radar/IR/sonar types)

```matlab
% These can be passed as name-value pairs to any radar/IR/sonar sensor:
'rpm'          — Rotation rate (rev/min)
'fov'          — [azimuth; elevation] beamwidth in degrees
'sector'       — [min max] azimuth scan limits in degrees
'tilt'         — Elevation tilt offset in degrees
'pd'           — Detection probability (0-1)
'far'          — False alarm rate
'rangeLimits'  — [min max] range in meters
'rangeRes'     — Range resolution in meters
'hasElevation' — true/false
'hasRangeRate' — true/false
'hasINS'       — true/false (default true)
'detCoords'    — 'Scenario' | 'Body' | 'Sensor spherical'
'mountingLoc'  — [x y z] meters on platform
'updateRate'   — Hz (overrides RPM-derived rate)
```

Any fusionRadarSensor property not in this list can still be passed as a
name-value pair — the factory applies unmatched parameters directly to
the sensor object via `isprop`.

## Creating a Custom Sensor

Three options:

### Option A: CUSTOM_RADAR with overrides
```matlab
[s, m] = buildSensor(5, 'CUSTOM_RADAR', ...
    'fov', [2; 20], 'rangeLimits', [0 500000], 'updateRate', 10);
```

### Option B: Subclass customSensorTemplate
Copy `customSensorTemplate.m`, rename the class, implement `stepImpl()`.
Must return `{objectDetection, ...}` from `stepImpl(obj, targets, time)`.

### Option C: Wrap an external sensor
Write a thin wrapper class that converts your sensor's native output
into `objectDetection` format compatible with the trackers.

## Integration with Existing Code

`buildSensor` is a superset of `buildCustomFusionRadarSensor` and `buildIFFSensor`.
Those files still work and are used by `createScenario3D`. For new scenarios,
prefer `buildSensor` for consistency:

```matlab
% Old way (still works)
[psr, meta] = buildCustomFusionRadarSensor(1, 'rpm', 12.5);
[ssr, meta] = buildIFFSensor(2);

% New way (recommended)
[psr, meta] = buildSensor(1, 'PSR');
[ssr, meta] = buildSensor(2, 'SSR');
```
