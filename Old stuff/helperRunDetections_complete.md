# helperRunDetections.m: Complete Top-to-Bottom Walkthrough

## THE BIG PICTURE FIRST

What does this function actually DO?

1. **Takes a radar scenario** (targets + rotating radar)
2. **Simulates the radar running in real-time**, scan by scan
3. **Collects detections** from each scan
4. **Optionally degrades detections** to simulate rain/weather
5. **Returns a log** with timestamps, truth positions, and detection lists

Think of it like: **Press "play" on the scenario, record everything the radar sees, optionally make it noisier to simulate rain, then give the data to the tracker.**

The output is what feeds into `helperRunTracker.m`.

---

# PART 1: FUNCTION START & SETUP (Lines 1-60)

## Line 1: Function Definition

```matlab
function dataLog = helperRunDetections(scenario, enableDegradation)
```

**What it means:**
- Input: `scenario` (a `trackingScenario` object from `helperCreateScenario3D`)
- Input: `enableDegradation` (boolean: should we add weather effects?)
- Output: `dataLog` (a struct with detection log)

**Pattern:** This is a classic MATLAB "helper" function. Small, focused, does one thing.

---

## Lines 3-6: Default Parameter & Logging

```matlab
if nargin < 2
    enableDegradation = false;   % Default to IDEAL
end
fprintf("enableDegradation = %d\n", enableDegradation);
```

**What it does:**
- If caller doesn't provide `enableDegradation`, default to `false` (no weather effects)
- Print to console which mode we're running in

**Why this matters:**
- When you're debugging, you want to know: "Am I testing ideal detections or degraded detections?"
- Without this print, you could accidentally run ideal detections and compare to degraded truth, thinking the algorithm is broken

**Comment:** The `fprintf` is good practice. Many MATLAB scripts fail silently because you don't know what mode you're in.

---

## Lines 8-10: Extract Radar from Scenario

```matlab
tower = scenario.Platforms{1};
radar = tower.Sensors{1};
```

**What it does:**
- `scenario.Platforms{1}` = first platform in scenario = the radar tower (stationary)
- `tower.Sensors{1}` = first sensor on the tower = the radar

**Why this structure?**
MATLAB's `trackingScenario` has a hierarchy:
```
trackingScenario
├── Platforms (array of platform objects)
│   ├── [1] Tower (radar location)
│   │   └── Sensors
│   │       └── [1] fusionRadarSensor (the actual radar)
│   ├── [2] Target 1 (aircraft 1)
│   ├── [3] Target 2 (aircraft 2)
│   └── ...
```

So `scenario.Platforms{1}` is the tower (because `helperCreateScenario3D` added the tower first).

**Important assumption:** This assumes the tower is ALWAYS the first platform. If someone rewrites `helperCreateScenario3D` and adds targets before the tower, this breaks. **This is brittle code.**

**Better approach:**
```matlab
% Find the platform with a radar sensor
radarIdx = find(arrayfun(@(p) ~isempty(p.Sensors), scenario.Platforms), 1);
tower = scenario.Platforms(radarIdx);
radar = tower.Sensors{1};
```

This is defensive programming—it works even if tower isn't first.

---

## Line 13: Restart Scenario

```matlab
restart(scenario);
```

**What it does:**
- Resets the scenario to time=0 with targets at their initial positions
- Clears any simulation history

**Why this matters:**
If you ran `helperRunDetections` twice on the same scenario object without restarting, the second run would start from where the first left off (t=50s), not from the beginning (t=0s). That would be wrong.

**When is this called?**
Every time `helperRunDetections` is called. So you can safely call it multiple times on the same scenario and get consistent results.

---

## Lines 15-18: Initialize Output Structure

```matlab
scanBuffer = {};
dataLog.Time = [];
dataLog.Truth = [];
dataLog.Detections = {};
```

**What it does:**
- `scanBuffer` = temporary storage for detections from one scan (gets reset after each scan)
- `dataLog.Time` = array of timestamps (one per scan)
- `dataLog.Truth` = array of ground-truth positions (one per scan)
- `dataLog.Detections` = cell array of detection lists (one per scan)

**Explanation of "scan":**
A rotating radar produces detections continuously as it sweeps. When it completes one full rotation (or sector rotation), that's one "scan". MATLAB tells us when a scan ends via `config.IsScanDone`.

**Example after running scenario:**
```
dataLog.Time = [0.0, 0.01, 0.02, 0.03, ...]  % seconds
dataLog.Truth = [targets@0.0, targets@0.01, ...]
dataLog.Detections = {
    [det1_scan1, det2_scan1, det3_scan1],
    [det1_scan2, det2_scan2],
    ...
}
```

---

## Lines 20-22: RNG (Random Number Generator) Setup

```matlab
s = rng;
rng(2018);
disp('Please wait. Generating detections for scenario .....')
```

**What it does:**
- `s = rng` = **SAVE the current RNG state** (important!)
- `rng(2018)` = **SET RNG seed to 2018** (fixed seed, so results are repeatable)
- Print a message so the user knows something is happening

**Why save the state?**
If someone calls `helperRunDetections` in the middle of their script, they don't want it to break their random number stream. By saving and restoring the RNG state, we're being a good citizen.

**Why seed with 2018?**
Fixed seed = repeatable results. If the code for degradation uses `rand()` or `randn()`, we want the same degradation every time. This is **crucial for debugging.**

The number 2018 is arbitrary (year the code was written?). Could be any number.

**Later in the code (line 232):**
```matlab
rng(s);
```
This restores the saved state. So after `helperRunDetections` finishes, the caller's RNG is unaffected.

---

# PART 2: THE MAIN LOOP (Lines 24-230)

## Line 24: While Loop - Advance Scenario

```matlab
while advance(scenario)
```

**What it does:**
- `advance(scenario)` steps the scenario forward by one timestep (0.01 seconds at 100 Hz)
- Returns `true` if there's more simulation time left
- Returns `false` when scenario ends
- Loop runs until scenario is exhausted

**Example execution:**
```
Iteration 1: t=0.00s
Iteration 2: t=0.01s
Iteration 3: t=0.02s
...
Iteration 5000: t=50.00s (scenario ends, advance returns false)
```

**Important:** Each iteration is ONE UPDATE (not one scan). A scan typically takes ~10 updates to complete (depending on beam width and update rate).

---

## Lines 25-31: Get Current State & Call Radar

```matlab
simTime = scenario.SimulationTime;
targets = targetPoses(tower);
ins = pose(tower, 'true');
[dets,~,config] = radar(targets,ins,simTime);
dets = dets(:);     % enforce column cell array
nFalse = 0;
```

**Line 25: Current time**
```matlab
simTime = scenario.SimulationTime;
```
Gets the current simulation time (should be 0.00, 0.01, 0.02, ... 50.00).

**Line 26: Ground-truth target poses**
```matlab
targets = targetPoses(tower);
```
Gets the TRUE position and velocity of all aircraft at this moment. This is what the radar is "looking at."

Example output:
```
targets = 
  1x2 array of platformPose:
    [1] Position: [-1500.0, -20000.0, -3000.0], Velocity: [133.3, 0, 0]
    [2] Position: [2000.0, -19000.0, -3500.0], Velocity: [-80.0, 0, 0]
```

**Line 27: Radar inertial measurement system (INS)**
```matlab
ins = pose(tower, 'true');
```
Gets the radar's own pose (position and orientation). Since the tower is stationary at origin, this is always roughly (0, 0, 0).

The radar needs this because it measures detections relative to itself. It needs to know "where am I so I can report detections in scenario coordinates?"

**Line 28-29: Call the radar sensor**
```matlab
[dets,~,config] = radar(targets,ins,simTime);
```

**This is the key line.** What happens here:

1. Pass in ground-truth target positions (`targets`)
2. Pass in radar's own position (`ins`)
3. Pass in current time (`simTime`)
4. MATLAB's `fusionRadarSensor` internally:
   - Checks which targets are in the radar's field of view
   - Checks if targets are within beam resolution
   - Simulates detection (with 80% probability)
   - Adds realistic measurement noise
   - Returns detections

**Output:**
- `dets` = cell array of `objectDetection` objects (what radar "saw")
- `~` = we don't care about this output (SNR or something)
- `config` = struct with metadata, including `config.IsScanDone` (boolean: did scan complete?)

**Example output:**
```
dets = {
  [1] objectDetection with measurement [0.5, -20.1, -3.05] +/- noise
  [2] objectDetection with measurement [2.1, -19.0, -3.48] +/- noise
}
config = struct with IsScanDone = false  (we're still in the middle of the scan)
```

**Line 30: Enforce column cell array**
```matlab
dets = dets(:);
```
MATLAB sometimes returns row cell arrays, sometimes columns. This forces column, so later code doesn't break.

**Line 31: Initialize false detection counter**
```matlab
nFalse = 0;
```
We'll update this later if we add clutter.

---

## Lines 33-52: DEGRADATION LAYER 1 - Per-Update Detection Loss & Noise

This is the "weather" layer. If `enableDegradation=true`, we degrade detections.

```matlab
if enableDegradation
    w = weatherSeverity(simTime);   % 0 (clear) or 1 (storm)

    pdEff = (1-w)*0.95 + w*0.70;    % 95% detection clear, 70% storm
    Rmult = 1 + 2*w;                % 1x noise clear, 3x noise storm

    if ~isempty(dets)
        keep = rand(numel(dets),1) < pdEff;
        dets = dets(keep);
    end

    for i = 1:numel(dets)
        dets{i}.MeasurementNoise = dets{i}.MeasurementNoise * Rmult;
    end
end
```

**What's happening:**

1. **Get weather severity**
   ```matlab
   w = weatherSeverity(simTime);
   ```
   Returns 0 (clear) or 1 (full storm). During t=15-30s, w=1. Otherwise w=0.

2. **Compute effective detection probability**
   ```matlab
   pdEff = (1-w)*0.95 + w*0.70;
   ```
   Linear interpolation:
   - If w=0 (clear): pdEff = 1.0×0.95 + 0.0×0.70 = 0.95 (95% chance to detect)
   - If w=1 (storm): pdEff = 0.0×0.95 + 1.0×0.70 = 0.70 (70% chance to detect)
   
   So in a storm, you lose 25% of detections. Realistic.

3. **Compute noise multiplier**
   ```matlab
   Rmult = 1 + 2*w;
   ```
   Linear interpolation:
   - If w=0 (clear): Rmult = 1 + 0 = 1 (no extra noise)
   - If w=1 (storm): Rmult = 1 + 2 = 3 (3x noise)
   
   So in a storm, measurement noise is 3x worse.

4. **Drop detections randomly**
   ```matlab
   if ~isempty(dets)
       keep = rand(numel(dets),1) < pdEff;
       dets = dets(keep);
   end
   ```
   
   **This is simulation of "missed detections."** If you have 5 detections and pdEff=0.70:
   ```
   rand(5,1) generates [0.34, 0.89, 0.12, 0.76, 0.55]
   keep = [0.34<0.70, 0.89<0.70, 0.12<0.70, 0.76<0.70, 0.55<0.70]
         = [true, false, true, false, true]
   dets = dets([true, false, true, false, true])  % Keep detections 1, 3, 5
   ```
   
   So we drop detections 2 and 4. That's realistic rain behavior—you miss some targets.

5. **Inflate measurement noise**
   ```matlab
   for i = 1:numel(dets)
       dets{i}.MeasurementNoise = dets{i}.MeasurementNoise * Rmult;
   end
   ```
   
   Each remaining detection gets its noise covariance multiplied by Rmult. If a detection had noise σ²=10 (meters²), now it's 30 (in a storm).
   
   **Why?** Rain scatters radar signals, making measurements noisier/less precise.

---

## Lines 54-58: Buffer Detections Until Scan Completes

```matlab
if ~isempty(dets)
    scanBuffer = [scanBuffer; dets];
end

if config.IsScanDone
```

**What's happening:**

Every update (every 0.01 seconds), the radar produces some detections. We accumulate them in `scanBuffer` until a full scan completes.

**Example:**
```
Update 1 (t=0.00): Radar detects 2 targets
  scanBuffer = [det1, det2]
Update 2 (t=0.01): Radar detects 1 target
  scanBuffer = [det1, det2, det3]
Update 3 (t=0.02): Radar detects 2 targets
  scanBuffer = [det1, det2, det3, det4, det5]
...
Update 10 (t=0.09): IsScanDone=true, scan is complete
```

So `scanBuffer` contains all detections from a complete azimuth scan (multiple updates grouped together).

**Why buffer this way?**
Trackers expect a **batch of detections at a single time step**, not individual detections. By buffering, we're mimicking real radar: "Here are all the detections from this scan sweep."

---

## Lines 59-70: Debug Logging (Optional)

```matlab
if config.IsScanDone
    nBefore = numel(scanBuffer);

    if ~isempty(scanBuffer)
        times = cellfun(@(d)d.Time, scanBuffer);
        fprintf("Scan buffer time span (pre-snap) = %.6f s\n", ...
            max(times) - min(times));
    else
        fprintf("Scan buffer is empty at scan end.\n");
    end
```

**What it does:**
- Checks how much time spread is in the scan buffer (should be ~0.01-0.05s)
- If spread is large, something's wrong with the scenario timing

**Example output:**
```
Scan buffer time span (pre-snap) = 0.010000 s
Scan buffer time span (pre-snap) = 0.009999 s
```

This tells you: "All detections in this scan happened within 0.01s of each other, which is correct."

**If you saw:**
```
Scan buffer time span (pre-snap) = 0.500000 s
```

That would be a **red flag.** It means detections are spread over half a second, which shouldn't happen for one scan. Something's broken.

**This is good defensive coding.** It catches bugs early.

---

## Lines 72-117: DEGRADATION LAYER 2 - Clutter Injection (Per-Scan)

This is where we add **false detections (clutter)** to simulate rain/weather causing the radar to see things that aren't there.

```matlab
if enableDegradation
    wScan = weatherSeverity(simTime);

    lambdaFalsePerScan = (1-wScan)*0.0 + wScan*3.0;
    nFalse = poissrnd(lambdaFalsePerScan);

    haveMP = ~isempty(scanBuffer) && ...
             isprop(scanBuffer{1}, 'MeasurementParameters');
    if haveMP
        mp = scanBuffer{1}.MeasurementParameters;
    end

    if wScan > 0
        clutterSigma = 150;
    else
        clutterSigma = 100;
    end
    Rclutter = eye(3) * clutterSigma^2;

    for i = 1:nFalse
        meas = falseMeasInSurveillanceVolume();
        if haveMP
            scanBuffer{end+1,1} = objectDetection(simTime, meas, ...
                'MeasurementNoise', Rclutter, ...
                'SensorIndex', 1, ...
                'MeasurementParameters', mp);
        else
            scanBuffer{end+1,1} = objectDetection(simTime, meas, ...
                'MeasurementNoise', Rclutter, ...
                'SensorIndex', 1);
        end
    end
end
```

**Breaking it down:**

1. **Compute false detections per scan**
   ```matlab
   lambdaFalsePerScan = (1-wScan)*0.0 + wScan*3.0;
   ```
   - Clear (w=0): λ=0 (no clutter)
   - Storm (w=1): λ=3 (average 3 false detections per scan)

2. **Sample from Poisson distribution**
   ```matlab
   nFalse = poissrnd(lambdaFalsePerScan);
   ```
   
   Poisson distribution models count data. With λ=3:
   ```
   One scan: 2 false detections
   Next scan: 5 false detections
   Next scan: 3 false detections
   Next scan: 1 false detection
   ```
   
   Realistic: clutter amount varies scan-to-scan.

3. **Get measurement parameters from real detections**
   ```matlab
   haveMP = ~isempty(scanBuffer) && isprop(scanBuffer{1}, 'MeasurementParameters');
   if haveMP
       mp = scanBuffer{1}.MeasurementParameters;
   end
   ```
   
   If there are real detections, grab their measurement parameters (coordinate frame info, etc.). Use this for clutter so it matches.
   
   **If scanBuffer is empty:** (no real detections this scan) — use default parameters.

4. **Create clutter with large noise**
   ```matlab
   if wScan > 0
       clutterSigma = 150;  % meters (storm)
   else
       clutterSigma = 100;  % meters
   end
   Rclutter = eye(3) * clutterSigma^2;
   ```
   
   Clutter is less reliable than real detections, so noise is huge (100-150m vs. maybe 10m for real).
   
   `eye(3)` creates 3×3 identity, so:
   ```
   Rclutter = [10000    0      0  ]   (in storm: [22500    0      0  ])
              [    0 10000      0  ]                  [    0 22500      0  ]
              [    0    0   10000  ]                  [    0    0   22500  ]
   ```

5. **Generate and add false detections**
   ```matlab
   for i = 1:nFalse
       meas = falseMeasInSurveillanceVolume();
       % Create objectDetection with false measurement
       scanBuffer{end+1,1} = objectDetection(simTime, meas, ...);
   end
   ```
   
   For each false detection:
   - Generate a random measurement in the surveillance volume
   - Wrap it in an `objectDetection` object with huge noise
   - Add it to the scan buffer

---

## Lines 119-123: Force Timestamp Alignment

```matlab
for k = 1:numel(scanBuffer)
    scanBuffer{k}.Time = simTime;
end
```

**What it does:**
All detections in a scan came from different update times (0.00s, 0.01s, 0.02s, etc.). But for tracking, we want them all at the **same time** (scan end time).

**Why?**
JPDA and other trackers assume a batch of detections at time T. If detections are spread over 0.05s, the tracker gets confused.

**Example:**
Before:
```
det1.Time = 0.00s
det2.Time = 0.01s
det3.Time = 0.02s
```

After:
```
det1.Time = 0.02s  (scan end time)
det2.Time = 0.02s
det3.Time = 0.02s
```

Now they're all at the same time step.

---

## Lines 125-129: Region-of-Interest (ROI) Gating

```matlab
scanBuffer = gateDetectionsROI(scanBuffer);

fprintf("t=%.2f: dets=%d (before=%d, clutterAdded=%d)\n", ...
    simTime, numel(scanBuffer), nBefore, nFalse);
```

**What it does:**
- Removes detections outside a 3D bounding box (hardcoded at lines 251-275)
- Keeps the scan clean by discarding implausible detections

**Example:**
```
Before ROI gate: 15 detections
After ROI gate: 12 detections  (3 removed as out-of-region)
```

The helper function (lines 251-275):
```matlab
function detsOut = gateDetectionsROI(detsIn)
    if isempty(detsIn)
        detsOut = detsIn;
        return;
    end

    % ROI bounds (meters)
    xMin = -8000;  xMax =  8000;
    yMin = -26000; yMax = -16000;
    zMin = -8000;  zMax =  500;

    keep = false(numel(detsIn),1);
    for ii = 1:numel(detsIn)
        z = detsIn{ii}.Measurement(:);
        if numel(z) == 2
            z = [z; 0];
        end

        keep(ii) = (z(1) >= xMin && z(1) <= xMax) && ...
                   (z(2) >= yMin && z(2) <= yMax) && ...
                   (z(3) >= zMin && z(3) <= zMax);
    end
    detsOut = detsIn(keep);
end
```

**What this does:**
- Check each detection's [x, y, z] position
- Keep only if inside bounds
- Discard rest

**Example:**
```
Detection 1: [500, -20000, -3000] → Inside bounds ✓ Keep
Detection 2: [50000, -20000, -3000] → Outside (x too large) ✗ Discard
Detection 3: [500, -10000, -3000] → Outside (y too large) ✗ Discard
Detection 4: [500, -20000, 5000] → Outside (z too large) ✗ Discard
```

**Important note:**
These bounds are **hardcoded and should match your scenario geometry.** If your targets are at (-2000 to 1500, -20000 to -18000, -4000 to -3000), these bounds make sense. If scenario changes, bounds need to change too.

**⚠️ This is a weakness:** Bounds should be parameters, not hardcoded.

The debug output:
```matlab
fprintf("t=%.2f: dets=%d (before=%d, clutterAdded=%d)\n", ...
    simTime, numel(scanBuffer), nBefore, nFalse);
```

Prints something like:
```
t=0.03: dets=4 (before=3, clutterAdded=0)
t=0.05: dets=5 (before=5, clutterAdded=0)
t=0.20: dets=8 (before=4, clutterAdded=4)  ← Storm started, added clutter
```

Good for debugging—you see when clutter is injected and how many detections there are.

---

## Lines 131-136: Logging the Scan

```matlab
dataLog.Time = [dataLog.Time, simTime];
dataLog.Truth = [dataLog.Truth, targets];
dataLog.Detections = [dataLog.Detections(:)', {scanBuffer}];

% Reset for next scan
scanBuffer = {};
```

**What it does:**
Appends the current scan to the output log.

**Line 131: Append timestamp**
```matlab
dataLog.Time = [dataLog.Time, simTime];
```

This **creates a new array** every time. Not efficient, but works.

Example progression:
```
Iteration 1: dataLog.Time = [0.03]
Iteration 2: dataLog.Time = [0.03, 0.05]
Iteration 3: dataLog.Time = [0.03, 0.05, 0.07]
```

⚠️ **This is the performance bottleneck I mentioned.** For 100 scans, this is 100 memory allocations. Better to pre-allocate.

**Line 132: Append ground truth**
```matlab
dataLog.Truth = [dataLog.Truth, targets];
```

Same issue. But now you're storing struct arrays of target poses.

**Line 133: Append detections**
```matlab
dataLog.Detections = [dataLog.Detections(:)', {scanBuffer}];
```

Cell array concatenation. The `(:)'` reshapes to row, then adds `{scanBuffer}` as a new cell.

**Line 136: Reset buffer**
```matlab
scanBuffer = {};
```

Empty it for the next scan.

---

## Lines 138-139: Close Loop & Restore RNG

```matlab
end  % closes while advance(scenario) loop

rng(s);
disp('Detections generation complete.')
```

**What it does:**
- Loop ends when scenario is exhausted (advance returns false)
- Restore RNG state so caller's random numbers aren't affected
- Print completion message

---

# PART 3: THE HELPER FUNCTIONS

## weatherSeverity (Lines 243-248)

```matlab
function w = weatherSeverity(t)
    stormStart = 15;   % seconds
    stormEnd   = 30;   % seconds
    w = double(t >= stormStart && t <= stormEnd);
end
```

**What it does:**
Returns 0 (clear) or 1 (storm).

**Timeline:**
```
t=0 to 14:   w=0 (clear)
t=15 to 30:  w=1 (storm)
t=31+:       w=0 (clear again)
```

**The math:**
```matlab
t >= stormStart && t <= stormEnd
```
- Returns logical true/false
- `double()` converts true→1, false→0

So you get w ∈ {0, 1}.

**⚠️ Problem: Binary on/off**
Real weather doesn't switch instantly. Better would be:
```matlab
function w = weatherSeverity(t)
    stormStart = 15;
    stormEnd = 30;
    if t < stormStart || t > stormEnd
        w = 0;
    else
        % Smooth ramp: 0 → 1 → 0
        progress = (t - stormStart) / (stormEnd - stormStart);
        w = sin(progress * pi);  % smooth rise and fall
    end
end
```

This gives:
```
t=15:   w ≈ 0 (just starting)
t=22.5: w ≈ 1 (peak)
t=30:   w ≈ 0 (ending)
```

Much more realistic.

---

## falseMeasInSurveillanceVolume (Lines 250-255)

```matlab
function meas = falseMeasInSurveillanceVolume()
    x = (-1.5e3) + (3.0e3)*rand;     % [-1.5 km, +1.5 km]
    y = (-20.5e3) + (2.0e3)*rand;    % [-20.5 km, -18.5 km]
    z = -3e3 + 700*randn;            % around -3 km
    meas = [x; y; z];
end
```

**What it does:**
Generates a random false detection (clutter) somewhere in the surveillance volume.

**Breaking it down:**

```matlab
x = (-1.5e3) + (3.0e3)*rand;
```
- `rand` returns random number in [0, 1)
- `(3.0e3)*rand` gives [0, 3000)
- `(-1.5e3) + ...` shifts to [-1500, 1500)

So x ∈ [-1500, 1500] meters.

```matlab
y = (-20.5e3) + (2.0e3)*rand;
```
Similarly: y ∈ [-20500, -18500] meters.

```matlab
z = -3e3 + 700*randn;
```
- `randn` returns Gaussian random (mean=0, σ=1)
- `700*randn` gives Gaussian with σ=700
- `(-3e3) + ...` centers at -3000 (3 km altitude)

So z ∈ roughly [-4100, -1900] meters (3σ range).

**Why different distributions?**
- x, y use uniform (rectangular surveillance volume)
- z uses Gaussian (clutter more likely near -3km altitude where targets are)

**Visualization:**
```
Surveillance volume:
  x: -1.5 km to +1.5 km
  y: -20.5 km to -18.5 km
  z: roughly -4 km to -2 km (Gaussian around -3 km)
```

This roughly matches where targets fly in `helperCreateScenario3D`.

---

## gateDetectionsROI (Lines 257-275)

Already covered above. Just a 3D bounding box filter.

---

# COMPLETE DATA FLOW EXAMPLE

Let's trace one complete execution:

```
helperRunDetections(scenario, enableDegradation=true)

t=0.00s (Update 1):
  targets = [aircraft1@t=0, aircraft2@t=0]
  radar() produces 2 detections (radar sees both targets clearly)
  dets = [det1, det2]
  
  enableDegradation=true, w=0 (clear weather):
    pdEff = 0.95, Rmult = 1
    keep = [true, true] (both survive)
    dets = [det1, det2] (unchanged)
  
  scanBuffer = [det1, det2]
  config.IsScanDone = false (still scanning)

t=0.01s (Update 2):
  targets = [aircraft1@t=0.01, aircraft2@t=0.01]
  radar() produces 2 detections
  dets = [det3, det4]
  scanBuffer = [det1, det2, det3, det4]
  config.IsScanDone = false

...continue updates...

t=0.10s (Update 10):
  targets = [aircraft1@t=0.10, aircraft2@t=0.10]
  radar() produces 1 detection
  dets = [det11]
  scanBuffer = [det1, det2, det3, det4, ..., det11]
  config.IsScanDone = true  ← Scan is complete!
  
  nBefore = 11
  
  enableDegradation=true, w=0 (still clear):
    lambdaFalse = 0
    nFalse = 0 (no clutter added)
  
  Timestamp snap:
    All detections now have Time = 0.10s
  
  ROI gate:
    All 11 detections are inside bounds, so all kept
  
  Log scan:
    dataLog.Time(1) = 0.10
    dataLog.Truth(1) = [aircraft1, aircraft2]
    dataLog.Detections{1} = [det1, det2, ..., det11]
  
  scanBuffer = {} (reset for next scan)

t=0.11s (Update 11):
  New scan begins...
  scanBuffer = [det12]
  
... continue until t=50s ...

AFTER LOOP:
  rng(s)  (restore caller's RNG)
  Return dataLog with 500+ scans of detections
```

---

# OPTIMIZATION OPPORTUNITIES

## 1. Array Concatenation (HIGHEST PRIORITY)

**Current (slow):**
```matlab
dataLog.Time = [dataLog.Time, simTime];
dataLog.Truth = [dataLog.Truth, targets];
dataLog.Detections = [dataLog.Detections(:)', {scanBuffer}];
```

For a 50-second scenario at ~10 Hz scan rate = ~500 scans. This does 500 allocations.

**Optimized (fast):**
```matlab
% Before loop
estimatedScans = ceil(scenario.SimulationTime * 10);  % rough estimate
dataLog.Time = zeros(1, estimatedScans);
dataLog.Truth = repmat(struct(), estimatedScans, 1);
dataLog.Detections = cell(1, estimatedScans);
scanIdx = 0;

% Inside loop
scanIdx = scanIdx + 1;
dataLog.Time(scanIdx) = simTime;
dataLog.Truth(scanIdx) = targets;
dataLog.Detections{scanIdx} = scanBuffer;

% After loop
dataLog.Time = dataLog.Time(1:scanIdx);
dataLog.Truth = dataLog.Truth(1:scanIdx);
dataLog.Detections = dataLog.Detections(1:scanIdx);
```

**Speed improvement:** 10-20x faster. Minutes becomes seconds.

---

## 2. weatherSeverity Improvement

**Current:**
```matlab
w = double(t >= 15 && t <= 30);  % Binary
```

**Improved:**
```matlab
function w = weatherSeverity(t)
    if t < 15 || t > 30
        w = 0;
    else
        progress = (t - 15) / 15;  % 0 to 1 to 0
        w = sin(progress * pi);     % smooth
    end
end
```

**Benefit:** More realistic degradation curve.

---

## 3. Parameterize Constants

**Current:**
- ROI bounds hardcoded (lines 262-265)
- Storm times hardcoded (lines 246-247)
- Clutter parameters hardcoded (lines 188-191)
- Detection probability hardcoded (line 87)

**Improved:**
```matlab
function dataLog = helperRunDetections(scenario, enableDegradation, degradationParams)
    if nargin < 3
        degradationParams.stormStart = 15;
        degradationParams.stormEnd = 30;
        degradationParams.pdClear = 0.95;
        degradationParams.pdStorm = 0.70;
        degradationParams.clutterLambda = 3.0;
        % ... etc
    end
end
```

**Benefit:** Reusable across different scenarios.

---

## 4. Make Debug Output Optional

**Current:**
```matlab
fprintf("Scan buffer time span... s\n", ...)  % Always prints
fprintf("t=%.2f: dets=%d ...\n", ...)         % Always prints
```

**Improved:**
```matlab
function dataLog = helperRunDetections(scenario, enableDegradation, varargin)
    p = inputParser;
    addParameter(p, 'Verbose', true);  % Add flag
    parse(p, varargin{:});
    verbose = p.Results.Verbose;
    
    % Then use:
    if verbose
        fprintf(...)
    end
end
```

**Benefit:** Cleaner output when running multiple tests.

---

## 5. Defensive Tower/Radar Extraction

**Current:**
```matlab
tower = scenario.Platforms{1};  % Assumes tower is first
```

**Improved:**
```matlab
radarIdx = find(arrayfun(@(p) ~isempty(p.Sensors), scenario.Platforms), 1);
if isempty(radarIdx)
    error('No platform with sensors found in scenario');
end
tower = scenario.Platforms(radarIdx);
```

**Benefit:** Works even if scenario structure changes.

---

# SUMMARY TABLE

| Aspect | Status | Comment |
|--------|--------|---------|
| **Overall logic** | ✅ Correct | Scan buffering, degradation, logging all work |
| **Array concatenation** | ⚠️ Slow | Pre-allocate for speed |
| **weatherSeverity** | ⚠️ Too simple | Binary on/off instead of smooth ramp |
| **Hardcoded parameters** | ⚠️ Inflexible | ROI, storm times, clutter params should be inputs |
| **Debug output** | ✅ Good | Helps troubleshoot, but could be optional |
| **RNG handling** | ✅ Correct | Saved/restored properly for reproducibility |
| **Defensive coding** | ⚠️ Some gaps | Assumes platform order, no error checks |

