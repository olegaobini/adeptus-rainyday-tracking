[33mcommit e9d41b8a8f48f7c376952dcf610bf3db694e5f21[m[33m ([m[1;36mHEAD[m[33m -> [m[1;32mMichael-Adding-Globe-View-to-menu[m[33m)[m
Author: Hardimic <mjh49@uw.edu>
Date:   Wed May 27 19:03:28 2026 -0700

    ﻿v3.7.5 - Sensor coverage volume rendering (Site 1) + underground-detection viz filter
    
    - NEW src/+trackbench/+reporting/computeSensorCoverageVolume.m: closed
      swept-volume mesh per cov struct (one element of dataLog.SensorCoverage).
      Returns patch-ready [V, F] in world-frame NED meters. Outer (az,el)
      shell + top/bottom (az,r) cones + (sector only) (el,r) side walls.
      Default 30 az x 10 el sampling. R2025b doc-anchored: MountingAngles=
      [z-yaw,y-pitch,x-roll]; RangeLimits Inf-handling cap at 120km per
      drawBeamEnvelope.m:47 precedent.
    
    - src/+trackbench/+reporting/plotInitialScenario.m:
      * Site 1 integration (lines 41-48): replaces drawBeamEnvelope call
        with per-sensor loop patching computeSensorCoverageVolume output.
        Colors mirror drawSensorCoverage.m v3.6.15 (FROZEN): PSR blue,
        IR magenta, sector radar green. MSSR skipped. FaceAlpha=0.18.
      * Underground-detection viz filter at detection scatter sites (line
        147 cell-array branch, line 157 struct-array branch): one-line
        guard 'if meas(3) <= 1' before addpoints. Drops detections >1m
        below ground in NED (display altitude < -1m). Mitigates HasINS=
        false body-frame visualization artifact made visible by new
        coverage-volume reference frame. Root cause investigation deferred
        to v3.7.4 BUGHUNT queue (three candidates filed).
    
    - src/+trackbench/+reporting/drawBeamEnvelope.m: deprecation header.
      Fallback only; no longer called from primary plotting path.
    
    Empirical (Michael IDE 2026-05-27):
      PosterDemo bit-identical to v3.7.3: T1=1233.1m T2=340.0m T3=2043.7m
      T4=544.9m T5=4932.4m T6=1760.7m T7=1654.7m Avg=1787.1m Tracked%=
      98/98/73 Swaps=CLEAN Truth3-est-fail=27% canary t=4.80:Primary=8
      clutter=3.
      test_IRST: Tracked%=96% (TruID 1 -> AssocTrk 8), 23 scans.
    
    Visual gates passed: PSR blue swept volume aligned with existing ground
    ring; IRST magenta volume; tracker plot (Site 2, BUGHUNT-owned)
    intentionally unchanged.
    
    Reminder: clear classes; clear all; rehash after +reporting/ edits.

 .../+reporting/computeSensorCoverageVolume.m       | 238 [32m+++++++++++++++++++++[m
 .../src/+trackbench/+reporting/drawBeamEnvelope.m  |   6 [32m+[m
 .../+trackbench/+reporting/plotInitialScenario.m   |  49 [32m++++[m[31m-[m
 3 files changed, 288 insertions(+), 5 deletions(-)
