# Rainy Day — Radar Tracking Simulation

**Version 3.5.1** — Boeing-sponsored senior capstone project, University of Washington.

A modular MATLAB framework for evaluating radar target tracking performance
under real-world degraded conditions (weather, terrain, sensor limits).

---

## 👤 Just here to test the app? → **[`TESTING.md`](TESTING.md)**

One-page guide. Download the .exe from this repo, install it, run the GUI,
send feedback. No MATLAB needed.

The installer is right here in the repo at:  
`adeptus-rainyday-tracking/installer/web/RainyDayTrackerInstaller_web.exe`

---

## 👩‍💻 Working on the code?

Full developer documentation lives in the project subfolder:

- **[`adeptus-rainyday-tracking/README.md`](adeptus-rainyday-tracking/README.md)** — architecture, data flow, sensor types, tracker algorithms, environment modeling, validation suite, change log.
- **[`adeptus-rainyday-tracking/QUICKSTART.md`](adeptus-rainyday-tracking/QUICKSTART.md)** — one-page MATLAB command reference for daily use.
- **[`CONTRIBUTING.md`](CONTRIBUTING.md)** — git workflow (branches + PRs, please don't commit on `main`).

Quickest path to a working dev environment:

1. Install **MATLAB R2025b** with Sensor Fusion & Tracking Toolbox, Radar Toolbox, and Mapping Toolbox.
2. Clone this repo to a **non-OneDrive** path (OneDrive's sync layer interferes with MATLAB Compiler builds).
3. In MATLAB:
   ```matlab
   cd <repo>\adeptus-rainyday-tracking
   addpath("scripts"); addpath(genpath("src"));
   runTestPlan        % expect 27/27 PASS
   ```

---

## Repository layout

```
.
├── adeptus-rainyday-tracking/   ← MATLAB project (source + configs + docs)
├── Tail_687_1/                  ← NASA DASHlink flight data (3 FDR .mat files)
├── TESTING.md                   ← installer-only testing guide for teammates
├── README.md                    ← this file (top-level overview)
├── QUICKSTART.md                ← MATLAB command reference (dev workflow)
├── CONTRIBUTING.md              ← git workflow
└── Cheatsheet.txt               ← long-form command reference
```

Both `adeptus-rainyday-tracking/` and `Tail_687_1/` must stay siblings at the
repo root — the MATLAB project resolves the NASA data folder as a sibling at
runtime, and the demos referencing real flight data will break otherwise.

The 3 NASA flight files are whitelisted in `.gitignore` and ship with the
clone (~7.8 MB total). Additional Tail-687 flights are publicly available
from <https://c3.ndc.nasa.gov/dashlink/resources/664/>.

---

## Building & distributing the installer

Two flavors exist; the **web** flavor is the default we ship to teammates.

| Flavor   | Size  | Internet at install? | Use case                            |
|----------|-------|----------------------|-------------------------------------|
| `web`    | ~5 MB | yes (downloads MCR)  | Default — distribute via team chat  |
| `offline`| ~1.5 GB | no                 | Air-gapped / demo machines only     |

To rebuild after editing source:

```matlab
addpath("scripts"); addpath(genpath("src"));
build_executable                 % bake updated source into mainMenu.exe (3–7 min)
build_installer('web', 'OutputDir', 'C:\Users\Admin\Documents\RainyDay_Installer')
```

The `OutputDir` override is required if your repo lives on OneDrive — the
installer build's internal file ops choke on OneDrive's sync hooks.

Both `installer/` and `trackbench/` are mostly gitignored — the one
exception is `installer/web/RainyDayTrackerInstaller_web.exe` (~5 MB), which
is committed so testing teammates can grab it directly from GitHub. The
offline installer (1.5 GB) and other build artifacts stay out of git.

---

## License

Boeing Proprietary.
