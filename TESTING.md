# Testing the Rainy Day Tracker

**For teammates helping with installer-based testing.** No MATLAB needed.

---

## What you'll need

- Windows 10 or 11 (x64)
- ~3 GB free disk space
- Internet connection during install (to download the MATLAB Runtime)
- ~10 minutes for first-time install

That's it. You do **not** need MATLAB, MATLAB Compiler, or any toolboxes.

---

## Install

1. Get **`RainyDayTrackerInstaller_web.exe`** from the repo:  
   `adeptus-rainyday-tracking/installer/web/RainyDayTrackerInstaller_web.exe`

   Easiest way: open the repo on GitHub, click into that folder, click the
   `.exe` file, then click the **Download raw file** button (the small
   download icon near the top right of the file view). Save it anywhere
   convenient — Downloads is fine.

   If you have the repo cloned locally, you already have the file at the
   path above; no download needed.

2. Double-click the `.exe` to run the installer. Click through the prompts
   (defaults are fine).
3. The installer will:
   - Copy the app to your **Program Files** folder.
   - Download MATLAB Runtime R2025b (~2 GB, one-time).
   - Add a **Rainy Day Tracker** shortcut to your Start Menu.
4. Click **Finish** when it's done.

> **Heads-up:** Windows Defender SmartScreen may flag the installer the first time
> (it's an unsigned binary from a private team, not from a known publisher).
> Click **More info → Run anyway**.

---

## First launch

1. Open the Start menu, type `Rainy Day Tracker`, press Enter.
2. A small console window may flash — that's normal.
3. The **3-button main menu** opens:
   - 1. Path Editor / Scenario Builder
   - 2. Run Simulation
   - 3. Validation & Documentation

On first launch the app silently creates a per-user data folder at:

```
C:\Users\<your-username>\AppData\Local\RainyDay\
```

This is where your edits, cached detections, and saved results live. You don't
need to interact with it directly — but if anything gets weird and you want a
clean slate, see **Reset** below.

---

## What to test

Click each button and confirm it does what's described. Report anything that
errors, looks wrong, or confused you.

### Button 1 — Path Editor / Scenario Builder
Window should open showing a 2D map area on the left and panels on the right
(Targets / Sensors / Environment toggle). Try:
- Click the Targets toggle → click a few points on the map → see waypoints appear.
- Switch to Sensors mode → switch to Environment mode → confirm panels swap.
- Close the window — it should close cleanly without errors.

### Button 2 — Run Simulation
Window opens with a run-file dropdown. Try:
- Pick **`dasr_baseline`** from the dropdown.
- Click **Run**. Expect:
  - Console output describing the run plan.
  - 3D plots open (initial scenario, then tracker results).
  - A summary table in the console with track swaps + position RMS per tracker.
- Try a second run with **`demo_multi_4_rain`** (heavy rain demo).
- Close the window cleanly when done.

### Button 3 — Validation & Documentation
Three tabs: Test Plan / Diagnostic Suite / Documentation.
- **Test Plan tab:** click Run. Should show **27/27 PASS** in the inline grid.
  Takes a few minutes.
- **Documentation tab:** click each link. The PDF / README should open in your
  default app.

---

## What to report back

Send Michael a quick note covering:

- ✅ What worked
- ❌ What didn't (screenshot any error pop-ups + the console window if visible)
- 😕 Anything that confused you or where the UI behaved unexpectedly
- 🐢 Anything that felt unusably slow

Don't worry about being polished — bullet points in chat are perfect.

---

## Reset (clean slate)

If something gets stuck or you want to test the first-launch experience again:

1. Close the app completely.
2. Delete the folder `C:\Users\<your-username>\AppData\Local\RainyDay\`
   (paste `%LOCALAPPDATA%\RainyDay` into File Explorer's address bar to find it).
3. Relaunch from the Start menu — it'll re-seed from a clean install.

You don't need to reinstall the app to reset; just delete the user data folder.

---

## Uninstall

Settings → Apps → search "Rainy Day Tracker" → Uninstall. Or use the entry in
Apps & Features. The MATLAB Runtime stays installed (it's reusable across
MathWorks-built apps); uninstall that separately if you need to free space.

---

## Source code (optional)

You have the same git access Michael does, so the source is available if you
want to look. But for testing purposes you don't need to clone or open
anything — the installer is self-contained. Code-side workflow lives in
`QUICKSTART.md` and `README.md` for whenever you want to dig in.
