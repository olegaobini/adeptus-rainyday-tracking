# Contributing to Rainy Day

## Workflow — please follow this

**Don't commit directly to `main`.** Always create your own branch first.

**In GitHub Desktop:**
1. Switch to `main` (click the branch dropdown at the top)
2. Click the dropdown again → **New branch**
3. Name it something descriptive, like `daniel-gobutton` or `james-ui-cleanup`
4. Commit and push on that branch

When your change is ready, open a **Pull Request** on GitHub so it can be reviewed before merging to `main`.

## Why this matters

`main` is the presentation-ready / deliverable version of the project. Direct commits to it make it impossible to track who changed what, can break the simulation for everyone else without warning, and can overwrite work in progress. A 30-second PR lets the change be sanity-checked before it affects the deliverable version. This is how every real software team works — it's not arbitrary.

## Getting started with the code

- **`QUICKSTART.md`** — one-page reference with every command you'll need
- **`README.md`** — full architecture, data flow, and feature list
- **`Cheatsheet.txt`** — long-form command reference

## Before you open a PR

- Run `runTestPlan` in MATLAB — if something was passing before your change and breaks now, fix it before asking for review
- Keep your PR focused on one thing (one feature, one bug fix) — easier to review and easier to revert if needed
- Write a clear PR description: what you changed and what you tested

## Questions

Ping Michael on the team chat before starting anything non-trivial — it's almost always faster than guessing at how a component fits in, and it'll save you from wasted work if something similar already exists.
