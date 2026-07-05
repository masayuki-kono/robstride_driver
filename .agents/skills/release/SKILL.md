---
name: release
description: "Release procedure for robstride_driver. USE WHEN: cutting a new release (e.g. 'release v0.2.0'), updating CHANGELOG.md, or tagging a version."
---

# Release Guide

Step-by-step procedure for releasing `robstride_driver` version `vX.Y.Z`.
The agent performs every step except merging the release PR, which may
require a human review depending on branch protection.

Versioning follows [Semantic Versioning](https://semver.org/): bump MAJOR
for breaking API changes, MINOR for new backwards-compatible features,
PATCH for fixes only.

## 1. Preconditions (abort the release if any fails)

```bash
git checkout main && git pull origin main
git status --porcelain          # must be empty
gh run list --branch main -L 5  # latest CI and Lint runs must be green
```

Build and test locally:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
pre-commit run --all-files
```

## 2. Release branch

```bash
git checkout -b release/vX.Y.Z
```

## 3. Update version numbers

The version is written in **two places**; both must be updated and must
match, otherwise abort:

- `CMakeLists.txt` — `project(robstride_driver VERSION X.Y.Z ...)`
- `package.xml` — `<version>X.Y.Z</version>`

## 4. Update CHANGELOG.md

`CHANGELOG.md` follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

1. Collect the changes since the previous tag:
   `git log --oneline --no-merges <prev-tag>..HEAD`
2. Move the relevant entries from `[Unreleased]` into a new
   `## [X.Y.Z] - YYYY-MM-DD` section, grouped under
   `Added` / `Changed` / `Fixed` / `Removed` etc.
   Write entries for users of the library, not a raw commit dump.
3. Keep an empty `[Unreleased]` section at the top.
4. Update the link definitions at the bottom of the file.

## 5. Commit, push, PR

```bash
git add CHANGELOG.md CMakeLists.txt package.xml
git commit -m "chore(release): vX.Y.Z"
git push -u origin release/vX.Y.Z
gh pr create --title "chore(release): vX.Y.Z" --base main
```

Wait for CI to pass. Merging may require human review (CodeRabbit /
branch protection); if the agent cannot merge, report the PR URL and
resume from step 6 after the merge.

## 6. Tag (after the PR is merged)

Always tag the merged `main` HEAD, never the release branch (squash
merges rewrite the commit hash):

```bash
git checkout main && git pull origin main
git tag -a vX.Y.Z -m "robstride_driver vX.Y.Z"
git push origin vX.Y.Z
```

## 7. GitHub Release (automated)

Pushing the tag triggers `.github/workflows/release.yml`, which extracts
the `[X.Y.Z]` section from `CHANGELOG.md` and creates the GitHub Release.
Verify:

```bash
gh run watch $(gh run list --workflow=release.yml -L 1 --json databaseId -q '.[0].databaseId')
gh release view vX.Y.Z
```

## 8. Cleanup

```bash
git branch -d release/vX.Y.Z
```
