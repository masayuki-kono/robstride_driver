---
name: release
description: >-
  Release procedure for robstride_driver (CHANGELOG, version bump, PR, tag).
  Use when cutting a new release (e.g. 'release v0.2.0'), updating CHANGELOG.md,
  or tagging a version.
---

# Release Guide

Step-by-step procedure for releasing `robstride_driver` version `vX.Y.Z`.
The agent performs every step except merging the release PR, which may
require a human review depending on branch protection.

Versioning follows [Semantic Versioning](https://semver.org/): bump MAJOR
for breaking API changes, MINOR for new backwards-compatible features,
PATCH for fixes only.

**Repository:** `masayuki-kono/robstride_driver`. Use **GitHub MCP**
(`user-github`) for PR operations and local `git` for branch / commit /
tag / push. No `gh` CLI is required.

## 1. Preconditions (abort the release if any fails)

```bash
git checkout main && git pull origin main
git status --porcelain          # must be empty
```

Confirm recent CI on `main` is green via the GitHub Actions UI (CI / Lint),
or after opening the release PR use MCP `get_pull_request_status`.

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
```

Create the PR via GitHub MCP `create_pull_request`:

| Arg | Value |
|-----|--------|
| `owner` | `masayuki-kono` |
| `repo` | `robstride_driver` |
| `title` | `chore(release): vX.Y.Z` |
| `head` | `release/vX.Y.Z` |
| `base` | `main` |
| `body` | Short release notes (link CHANGELOG section) |

Poll CI with `get_pull_request_status` (`owner` / `repo` / `pull_number`).
Optional: `get_pull_request` / `get_pull_request_files` for review context.

Wait for CI to pass. Merging may require human review (CodeRabbit /
branch protection). Prefer human merge unless the user explicitly asks to
merge; if using MCP, `merge_pull_request` only after CI is green and the
user approves. If the agent cannot merge, report the PR URL and resume
from step 6 after the merge.

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
Confirm in the GitHub UI (Actions → Release workflow, then the Releases
page). This MCP does not expose Actions or Releases APIs.

## 8. Cleanup

```bash
git branch -d release/vX.Y.Z
```
