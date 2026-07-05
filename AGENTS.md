# AGENTS.md

Guidance for AI coding agents (and human contributors) working on this
repository. It is tool-agnostic: any agent that understands Markdown can
use it.

## Project overview

`robstride_driver` is a ROS-independent C++20 / CMake library that drives
[RobStride](https://www.robstride.com/) quasi-direct-drive motors over
Linux SocketCAN or the official RobStride USB-CAN module (AT serial
framing). See [docs/architecture.md](docs/architecture.md) for the
layering and class responsibilities and
[docs/protocol.md](docs/protocol.md) for the CAN protocol.

## Repository layout

| Path | Contents |
|------|----------|
| `include/robstride_driver/` | Public headers (the installed API) |
| `src/` | Library implementation |
| `tests/` | GoogleTest unit tests (no hardware required) |
| `examples/` | CLI examples (`velocity_control`, `tracking_capture`) |
| `tools/` | Python helper scripts (plotting) |
| `docs/` | Hardware setup, protocol, architecture, test results |
| `third_party/Product_Information` | Official RobStride documentation (git submodule, reference only — never edit) |

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

GoogleTest is required for the unit tests (`sudo apt install libgtest-dev`).
The `third_party/` submodule is not needed to build; do not initialize it
unless you need the vendor documentation.

## Code style and linting

- C++: Google C++ Style, enforced by `.clang-format` and `.clang-tidy`.
- Python (`tools/`): linted and formatted with ruff (`ruff.toml`).
- Every source file starts with a copyright line and an
  `SPDX-License-Identifier: MIT` comment.
- Public headers document every declaration with `///` comments.

Run the checks the same way CI does:

```bash
clang-format --dry-run --Werror src/*.cpp include/robstride_driver/*.hpp tests/*.cpp examples/*.cpp
clang-tidy -p build src/*.cpp examples/*.cpp tests/*.cpp
ruff check tools/ && ruff format --check tools/
```

Or via pre-commit: `pre-commit run --all-files`.

## Verification checklist for changes

1. The build succeeds with `-Wall -Wextra -Wpedantic` (default flags).
2. `ctest` passes; add or update unit tests for any behavior change.
   Protocol and motor logic must stay testable without hardware — keep
   transports behind the `CanInterface` abstraction.
3. clang-format, clang-tidy and ruff report no issues.
4. Update the relevant `docs/` page when behavior, protocol handling or
   hardware assumptions change.

Hardware-in-the-loop behavior (tracking performance, real CAN buses) is
**not** covered by CI; [docs/test_results.md](docs/test_results.md)
documents how those results were measured on a real RS02.

## Conventions

- Commit messages follow the conventional-commit style used in the log
  (`feat:`, `fix:`, `docs:`, `style:`, `refactor:`, `chore:`, `ci:`).
- Keep the library free of ROS (and other framework) dependencies;
  ROS integration lives in downstream packages.
- New public API must be usable through `robstride_driver.hpp` and
  documented in the README when user-facing.
