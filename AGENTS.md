# Repository Guidelines

## Project Structure & Module Organization
Firmware lives in `Software`: `Software.ino` ties the Arduino lifecycle together and `src/` hosts focused modules (`battery`, `charger`, `communication`, `datalayer`, `inverter`, `devboard`). Shared defaults sit in `system_settings.h`, while user-tunable flags, pins, and feature toggles belong in `USER_SETTINGS.{h,cpp}`. Secrets (Wi-Fi, MQTT, API tokens) are copied from `USER_SECRETS.TEMPLATE.h` into the untracked `USER_SECRETS.h`.

## Build, Test, and Development Commands
- `pio run` — build the firmware with the PlatformIO toolchain.
- `pio run -t upload --upload-port /dev/ttyUSB0` — flash the LilyGo board.
- `pio device monitor -b 115200` — stream logs at the firmware baud rate.
- `cmake -S . -B build && cmake --build build` — compile host-only unit tests in `test/utils`.
- `./build/events_test` — execute individual µtests built above.
- `pre-commit run --all-files` — enforce clang-format, licence, and whitespace hooks.

## Coding Style & Naming Conventions
Formatting is enforced by `.clang-format` (Google style, 2-space indent, 120-column max, left-aligned pointers, same-line braces). Run `clang-format -i <file>` or let `pre-commit` fix violations. Use snake_case for functions/variables, PascalCase for structs/classes, and ALL_CAPS for macros or config constants. Keep Arduino globals in `include.h` and isolate hardware abstractions under `devboard/` so they remain mockable.

## Testing Guidelines
Unit tests rely on the lightweight harness in `test/test_lib.{h,cpp}` plus the `microtest` macros. Place host tests under `test/utils`, name files `<feature>_test.cpp`, and include the real headers they exercise so compilation mirrors firmware builds. Tests compile with `UNIT_TEST` defined, so gate hardware-only paths behind `#ifndef UNIT_TEST`. When you touch battery or inverter logic, add the manual verification evidence (serial traces, inverter screenshots) to the PR alongside the automated test results.

## Commit & Pull Request Guidelines
Commits typically use short typed prefixes such as `bugfix/<scope>` or `improvement/<topic>` (see `bugfix/SOC-CMFA`, `improvement/event-stop`). Follow `<type>/<scope>: summary`, keep each commit focused, and include matching tests/config edits. Pull requests should ship a problem statement, a bullet list of changes, test evidence (`events_test` output, PlatformIO build logs), any `USER_SETTINGS` diffs, and links to issues or discussions. Attach screenshots or serial logs for inverter/battery behavior and ensure CI formatting passes before review.

## Configuration & Security Tips
Never commit populated `USER_SECRETS.h`; keep credentials in local overrides or environment variables. Note wiring or inverter assumptions inside `USER_SETTINGS.h` comments so installers can reproduce your setup. When toggling options such as `FUNCTION_TIME_MEASUREMENT`, document the default, test both states, and ensure the fallback path remains safe for downstream builds.
