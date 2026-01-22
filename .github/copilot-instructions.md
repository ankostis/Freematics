# JRCMATICS Copilot Instructions

## Project Overview

ESP32-based vehicle telemetry (OBD/GPS/MEMS) on Freematics ONE+ hardware. Main firmware in [firmware_v5/telelogger](firmware_v5/telelogger), shared libs in [libraries/](libraries/).

## Build & Setup

- **IDE**: VSCode + PlatformIO extension, open [firmware_v5/telelogger](firmware_v5/telelogger) folder
- **Sparse checkout**: Clone with `--sparse`, use `git sparse-checkout set --cone server libraries/{FreematicsPlus,httpd,TinyGPS,Utils} firmware_v5/{telelogger,sandbox}` (500MB → 45MB)
- **Config pattern**: [config.h](firmware_v5/telelogger/config.h) defines defaults, `secrets.h` overlays credentials (git-ignored)
  - CONFIG_ prefixed macros from platformio.ini take precedence

## Architecture

- **State machine**: [telelogger.cpp](firmware_v5/telelogger/telelogger.cpp) `State` class with bitflags (STATE_OBD_READY, STATE_GPS_READY, STATE_NET_READY, etc)
- **Key classes**: `OBD` (extends COBD), `MEMS_I2C`, `TeleClientUDP`/`TeleClientHTTP`, `SDLogger`/`SPIFFSLogger`
- **Init sequence**: Hardware detect → OBD → GPS → MEMS → Storage → Network → Server
- **Data flow**: Tiered PID poll → Circular buffer (PSRAM/IRAM) → Batch transmit → Retry on fail
- **Network**: WiFi priority, cellular fallback via [FreematicsNetwork.cpp](libraries/FreematicsPlus/FreematicsNetwork.cpp) `CellSIMCOM` class

## Modem-Specific (libraries/FreematicsPlus/FreematicsNetwork.cpp)

- **SIM7600/SIM5360**: Use queryIP() for DNS before connection
- **SIM7070G**: most recent & different

## Feature Flags (config.h)

Conditional compilation via ENABLE_OBD, ENABLE_MEMS, ENABLE_WIFI, ENABLE_BLE, ENABLE_HTTPD, STORAGE (NONE/SPIFFS/SD), GNSS (NONE/STANDALONE/CELLULAR), SERVER_PROTOCOL (UDP/HTTPS_GET/HTTPS_POST). Check `#if ENABLE_*` before modifying dependent code.

## SUMMARY.md - Git Commit Message Draft

Maintain in repo root as working doc for commits:
- Clear when describing new changes (ask if unsure)
- Document staged/uncommitted changes (specify which)
- Use `git diff --staged` to check index (not `get_changed_files` API)
- Lines <90 chars, concise, suitable for git commit

Header line (1st line):
- Syntax: `type(scope) phrase` or `type: phrase` (skip `:` if scope present)
- Types: feat/enh/refact/drop/chore/doc/style, lowercase scope
- Scope can be `tests`/`TCs` (no `test` type exists)
- <50ish chars, end with `>` if body follows
- Separate distinct changes with `;`
- Loosely follow "conventional commits"

Body (after header):
- Simple/direct tone: present tense (changes), past tense (bugs/old behavior)
- Big picture first, be succinct, use "etc" freely
- What changed and why (skip file listings, reviewer sees diffs)
- NO markdown headers (#), confuses vim rebase-interactive
- Backticks for code, italics for concepts, bold for emphasis
- Bullet points for actions/todos, paragraphs for explanations
- TODOs/performance/test changes at bottom if present
- Include "Why" only when non-obvious (workarounds, arch decisions, timing)
