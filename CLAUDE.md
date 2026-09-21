# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`cflib` is the Python library for communicating with Bitcraze Crazyflie quadcopters. It is the driver layer used by the Crazyflie PC client and user scripts — it handles the radio/USB link, the CRTP packet protocol, and higher-level abstractions (logging, parameters, commander, memory, high-level commander, swarms, positioning).

Python ≥ 3.10 is required. Build system: setuptools + setuptools_scm (version is derived from git tags).

## Common commands

Install in editable mode for development:
```
pip install -e .
```

Optional extras: `pip install -e '.[dev]'`, `.[qualisys]`, `.[motioncapture]`.

Run the full unit test suite:
```
python3 -m unittest discover ./test
```

Run a single test module / class / method:
```
python3 -m unittest test.crazyflie.test_param
python3 -m unittest test.crazyflie.test_param.ParamTest.test_some_case
```

Run the project's full local CI (pre-commit + tests + build) the same way the Bitcraze CI does:
```
python3 tools/build/build
```

Individual steps of that pipeline:
```
python3 tools/build/verify   # pre-commit run --all-files (lint)
python3 tools/build/test     # unittest discover ./test
python3 tools/build/bdist    # build the wheel
```

The tox entry point wraps the same flow with coverage:
```
tox            # coverage + unittest + pre-commit
make test      # delegates to tox
```

Lint style is enforced by pre-commit (flake8, max line length 120 — see `tox.ini`).

## Architecture

The library is organized as layers; most user code enters at the top layer and never touches the ones below.

### Transport layer — `cflib/crtp/`
CRTP is Crazyflie's packet protocol. Each transport is a `CRTPDriver` subclass registered in `cflib/crtp/__init__.py::init_drivers()`:
- `RadioDriver` (Crazyradio USB dongle, `radio://…` URIs) — the default
- `UsbDriver` (direct USB, `usb://…`)
- `SerialDriver`, `TcpDriver`, `UdpDriver`, `PrrtDriver` — opt-in
- `CfLinkCppDriver` — enabled by setting env var `USE_CFLINK=cpp` (replaces the default radio/usb drivers with the C++ link implementation)

Drivers are discovered via URI scheme. `cflib.crtp.scan_interfaces()` / `init_drivers()` must be called before opening a link.

### Core device layer — `cflib/crazyflie/`
`Crazyflie` (in `__init__.py`) is the central object. On connect it instantiates and wires together sub-services that are each responsible for one CRTP port:
- `Commander` / `HighLevelCommander` — setpoints and trajectory execution
- `Log` — asynchronous logging of onboard variables (TOC-based)
- `Param` — read/write onboard parameters (TOC-based, cached via `TocCache` / `toccache.py`)
- `Memory` (`mem/`) — onboard memory subsystems (LED ring, trajectories, lighthouse geo, etc.)
- `Console`, `Localization`, `PlatformService`, `Appchannel`, `LinkStatistics`, `Extpos`

State transitions (`State.DISCONNECTED → INITIALIZED → CONNECTED → SETUP_FINISHED`) are signalled via `Caller` callback objects (`cflib/utils/callbacks.py`); almost all communication in the lib is asynchronous / callback-driven.

`SyncCrazyflie` and `SyncLogger` wrap the async callbacks in blocking context-manager APIs — most example scripts and tests use these.

`Swarm` coordinates multiple `SyncCrazyflie` instances in parallel.

### Higher-level helpers
- `cflib/positioning/` — `MotionCommander` (body-frame velocity control) and `PositionHlCommander` (high-level setpoint helper) built on top of `Commander` / `HighLevelCommander`.
- `cflib/localization/` — lighthouse geometry estimation and related tools.
- `cflib/bootloader/` — firmware flashing over CRTP.
- `cflib/cpx/` — CPX protocol used to talk to the AI-deck / GAP8.
- `cflib/drivers/` — low-level USB + Crazyradio device drivers used by the CRTP radio/usb drivers.

### Tests — `test/`
Plain `unittest` layout mirroring the package tree (`test/crazyflie/`, `test/crtp/`, `test/positioning/`, `test/localization/`, `test/utils/`). `test/support/` holds fakes and helpers — tests do not touch real hardware. `sys_test/` contains hardware-in-the-loop system tests that are not part of the default unit test run.

### Examples — `examples/`
Runnable scripts grouped by feature area (`logging/`, `positioning/`, `swarm/`, `mocap/`, `lighthouse/`, `flowdeck/`, …). They are the canonical usage reference for the public API and typically start with `cflib.crtp.init_drivers()` + `SyncCrazyflie(uri)`.
