# Documentation Home

Welcome. This is the full reference for the Kinova Gen3 Lite middleware, and for the pipeline it sits in (an LLM proxy talks to this middleware over ROS 2 to move a real or simulated robot arm).

If you read nothing else, read these two pages:

1. [Overview](overview.md): what this system actually is and how a command travels through it, one page, plain English.
2. [Install the middleware and proxy](installation/middleware-and-proxy.md): get both halves of the pipeline built and sourced.

## Pick your path

- **I'm brand new to this project:** Start with [Overview](overview.md), then [Installation](installation/middleware-and-proxy.md). You do not need prior ROS 2 experience, both pages explain concepts as they come up.
- **I just want to get it running:** [Installation](installation/middleware-and-proxy.md), then [Quickstart](user-guide/quickstart.md).
- **I need to add a new object, movement, or recipe:** [Configuration](installation/configuration.md) covers the coordinate dictionary, relative movements, obstacles and recipes.
- **I'm writing or running tests:** [Testing](technical/testing.md) covers the middleware's test suites and the proxy's LLM-output test suite, and is honest about what neither one currently catches.
- **Something's broken:** [Troubleshooting](user-guide/troubleshooting.md) is organized by symptom, with the likely cause and the fix, split by which half of the pipeline it's in.
- **I want to understand how it actually works:** [Architecture](technical/architecture.md) is the deep dive, per-node internals, the full pipeline diagram, every custom service and message type, and the package layout.
- **I want the formal guarantees and safety model:** [Safety and Contracts](technical/safety-and-contracts.md) covers the execution contract, failure modes, and safety layers this middleware is designed against.

## The whole map

Pages marked *draft* are placeholders that don't have content yet.

### Home

| Doc | What it gives you |
|---|---|
| [Overview](overview.md) | What the system is, the request path end to end, the four middleware nodes, the proxy's two domains |

### User guide

| Doc | What it gives you |
|---|---|
| [Quickstart](user-guide/quickstart.md) | Starting the middleware and the proxy, running the middleware alone, watching live telemetry |
| [Using the TUI](user-guide/tui.md) | *Draft.* Typing commands into the terminal UI |
| [Command reference](user-guide/command-reference.md) | *Draft.* The actions, objects and movements you can ask for |
| [Troubleshooting](user-guide/troubleshooting.md) | Symptom-first tables across build errors, recipe errors, motion errors, telemetry, and proxy/LLM connection issues |
| [Demos](user-guide/demos.md) | Short recordings of each feature in action |

### Installation

| Doc | What it gives you |
|---|---|
| [Requirements](installation/requirements.md) | *Draft.* What you need before you install |
| [Non-native Ubuntu](installation/non-native-ubuntu.md) | Getting Ubuntu 22.04 on Windows, macOS or another Linux |
| [Kinova drivers](installation/kinova-drivers.md) | *Draft.* Installing and checking the Kortex driver stack |
| [Middleware and proxy](installation/middleware-and-proxy.md) | ROS 2, the Kinova driver packages, the middleware workspace, the proxy, an LLM provider |
| [Camera and calibration](installation/camera-and-calibration.md) | *Draft.* Setting up and calibrating the camera |
| [Simulation vs real arm](installation/simulation-vs-real.md) | *Draft.* What differs and how to switch |
| [Configuration](installation/configuration.md) | Coordinate dictionary, relative movements, obstacles, motion settings, recipes and the action contract |

### Technical

| Doc | What it gives you |
|---|---|
| [Architecture](technical/architecture.md) | Full pipeline diagram, per-node internals, custom service/message types, package layout |
| [Middleware nodes](technical/middleware-nodes.md) | *Draft.* What each node does |
| [Motion and actions](technical/motion-and-actions/index.md) | How actions become motion. [Pour](technical/motion-and-actions/pour.md), [Push](technical/motion-and-actions/push.md) and [Throw](technical/motion-and-actions/throw.md) show how each composite motion was captured and designed |
| [Proxy internals](technical/proxy-internals.md) | *Draft.* How the backend, LLM proxy and TUI fit together |
| [Computer vision](technical/computer-vision.md) | *Draft.* How objects are detected and added to the scene |
| [Safety and contracts](technical/safety-and-contracts.md) | The execution contract, safety layers, failure mode table, and system readiness definition |
| [Kinova drivers and APIs](technical/kinova-drivers-and-apis.md) | *Draft.* The driver interfaces, their errors, and where to look when a move fails |
| [Testing](technical/testing.md) | The middleware's recipe test suite, the proxy's LLM-output test suite, a manual smoke test |
| [Contributing](technical/contributing.md) | *Draft.* Branches, pull requests and the docstring rule |
| [API reference](technical/api/nodes.md) | Function-level reference generated from the code's own docstrings, for the middleware and the proxy |

## Support

Found something in these docs that's wrong, stale, or confusing? Flag it to the team or open a PR, documentation fixes are cheap and worth making immediately.
