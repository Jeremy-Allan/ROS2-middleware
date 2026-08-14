# Documentation Home

Welcome. This is the full reference for the Kinova Gen3 Lite middleware, and for the pipeline it sits in (an LLM proxy talks to this middleware over ROS 2 to move a real or simulated robot arm).

If you read nothing else, read these two pages:

1. [Overview](overview.md): what this system actually is and how a command travels through it, one page, plain English.
2. [Installation](installation.md): get both halves of the pipeline built and sourced.

## Pick your path

**I'm brand new to this project.** Start with [Overview](overview.md), then [Installation](installation.md). You do not need prior ROS 2 experience, both pages explain concepts as they come up.

**I just want to get it running.** [Installation](installation.md), then [Running the System](running.md) walks through the three terminals you need.

**I need to add a new object, movement, or recipe.** [Configuration](configuration.md) covers the coordinate dictionary, relative movements, obstacles, recipes, and the LLM provider/prompt setup.

**I'm writing or running tests.** [Testing](testing.md) covers the middleware's recipe-based test suite and the proxy's LLM-output test suite, and is honest about what neither one currently catches.

**Something's broken.** [Troubleshooting](troubleshooting.md) is organized by symptom, with the likely cause and the fix, split by which half of the pipeline it's in.

**I want to understand how it actually works.** [Architecture](architecture.md) is the deep dive, per-node internals, the full pipeline diagram, every custom service and message type, and the package layout.

**I want the formal guarantees and safety model.** [Safety and Contracts](safety-and-contracts.md) covers the execution contract, failure modes, and safety layers this middleware is designed against.

## The whole map

### Start here

| Doc | What it gives you |
|---|---|
| [Overview](overview.md) | What the system is, the request path end to end, the four middleware nodes, the proxy's two domains |
| [Installation](installation.md) | ROS 2, the Kinova driver stack, the middleware workspace, the proxy, an LLM provider |

### Use it day to day

| Doc | What it gives you |
|---|---|
| [Running the System](running.md) | The three-terminal startup, running the middleware alone, watching live telemetry |
| [Configuration](configuration.md) | Coordinate dictionary, relative movements, obstacles, recipes and the action contract, LLM provider config, system prompt and schema |
| [Testing](testing.md) | The middleware's recipe test suite, the proxy's LLM-output test suite, a manual smoke test |
| [Troubleshooting](troubleshooting.md) | Symptom-first tables across build errors, recipe errors, motion errors, telemetry, and proxy/LLM connection issues |

### Understand it deeply

| Doc | What it gives you |
|---|---|
| [Architecture](architecture.md) | Full pipeline diagram, per-node internals, custom service/message types, package layout |
| [Safety and Contracts](safety-and-contracts.md) | The execution contract, safety layers, failure mode table, and system readiness definition |

### Reference

| Doc | What it gives you |
|---|---|
| [Video Demos](demos.md) | Short recordings of each feature in action |
| [PROJECT-OVERVIEW](../kinova_interface/resource/PROJECT-OVERVIEW.md) | Original Sprint 1/2 design notes and history, kept where it already lived |

## Where else to get help

Found something in these docs that's wrong, stale, or confusing? Flag it to the Core-Middleware sub-team or open a PR, documentation fixes are cheap and worth making immediately.
