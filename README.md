# Kinova Gen3 Lite ROS 2 Middleware

A modular ROS 2 middleware that lets a large language model control a Kinova Gen3 Lite robot arm. This repository is the execution layer: it takes a structured JSON recipe, resolves named objects into real coordinates, and drives the arm through MoveIt 2, step by step. It's paired with a separate LLM proxy ([`embodied-ai-proxy`](https://github.com/paul-isit/embodied-ai-proxy)) that turns natural language into that recipe.

Built for Swinburne University's Cyber Lab, researching how LLMs respond to robotic instructions, including malicious ones.

## How it fits together

```
natural language  ->  embodied-ai-proxy  ->  websocket bridge  ->  this middleware  ->  robot arm
```

Four ROS 2 nodes do the work here:

- **`hardware_interface_client.py`**, the Muscle: the only node that talks to the arm and gripper.
- **`environment_mapping_node.py`**, the Database: maps object names to coordinates, holds known obstacles.
- **`json_parser_node.py`**, the Brain: reads a recipe and drives the other nodes step by step.
- **`telemetry_node.py`**, the Monitor: aggregates health from all nodes, handles fault recovery.

Full explanation, with the request path spelled out end to end: [docs/overview.md](docs/overview.md).

## Quick start

Requires Ubuntu 22.04 LTS, ROS 2 Humble, Python 3.10+. Full setup, including the Kinova driver stack this repo depends on: [docs/installation.md](docs/installation.md).

```bash
cd ~/workspace/ros2_kortex_ws
colcon build --packages-select kinova_interface kinova_interfaces
source install/setup.bash

ros2 launch kinova_interface robot.launch.py
```

That launches the full stack in simulation and waits for a recipe. See [docs/running.md](docs/running.md) for running against physical hardware, running a recipe automatically, and running the full pipeline together with the proxy.

## Docs

**Start here:** the [Documentation Home](docs/README.md) maps everything.

- [Overview](docs/overview.md): what this system is, the request path end to end
- [Installation](docs/installation.md): ROS 2, the driver stack, the middleware, the proxy, an LLM provider
- [Running the System](docs/running.md): the three-terminal startup, running the middleware alone
- [Configuration](docs/configuration.md): coordinate dictionary, recipes, LLM provider and prompt setup
- [Testing](docs/testing.md): the recipe test suite and the proxy's LLM-output tests
- [Troubleshooting](docs/troubleshooting.md): symptom-first fixes
- [Architecture](docs/architecture.md): full pipeline diagram, per-node internals, interface types
- [Safety and Contracts](docs/safety-and-contracts.md): execution contract, safety layers, failure modes
- [Video Demos](docs/demos.md): short recordings per feature

## Repository layout

```
kinova_interface/      the runtime nodes, launch file, recipes, config
kinova_interfaces/     custom ROS 2 service and message definitions
docs/                  full documentation, start at docs/README.md
```

## Contributing

Core-Middleware sub-team owns this repo. Open a PR against a feature branch; flag documentation gaps directly, they're cheap to fix.
