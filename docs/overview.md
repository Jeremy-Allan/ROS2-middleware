[Docs Home](README.md)

# Overview

This project lets you type a plain English instruction, like "pick up the blue cube and put it on the tray," and have a real (or simulated) Kinova Gen3 Lite robot arm actually do it.

It assumes zero prior ROS 2 experience. Anywhere a ROS 2 concept shows up for the first time, it's explained in plain English before it's used.

## Two repositories, one pipeline

This is split across two separate git repositories, built and run independently, but they only make sense together:

- `Jeremy-Allan/ROS2-middleware` (this repo): the ROS 2 side. Four nodes that turn a structured recipe into real robot motion using MoveIt 2.
- `paul-isit/embodied-ai-proxy`: the LLM side. A set of Go services and a terminal UI that take a natural language command, send it to a configurable LLM, validate the LLM's JSON output, and forward it to this middleware over a websocket bridge.

## The path your command takes

1. You type a command into the proxy's terminal interface.
2. The proxy sends your command, plus a list of objects the robot actually knows about, to a large language model. That model can be a local model running through Ollama, or a cloud provider (OpenAI, Anthropic, Gemini).
3. The LLM responds with a structured JSON "recipe": a named sequence of steps such as `move_arm`, `gripper`, `relative_move`, `home`.
4. The proxy validates that JSON against a strict schema. If it's malformed, or asks for an object that doesn't exist, the proxy rejects it before anything is sent to the robot.
5. A valid recipe gets sent over a websocket to a small ROS 2 bridge node, which turns it into a real ROS 2 service call.
6. This middleware picks up that call, resolves any named object into real coordinates, and drives the arm through MoveIt 2, step by step, waiting for each step to finish before starting the next.
7. Status flows back the other direction the whole time, so the proxy's interface can show whether the robot is READY, BUSY, or FAULT.

## What the system does not do

- Accept direct hardware commands from the proxy's user input. Every command typed into the proxy goes through the LLM and the schema validator first. Note that the websocket bridge itself does not filter services, so any websocket client can call any middleware service (see [Safety and Contracts](safety-and-contracts.md)).
- Do any of its own path-planning or collision math. That's entirely delegated to MoveIt 2.
- Talk to the physical robot directly from the proxy side. Only this middleware talks to the standard Kinova ROS 2 driver (`ros2_kortex`), which in turn talks to the arm.

## The middleware's four nodes, in one sentence each

This repository is built from four ROS 2 programs called nodes, each doing one job and talking to the others over the network:

| Node | Nickname | Job |
|---|---|---|
| `hardware_interface_client.py` | The Muscle | The only node that actually talks to the arm and gripper. Sends motion commands, waits for them to finish, reports hardware faults. |
| `environment_mapping_node.py` | The Database | Holds the "map" of named objects (for example, `blue_cube` maps to a pose and shape), named relative movements, and known obstacles. Answers lookup questions from other nodes. |
| `json_parser_node.py` | The Brain / Supervisor | Reads a recipe, resolves names via the Database node, and tells the Muscle node what to do, step by step, in order. |
| `telemetry_node.py` | The Monitor | Listens to heartbeats from all the other nodes, decides if the whole system is READY, BUSY, or FAULT, and can trigger a hardware fault reset. |

Object positions come from a hand-maintained dictionary (see [Configuration](configuration.md)). An optional fifth node in the `computer_vision` package (`enable_CV:=true`) can add objects detected by a Kinect camera when `/vision/snapshot` is called.

See [Architecture](architecture.md) for the internals of each node.

## The proxy and the bridge

The proxy and the middleware are deliberately kept separate, so a bug in the LLM logic can never directly touch the robot:

| Side | What it is | Job |
|---|---|---|
| The proxy (embodied-ai-proxy) | No ROS 2 installed or launched | Talks to the LLM provider, builds the prompt, validates the JSON response against the schema, runs the terminal interface, and connects to the middleware's websocket as a client. |
| The websocket bridge (this repo's launch) | `rosbridge_suite`'s websocket server, started by `robot.launch.py` | Listens on port 9090 and translates incoming JSON requests into ROS 2 service calls against this middleware. The proxy's only way in. |

If you only remember one sentence about this whole project: **your words go to an LLM, the LLM's structured answer goes through a validator and a websocket bridge, and only then does the middleware's Brain drive the Muscle one step at a time while the Monitor watches everyone's pulse.**

Next: [Installation](installation.md)
