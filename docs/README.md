# Kinova Middleware Manual

This manual describes a system in which a language model controls a Kinova Gen3 Lite robot arm. The proxy turns an instruction typed in natural language into a recipe. The ROS 2 middleware runs the recipe on the arm with MoveIt 2.

| Section | Contents |
|---|---|
| [Overview](overview.md) | The parts of the system and the path of an instruction. |
| [User guide](user-guide/quickstart.md) | Start the system, the ten actions, recipes, demonstration recipes and troubleshooting. |
| [Installation](installation/requirements.md) | Requirements, the setup script, the Kinova drivers, the middleware and the proxy, the camera, simulation and real arm, configuration. |
| [Technical](technical/architecture.md) | Architecture, nodes, ROS interfaces, motion planning, safety, the Kinova drivers, computer vision, testing, contributing and the API reference. |
| [Proxy](proxy/README.md) | Installation, configuration, running, the terminal UI, evaluation of a language model, architecture, interfaces and the API reference of the proxy. |
