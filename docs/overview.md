# Overview

This system moves a Kinova Gen3 Lite robot arm from instructions typed in natural language. A language model converts each instruction into a recipe, an ordered list of named actions. The middleware runs the recipe through MoveIt 2 and the Kinova driver stack.

![System context: an operator types into the terminal UI; the backend asks an LLM provider for a recipe and sends it over a websocket to the ROS 2 nodes, which plan with MoveIt 2 and drive the arm through the Kortex driver.](assets/diagrams/system-context.svg)

The system has two parts that communicate over one websocket:

- The **proxy** (`embodied-ai-proxy`) takes the instruction, asks a language model for a recipe, checks it, and sends it on. It does not use ROS 2.
- The **middleware** (`ROS2-middleware`) is a set of ROS 2 packages. It resolves names to coordinates, plans each motion with MoveIt 2, and sends the motion to the arm.

## Components

The proxy, `embodied-ai-proxy`:

| Component | Role |
|---|---|
| Terminal UI | Accepts instructions. Shows the recipe, the result and the system status. |
| Backend | Builds the prompt, validates the model output, resolves object names and sends the recipe to the middleware. |
| LLM proxy | Sends prompts to Ollama, Google Gemini, OpenAI or Anthropic. |

The middleware, `ROS2-middleware`:

| Package | Role |
|---|---|
| `kinova_interface` | The four nodes, the ten actions, the launch file and the configuration. |
| `kinova_interfaces` | Custom ROS 2 services and messages. |
| `computer_vision` | Optional Kinect v2 node that adds detected objects to the scene. Alpha release. |

Software from other projects:

| Component | Role |
|---|---|
| `ros2_kortex` | Kinova's robot description, `ros2_control` driver and MoveIt configuration for the Gen3 Lite. |
| MoveIt 2 | Motion planning and collision checking. |
| `rosbridge_server` | Websocket access to ROS 2 services and topics. |

## How an instruction is processed

1. The operator types an instruction in the terminal UI.
2. The backend reads the objects, named movements, named orientations, table bounds and held objects from the middleware, and builds a prompt from the system prompt, the recipe schema and the instruction.
3. The LLM proxy sends the prompt to the configured provider and returns the text.
4. The backend extracts the JSON, validates it against the recipe schema and resolves object names against the live object list.
5. The backend sends the recipe to the `/execute_recipe` service through rosbridge.
6. `json_parser_node` runs the steps in order. Each action looks up names in `environment_mapping_node`, plans with MoveIt 2 and runs the motion through `hardware_interface_client`.
7. The result returns to the terminal UI. The UI also shows the system status published on `/system/status`.

Execution stops at the first step that fails.

## Where to continue

| Goal | Page |
|---|---|
| Install the software | [Requirements](installation/requirements.md) |
| Run the system | [Quickstart](user-guide/quickstart.md) |
| Write a recipe | [Recipes](user-guide/recipes.md) |
| Fix an error | [Troubleshooting](user-guide/troubleshooting.md) |
| Understand the design | [Architecture](technical/architecture.md) |
| Configure, run or evaluate the proxy | [Proxy documentation](proxy/README.md) |
