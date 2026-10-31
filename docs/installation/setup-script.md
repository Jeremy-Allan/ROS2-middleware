# Setup script

The setup script does the steps of [Kinova drivers](kinova-drivers.md) and [Middleware and proxy](middleware-and-proxy.md) in one command. It installs ROS 2 Humble, builds the Kinova packages and the middleware, and builds the proxy. It does not start anything.

## Prerequisites

- Ubuntu 22.04 on amd64 or arm64. For another system, see [Non-native Ubuntu](non-native-ubuntu.md).
- `curl` and `sudo`.
- Internet access. The build downloads the Kinova API from `artifactory.kinovaapps.com`.

## Run the script

Run the proxy's setup script. It builds the proxy, then runs the middleware's setup script:

```bash
curl -fsSL https://raw.githubusercontent.com/paul-isit/embodied-ai-proxy/main/setup/install.sh | bash
```

The script lists what it will change and asks once for confirmation. `sudo` asks for your password. The first build takes a while.

When it finishes, open a new terminal and continue with the [Quickstart](../user-guide/quickstart.md).

To set up the middleware only, run its script instead:

```bash
curl -fsSL https://raw.githubusercontent.com/Jeremy-Allan/ROS2-middleware/main/setup/install.sh | bash
```

## What the script does

| Step | Result |
|---|---|
| ROS 2 | Adds the ROS 2 apt source and installs `ros-humble-desktop`, `ros-dev-tools` and `ros-humble-ros2controlcli`. Initialises `rosdep`. |
| Sources | Clones `ROS2-middleware` on `main` and `ros2_kortex` at the commit pinned in `setup/ros2_kortex.repos`. |
| Build | Installs the dependencies with `rosdep` and builds up to `kinova_interface` with `--symlink-install`. `computer_vision` is not built. |
| Shell | Writes `activate`, which loads ROS 2 and the workspace, and adds a block that sources it to `~/.bashrc` or `~/.zshrc`. |
| Proxy | Installs Go 1.26.5 to `/usr/local/go` if Go is missing or older, clones `embodied-ai-proxy` on `main` and runs `./run.sh --build-only`. |

The directories match the rest of this manual:

```text
~/workspace
├── embodied-ai-proxy
└── ros2_kortex_ws
    ├── activate
    ├── setup.log
    ├── ros-packages.txt
    └── src
        ├── ROS2-middleware
        └── ros2_kortex
```

`ros-packages.txt` lists the installed ROS 2 package versions. Compare it between two machines that behave differently.

## Options

| Option | Effect |
|---|---|
| `--yes` | Skips the confirmation. Needed when no terminal is attached. |
| `--prefix <dir>` | Uses `<dir>` instead of `~/workspace`. |
| `--no-shell-config` | Does not change the shell startup file. Run `source ~/workspace/ros2_kortex_ws/activate` in each terminal. |
| `--force` | Middleware script only. Discards local changes in `ros2_kortex` to move it to the pinned commit. |
| `--skip-middleware` | Proxy script only. Builds the proxy without ROS 2. Runs on Linux and macOS. |

Pass options after `bash -s --`:

```bash
curl -fsSL <script URL> | bash -s -- --yes
```

## Proxy on another computer

With `--skip-middleware`, the proxy runs on a computer without ROS 2. Set `rosbridge_url` in `~/workspace/embodied-ai-proxy/data/config/config.json` to `ws://<middleware address>:9090`. Port 9090 of the middleware computer must be reachable.

## Update or repair

Run the same command again. Steps that are already done, such as cloning or adding the shell block, are skipped. The installs and the build run again but finish quickly when nothing changed, so a failed run continues where it stopped.

- `ros2_kortex` moves to the pinned commit. The script stops if it has local changes, unless `--force` is set.
- `ROS2-middleware` and `embodied-ai-proxy` are updated with `git pull --ff-only`, only when they are on `main` with no local changes.

## Remove

Delete `~/workspace/ros2_kortex_ws` and `~/workspace/embodied-ai-proxy`, and the lines between `# >>> embodied-ai >>>` and `# <<< embodied-ai <<<` in your shell startup file. ROS 2 and Go stay installed.

## Test a branch

To test the scripts from a branch before it is merged, push the branch and set the branch variables:

```bash
curl -fsSL https://raw.githubusercontent.com/paul-isit/embodied-ai-proxy/<branch>/setup/install.sh \
  | PROXY_BRANCH=<branch> MIDDLEWARE_BRANCH=<branch> bash
```

`MIDDLEWARE_BRANCH` also works with the middleware script.
