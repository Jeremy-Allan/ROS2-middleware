#!/usr/bin/env bash
# Sets up the ROS 2 side of embodied-ai: ROS 2 Humble, pinned ros2_kortex and this middleware.
# Usage: curl -fsSL https://raw.githubusercontent.com/Jeremy-Allan/ROS2-middleware/main/setup/install.sh | bash
#    or: ./setup/install.sh [--yes] [--prefix DIR] [--no-shell-config] [--force]
# Builds in <prefix>/ros2_kortex_ws, with ~/workspace as the default prefix.
set -Eeuo pipefail

MIDDLEWARE_REPO_URL="https://github.com/Jeremy-Allan/ROS2-middleware.git"
MIDDLEWARE_BRANCH="${MIDDLEWARE_BRANCH:-main}"
# gazebo_ros2_control is sim-only and has no arm64 package.
ROSDEP_SKIP_KEYS="gazebo_ros2_control"
BLOCK_START="# >>> embodied-ai >>>"
BLOCK_END="# <<< embodied-ai <<<"

assume_yes=false
prefix="$HOME/workspace"
workspace=""
write_shell_config=true
force=false
current_step="starting"

die() { echo "ERROR: $*" >&2; exit 1; }
step() { current_step="$1"; echo; echo "==> $1"; }

on_error() {
    echo
    echo "Setup failed during: $current_step"
    echo "Full log: $workspace/setup.log"
    echo "Fix the problem and run the same command again. It continues from where it stopped."
}

parse_arguments() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --yes) assume_yes=true ;;
            --prefix) [ $# -ge 2 ] || die "--prefix needs a folder"; prefix="$2"; shift ;;
            --no-shell-config) write_shell_config=false ;;
            --force) force=true ;;
            -h|--help) sed -n '2,5p' "${BASH_SOURCE[0]:-/dev/null}" 2>/dev/null; exit 0 ;;
            *) die "unknown option: $1" ;;
        esac
        shift
    done
    prefix="$(realpath -m "$prefix")"
    workspace="$prefix/ros2_kortex_ws"
}

check_environment() {
    [ "$(id -u)" -ne 0 ] || die "run this as your normal user, not root. It asks for sudo when needed."
    [ -r /etc/os-release ] || die "cannot read /etc/os-release"
    local os_id os_version
    os_id="$(. /etc/os-release && echo "$ID")"
    os_version="$(. /etc/os-release && echo "$VERSION_ID")"
    if [ "$os_id" != "ubuntu" ] || [ "$os_version" != "22.04" ]; then
        die "this needs Ubuntu 22.04 (found $os_id $os_version). Use WSL2, a UTM VM, Distrobox or native Ubuntu 22.04."
    fi
    case "$prefix" in
        /mnt/*) die "install under your Linux home folder, not $prefix. Builds under /mnt are slow and break permissions." ;;
    esac
    command -v sudo > /dev/null || die "sudo is required"
}

confirm_plan() {
    cat <<EOF

ROS 2 middleware setup will:
  - add the ROS 2 apt source and install ROS 2 Humble desktop, the ROS dev tools and ros2controlcli (sudo)
  - initialise rosdep (sudo)
  - clone ROS2-middleware ($MIDDLEWARE_BRANCH) and ros2_kortex (pinned commit) into $workspace/src
  - install their dependencies with rosdep (sudo) and build the workspace
EOF
    if $write_shell_config; then
        echo "  - add a marked block to $(shell_rc_file) that sources $workspace/activate"
    fi
    echo
    $assume_yes && return
    { : < /dev/tty; } 2> /dev/null || die "no terminal to ask for confirmation. Re-run with --yes."
    local answer
    read -r -p "Continue? [y/N] " answer < /dev/tty
    case "$answer" in
        y|Y|yes|YES) ;;
        *) echo "Cancelled. Nothing was changed."; exit 0 ;;
    esac
}

start_log() {
    mkdir -p "$workspace"
    exec > >(tee -a "$workspace/setup.log") 2>&1
    echo "--- setup started $(date) ---"
    trap on_error ERR
    sudo -v
}

install_ros() {
    step "Adding the ROS 2 apt source"
    if grep -rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/; then
        echo "Already present."
    else
        sudo apt-get update
        sudo apt-get install -y software-properties-common curl
        sudo add-apt-repository -y universe
        local apt_source_version
        apt_source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | cut -d'"' -f4)"
        [ -n "$apt_source_version" ] || die "could not look up the latest ros-apt-source release"
        curl -fsSL -o /tmp/ros2-apt-source.deb \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${apt_source_version}/ros2-apt-source_${apt_source_version}.jammy_all.deb"
        sudo dpkg -i /tmp/ros2-apt-source.deb
        rm -f /tmp/ros2-apt-source.deb
    fi

    step "Installing ROS 2 Humble and dev tools"
    sudo apt-get update
    sudo apt-get install -y git ros-humble-desktop ros-dev-tools ros-humble-ros2controlcli

    step "Setting up rosdep"
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update --rosdistro humble
}

# Prints "name commit" for each entry in a .repos file.
read_lockfile() {
    awk '/^  [A-Za-z0-9_.-]+:$/ { name = $1; sub(":", "", name) }
         /^    version:/ { print name, $2 }' "$1"
}

update_own_repo() {
    local repo_dir="$1" branch="$2"
    local current_branch
    current_branch="$(git -C "$repo_dir" rev-parse --abbrev-ref HEAD)"
    if [ "$current_branch" != "$branch" ]; then
        echo "Skipped updating $(basename "$repo_dir"): it is on '$current_branch', not '$branch'."
    elif [ -n "$(git -C "$repo_dir" status --porcelain)" ]; then
        echo "Skipped updating $(basename "$repo_dir"): it has uncommitted changes."
    else
        git -C "$repo_dir" pull --ff-only || echo "Skipped updating $(basename "$repo_dir"): it cannot fast-forward."
    fi
}

fetch_sources() {
    step "Fetching sources"
    mkdir -p "$workspace/src"
    local middleware_dir="$workspace/src/ROS2-middleware"
    if [ -d "$middleware_dir/.git" ]; then
        update_own_repo "$middleware_dir" "$MIDDLEWARE_BRANCH"
    else
        git clone --branch "$MIDDLEWARE_BRANCH" "$MIDDLEWARE_REPO_URL" "$middleware_dir"
    fi

    local lockfile="$middleware_dir/setup/ros2_kortex.repos"
    [ -f "$lockfile" ] || die "lockfile missing: $lockfile"
    local name commit
    while read -r name commit; do
        local repo_dir="$workspace/src/$name"
        if [ -d "$repo_dir/.git" ] && [ -n "$(git -C "$repo_dir" status --porcelain)" ]; then
            if $force; then
                echo "Discarding local changes in $name (--force)."
                git -C "$repo_dir" reset --hard
                git -C "$repo_dir" clean -fd
            else
                die "$name has local changes ($repo_dir). Commit or remove them, or re-run with --force to discard them."
            fi
        fi
    done < <(read_lockfile "$lockfile")
    vcs import "$workspace/src" < "$lockfile"
}

build_workspace() {
    step "Installing workspace dependencies"
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    rosdep install --from-paths "$workspace/src" --ignore-src --rosdistro humble -y --skip-keys "$ROSDEP_SKIP_KEYS"

    step "Building the workspace (the first build takes a while)"
    cd "$workspace"
    # Three workers keep the build within 8 GB of memory.
    colcon build --symlink-install --packages-up-to kinova_interface \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
}

shell_rc_file() {
    case "$(basename "${SHELL:-bash}")" in
        zsh) echo "$HOME/.zshrc" ;;
        bash) echo "$HOME/.bashrc" ;;
        *) echo "" ;;
    esac
}

write_shell_integration() {
    step "Writing the activate script"
    cat > "$workspace/activate" <<EOF
# Sources ROS 2 Humble and the embodied-ai workspace. Works in bash and zsh.
_embodied_ai_shell=bash
[ -n "\${ZSH_VERSION:-}" ] && _embodied_ai_shell=zsh
. /opt/ros/humble/setup.\$_embodied_ai_shell
[ -f "$workspace/install/setup.\$_embodied_ai_shell" ] && . "$workspace/install/setup.\$_embodied_ai_shell"
unset _embodied_ai_shell
case ":\$PATH:" in
    *:/usr/local/go/bin:*) ;;
    *) [ -d /usr/local/go/bin ] && export PATH="\$PATH:/usr/local/go/bin" ;;
esac
EOF
    echo "Wrote $workspace/activate"

    $write_shell_config || return 0
    local rc_file
    rc_file="$(shell_rc_file)"
    local source_line="[ -d /opt/ros/humble ] && [ -f \"$workspace/activate\" ] && . \"$workspace/activate\""
    if [ -z "$rc_file" ]; then
        echo "Your shell is not bash or zsh. Add this line to its startup file yourself:"
        echo "  $source_line"
    elif grep -qsF "$BLOCK_START" "$rc_file"; then
        echo "Shell block already in $rc_file."
    else
        printf '\n%s\n%s\n%s\n' "$BLOCK_START" "$source_line" "$BLOCK_END" >> "$rc_file"
        echo "Added a block to $rc_file between the '$BLOCK_START' and '$BLOCK_END' lines."
        echo "To remove it later, delete those lines."
    fi
}

run_checks() {
    step "Checking the result"
    local lockfile="$workspace/src/ROS2-middleware/setup/ros2_kortex.repos"
    local name commit actual_commit
    while read -r name commit; do
        actual_commit="$(git -C "$workspace/src/$name" rev-parse HEAD)"
        [ "$actual_commit" = "$commit" ] || die "$name is at $actual_commit, expected $commit"
        echo "OK  $name at pinned commit"
    done < <(read_lockfile "$lockfile")

    (
        set +u
        source "$workspace/install/setup.bash"
        ros2 pkg prefix kinova_interface > /dev/null
    ) || die "kinova_interface is not visible to ROS after the build"
    echo "OK  kinova_interface built and visible to ROS"

    dpkg -l 'ros-humble-*' > "$workspace/ros-packages.txt"
    echo "Saved installed ROS package versions to $workspace/ros-packages.txt"
}

print_next_steps() {
    cat <<EOF

ROS 2 side ready.

Start the simulated robot (open a new terminal first):
  ros2 launch kinova_interface robot.launch.py

Notes:
  - Config and launch edits apply without a rebuild. After adding a new file, run:
      cd $workspace && colcon build --symlink-install --packages-select kinova_interface
  - Update later by running the setup command again.
  - Remove everything: delete $workspace and the "embodied-ai" block in your shell startup file.
EOF
    if ! $write_shell_config || [ -z "$(shell_rc_file)" ]; then
        echo "  - Your shell does not source ROS automatically. In each new terminal run: source $workspace/activate"
    fi
}

main() {
    parse_arguments "$@"
    check_environment
    confirm_plan
    start_log
    install_ros
    fetch_sources
    build_workspace
    write_shell_integration
    run_checks
    print_next_steps
}

# Called on the last line so a partly downloaded script runs nothing.
main "$@"
