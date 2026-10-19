#!/bin/bash
##########################################################################
# ABV GNC - Environment Setup
#
# Installs ROS2 Humble, system dependencies, and third-party libraries
# required to build and run the ABV GNC stack.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
#
# Assumptions:
#   - Ubuntu 22.04 (Jammy) on x86_64 or ARM64 (Jetson Orin)
#   - Internet access for apt and git
#   - sudo privileges
#
# What this script does:
#   1. Installs ROS2 Humble Desktop
#   2. Installs system dependencies via apt
#   3. Builds and installs third-party libs from source (plog, libmotioncapture, JETGPIO)
#   4. Builds the ABV ROS2 packages via colcon
#
# Third-party libs are cloned to ~/libs by default (see LIBS_DIR)
##########################################################################

# where to clone libs that need built from source
LIBS_DIR=~/libs

# List of required system packages
DEPENDENCIES=(
    build-essential
    cmake
    curl
    vim
    tmux
    libeigen3-dev
    libyaml-cpp-dev
    libsfml-dev
    libboost-system-dev
    libboost-thread-dev
    qt6-base-dev
    libqt6charts6-dev
)

# Function to check and install a package
check_and_install() {
    PKG="$1"
    CUSTOM_INSTALL_FUNC="$2"

    if dpkg -s "$PKG" >/dev/null 2>&1; then
        echo "[✔] $PKG is already installed."
    else
        echo "[✘] $PKG is not installed."

        if [ -n "$CUSTOM_INSTALL_FUNC" ] && declare -f "$CUSTOM_INSTALL_FUNC" > /dev/null; then
            echo "[↪] Using custom install function: $CUSTOM_INSTALL_FUNC"
            "$CUSTOM_INSTALL_FUNC"
        else
            echo "[↪] Installing $PKG via apt..."
            sudo apt update && sudo apt install -y "$PKG"
        fi
    fi
}

clone_and_checkout() {
    LIB="$1"
    URL="$2"
    TAG="$3"
    DIR="$4"

    # Clone if missing
    if [ ! -d "$DIR/$LIB" ]; then
        git clone https://github.com/"$URL".git "$DIR/$LIB"
    fi

    cd "$DIR/$LIB"
    git checkout "$TAG"
}

install_from_source() {
    LIB="$1"
    URL="$2"
    TAG="$3"
    DIR="$4"
    shift 4
    CMAKE_ARGS=("$@")

    clone_and_checkout $LIB $URL $TAG $DIR

    # Always update submodules (safe even if none exist)
    echo "[↪] Updating submodules for $LIB..."
    git submodule update --init --recursive

    mkdir -p build
    cd build

    echo "[↪] Running CMake for $LIB with args: ${CMAKE_ARGS[*]}"

    cmake -DCMAKE_BUILD_TYPE=Release "${CMAKE_ARGS[@]}" ..
    make -j"$(nproc)"
    sudo make install
}

install_ros() {
    echo "[ROS] Starting installation of ROS 2 Humble..."

    # --- Configure UTF-8 Locale ---
    if ! locale | grep -q "UTF-8"; then
        echo "[ROS] Configuring UTF-8 locale..."
        sudo apt install -y locales
        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
    fi

    # --- Set Timezone Non-Interactively ---
    echo "[ROS] Setting timezone to America/New_York..."
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
    DEBIAN_FRONTEND=noninteractive sudo apt install -y tzdata
    dpkg-reconfigure -f noninteractive tzdata

    # --- Add Required Tools ---
    DEBIAN_FRONTEND=noninteractive sudo apt install -y software-properties-common curl gnupg lsb-release

    # --- Add Universe Repo (auto-confirm) ---
    echo "[ROS] Adding universe repository..."
    add-apt-repository -y universe

    # --- Add ROS 2 GPG Key ---
    echo "[ROS] Adding ROS 2 GPG key..."
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # --- Add ROS 2 Repository ---
    echo "[ROS] Adding ROS 2 repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/ros2.list

    # --- Install ROS 2 Humble and Tools ---
    echo "[ROS] Installing ROS 2 Humble and tools..."
    sudo apt update
    DEBIAN_FRONTEND=noninteractive sudo apt install -y ros-humble-desktop \
        ros-humble-xacro
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

    # --- Source ROS Environment ---
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    echo "[✔] ROS 2 Humble installed and environment sourced."
}

###########################################################################
## Script run logic
###########################################################################

# get current dir so we can come back at the end to build the packages
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# make sure things are updated
sudo apt update

# treat ros2 install special
check_and_install "ros-humble-desktop" "install_ros"

# install system deps
for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

# install deps from source
install_from_source plog SergiusTheBest/plog 1.1.10 $LIBS_DIR
install_from_source libmotioncapture samlovelace/libmotioncapture main "$LIBS_DIR" \
    "-DCMAKE_INSTALL_PREFIX=/usr/local"

# custom steps for JETGPIO
clone_and_checkout JETGPIO Rubberazer/JETGPIO v1.2 "$LIBS_DIR"
echo "orinagx" > hardware
make && sudo make install
# end JETGPIO custom

# robot_ws setup
ROBOT_WS=~/robot_ws
mkdir -p "$ROBOT_WS"/src
clone_and_checkout ptera samlovelace/ptera main "$ROBOT_WS"/src
source /opt/ros/humble/setup.bash
cd "$ROBOT_WS" && colcon build --packages-skip ptera_sim

# build the main workspace
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source "$ROBOT_WS"/install/setup.bash
colcon build
