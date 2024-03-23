# Desc: Install workspace dependencies

# Get the directory of the script, and the workspace directory
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
ws_dir="$script_dir/.."

# check if the directory already exists
if [ -d "$ws_dir/already_installed" ]; then
    echo "Error: Directory $ws_dir/src already exists. Maybe you have already installed the workspace."
    exit 1
fi
mkdir "$ws_dir/already_installed"

# Install ros packages ############################################################
mkdir "$ws_dir/src"
cd "$ws_dir/src" || exit

# Install r2
git clone https://github.com/Stew-000-1-0-011/r2.git

# Install urg_node2 collected by Stew-000-1-0-011
git clone https://github.com/Stew-000-1-0-011/urg_node2.git --recursive
rosdep update
rosdep install -i --from-paths urg_node2

# Install can_plugins2
git clone https://github.com/IndigoCarmine/can_plugins2.git

# Install odometry2024 (add noise version)
git clone https://github.com/Stew-000-1-0-011/odometry2024.git


# Install tools ###################################################################
mkdir "$ws_dir/tool"
cd "$ws_dir/tool" || exit

# Install Get_coordinate_on_map
git clone https://github.com/Won-CCS/Get_coordinate_on_map.git

# Install map_editor
git clone https://github.com/Stew-000-1-0-011/map_editor.git

cd "$ws_dir" || exit