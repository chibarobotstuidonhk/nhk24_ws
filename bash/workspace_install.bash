# Desc: Install workspace dependencies

. bash/keep_safe.bash

# Get the directory of the script, and the workspace directory
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
ws_dir="$script_dir/../"

# Install ros packages ############################################################
cd "$ws_dir/src" || exit

# Install urg_node2 collected by Stew-000-1-0-011
git clone https://github.com/Stew-000-1-0-011/urg_node2.git

# Install can_plugins2
git clone https://github.com/IndigoCarmine/can_plugins2.git

# Install odometry2024 (add noise version)
git clone https://github.com/Stew-000-1-0-011/odometry2024.git


# Install tools ###################################################################
cd "$ws_dir/tool" || exit

# Install Get_coordinate_on_map
git clone https://github.com/Won-CCS/Get_coordinate_on_map.git

# Install map_editor
git clone https://github.com/Stew-000-1-0-011/map_editor.git
