. bash/setup_install.bash

run_command() {
    gnome-terminal --tab --title="$1" -- bash -c ". bash/setup_install.bash; $2"
}
