#!/bin/bash
# sync_to_shared.sh

DEST_DIR=~/shared_dir/race_stack

echo "=== Syncing modified packages to shared_dir ==="

# Delete everything and recreate
echo "Cleaning destination..."
rm -rf "$DEST_DIR/"

# Create necessary subdirectories
mkdir -p "$DEST_DIR/packages"
mkdir -p "$DEST_DIR/utilities/libraries"
mkdir -p "$DEST_DIR/stack_master/config"
mkdir -p "$DEST_DIR/stack_master/launch"

# Copy modified packages
echo "Copying packages..."
cp -r state_machine "$DEST_DIR/"
cp -r controller "$DEST_DIR/"
cp -r packages/overtake_section_detector "$DEST_DIR/packages/"
cp -r utilities/libraries/f110_msgs "$DEST_DIR/utilities/libraries/"

# Copy modified configs
echo "Copying configs..."
cp stack_master/config/state_machine_params.yaml "$DEST_DIR/stack_master/config/"
cp stack_master/launch/head_to_head_launch.xml "$DEST_DIR/stack_master/launch/"

# Copy documentation
echo "Copying documentation..."
cp -r docs "$DEST_DIR/"

echo ""
echo "=== Sync complete! ==="
echo ""
echo "Next steps:"
echo "  cd ~/shared_dir"
echo "  ./dkrun.sh misys:forza"
echo ""
echo "Inside container:"
echo "  cd /home/misys/shared_dir/race_stack"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select f110_msgs state_machine controller"
echo "  source install/setup.bash"
echo "  ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM LU_table:=default ctrl_algo:=PP"
