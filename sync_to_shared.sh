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
mkdir -p "$DEST_DIR/stack_master/maps/small_hall"

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
cp stack_master/maps/small_hall/ot_sectors.yaml "$DEST_DIR/stack_master/maps/small_hall/"

echo "=== Sync complete! ==="