#!/bin/bash
# sync_to_shared.sh

DEST_DIR=~/shared_dir/race_stack

echo "=== Syncing modified packages to shared_dir ==="

# 1. 목적지 초기화
echo "Cleaning destination..."
rm -rf "$DEST_DIR/"

# 2. 필수 하위 폴더 생성
mkdir -p "$DEST_DIR/packages"
mkdir -p "$DEST_DIR/utilities/libraries"

# 3. 패키지 복사
echo "Copying packages..."
cp -r state_machine "$DEST_DIR/"
cp -r controller "$DEST_DIR/"
cp -r packages/overtake_section_detector "$DEST_DIR/packages/"
cp -r utilities/libraries/f110_msgs "$DEST_DIR/utilities/libraries/"
cp -r stack_master "$DEST_DIR/"

# 빌드 충돌 방지를 위해 불필요한 폴더 제거
rm -rf "$DEST_DIR/stack_master/build"
rm -rf "$DEST_DIR/stack_master/install"
rm -rf "$DEST_DIR/stack_master/log"
rm -rf "$DEST_DIR/stack_master/__pycache__"

echo "=== Sync complete! ==="