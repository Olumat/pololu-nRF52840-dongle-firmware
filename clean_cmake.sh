#!/bin/bash
# CMake cache cleanup script for nRF52840 Zephyr project
# This removes all cached build artifacts that reference old paths

set -e  # Exit on error

echo "========================================"
echo "Cleaning CMake cache and build artifacts"
echo "========================================"

# 1. Remove build directory completely
if [ -d "build" ]; then
    echo "Removing build/ directory..."
    rm -rf build
    echo "✓ build/ removed"
else
    echo "✓ No build/ directory found"
fi

# 2. Remove generated DFU package
if [ -f "zephyr.zip" ]; then
    echo "Removing zephyr.zip..."
    rm -f zephyr.zip
    echo "✓ zephyr.zip removed"
else
    echo "✓ No zephyr.zip found"
fi

# 3. Clean any stray CMake cache files
echo "Searching for stray CMakeCache.txt files..."
find . -name "CMakeCache.txt" -not -path "./build/*" 2>/dev/null | while read -r file; do
    echo "  Removing: $file"
    rm -f "$file"
done

# 4. Clean any stray CMakeFiles directories
echo "Searching for stray CMakeFiles directories..."
find . -name "CMakeFiles" -type d -not -path "./build/*" 2>/dev/null | while read -r dir; do
    echo "  Removing: $dir"
    rm -rf "$dir"
done

# 5. Clean Zephyr cache (optional, uncomment if needed)
# echo "Removing Zephyr cache..."
# rm -rf ~/.cache/zephyr
# echo "✓ Zephyr cache removed"

echo ""
echo "========================================"
echo "✓ Cleanup complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "1. Run: west build -b nrf52840dongle . --pristine -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
echo "2. Or run: ./build_and_flash.sh"
echo ""
