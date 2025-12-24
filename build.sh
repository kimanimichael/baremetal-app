cmake -S ./bootloader/  -B bootloader/cmake-build/ -G Ninja
cmake --build bootloader/cmake-build/
cp bootloader/cmake-build/baremetal_bootloader.bin dev_tools/

cmake -S .  -B cmake-build/ -G Ninja
cmake --build cmake-build/