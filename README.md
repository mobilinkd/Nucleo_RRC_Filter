# Build Requirements

 - stm32cubeclt_1.16.0

# Build Instructions

```bash
mkdir -p build/RelWithDebInfo
cd build/RelWithDebInfo
/opt/st/stm32cubeclt_1.16.0/CMake/bin/cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=../../cmake/gcc-arm-none-eabi.cmake -S../.. -B. -G Ninja
/opt/st/stm32cubeclt_1.16.0/CMake/bin/cmake --build . -- clean
/opt/st/stm32cubeclt_1.16.0/CMake/bin/cmake --build . --
```
