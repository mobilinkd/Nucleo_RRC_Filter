# RRC Filter

This is a root-raised cosine filter for M17 baseband demodulation. It is designed
to run on a modified STM32L432KC Nucleo32 board. The modification needed is to have
the SB17 solder-bridge shorted to provide HSE to the chip.

It should be easy enough to change the IOC to use the HSI or MSI instead if one
would prefer not to modify their Nucleo dev board in this way. The modification
will prevent the use of the op-amp and PGA.

Bias the input and feed it into PA7 (ADC1_IN12). I use a 22uF capacitor to couple
the signal from the source, with two 100k resistors to +3V3 and GND to bias the
input voltage at 1.65V.

```
                          100k
                  22uF  +--vvv---3V3
    AUDIO_IN ------||-------------------- PA7
                        +--vvv---GND
                          100k
```

The RRC-filtered output will be on PA4 (DAC1_OUT1).

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
