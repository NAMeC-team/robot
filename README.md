# mainboard

mainboard Mbed OS project

## Requirements

### Hardware requirements

The following boards are required:

- _List mainboard hardware requirements here_

### Software requirements

mainboard makes use of the following libraries (automatically
imported by `mbed deploy` or `mbed import`):

- _List mainboard software requirements here_

## Usage

To clone **and** deploy the project in one command, use `mbed import` and skip to the
target enabling instructions:

```shell
mbed import https://github.com/NAMeC-SSL/robot.git robot
```

Alternatively:

- Clone to "robot" and enter it:

  ```shell
  git clone https://github.com/NAMeC-SSL/robot.git robot
  cd robot
  ```

- Deploy software requirements with:
  ```shell
  mbed deploy
  ```

Enable the custom target:

```shell
cp zest-core-stm32l4a6rg/custom_targets.json .
```

Compile the project:

```shell
mbed compile
```

Program the target device with a Segger J-Link debug probe and
[`sixtron_flash`](https://gitlab.com/catie_6tron/6tron-flash) tool:

```shell
sixtron_flash stm32l4a6rg BUILD/ZEST_CORE_STM32L4A6RG/GCC_ARM/mainboard.elf
```

Debug on the target device with the probe and Segger
[Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger)
software.
