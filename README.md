# mainboard

mainboard Mbed OS project

## Requirements

### Hardware requirements

The following boards are required:

- _List mainboard hardware requirements here_

### Software requirements

You need to install the following [Setup tools](https://6tron.io/ressources_logicielles/mbed/setup-tools#6tron-flash-tool) (from the website of the CATIE).
Install the following using their website :
- Pipx
- GNUM Arm Embedded Toolchain
- Mbed CLI
- 6TRON Flash Tool
- SEGGER J-Link Debug Probe drivers

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
the `sixtron_flash` tool, that you installed with in the "Software Prerequisites" section.

```shell
sixtron_flash stm32l4a6rg BUILD/ZEST_CORE_STM32L4A6RG/GCC_ARM/robot.elf
```

Debug on the target device with the probe and Segger
[Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger)
software.

# Linter & static analyzer setup (LSP)
We recommend using [`clangd`](https://clangd.llvm.org/installation) as your LSP server
for auto-completing, indexing and checks. The repository has a `compile_commands.json` that you
must point your IDE to, because it is not in a standard location for `clangd`

## Debian-like
To install clangd, you can either install the apt package (see [here]())

## Official binaries
Download the latest binary of `clangd` https://github.com/clangd/clangd/releases/latest for your architecture
(usually Linux x86_64, unless you have a Mac or something spicy).

```bash
# If wget is not installed
# sudo apt install wget
wget https://github.com/clangd/clangd/releases/download/23.1.0/clangd-linux-23.1.0.zip
unzip clangd-linux-23.1.0.zip
```

The folder contains two folders : `bin` and `lib`. You can either install it for your user or system-wide.
### System-wide

```bash
mv clangd-linux-23.1.0/bin/clangd /usr/bin
mv clangd-linux-23.1.0/lib/* /usr/lib/
```

### User-only
```bash
mkdir -p $HOME/.local/bin
mv clangd-linux-23.1.0/bin/clangd $HOME/.local/bin

mkdir -p $HOME/.local/lib
mv clangd-linux-23.1.0/lib/clang $HOME/.local/lib
```

And add the following to your .bashrc file if it isn't present already
```
export PATH=$PATH:$HOME/.local/bin/
```

Once installed, you must setup your IDE to work with `clangd`.

## JetBrains / CLion
Should detect the file by itself. If it doesn't, follow the [official tutorial](https://www.jetbrains.com/help/clion/compilation-database.html#compdb_clion).

## VSCode
Install the [clangd extension for VSCode](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
and open any code file on your IDE. If clangd is not installed, VSCode will offer to install it for you.
If you accept, `clangd` will only be available to VSCode and not system-wide.

After `clangd` is installed, add the following lines to your user settings (brackets represent the main object).

```json
{
  "C_Cpp.intelliSenseEngine": "disabled",
  "clangd.arguments": [
    "--query-driver=**/arm-none-eabi-*",
    "--background-index",
    "--header-insertion=never",
    "--compile-commands-dir=${workspaceFolder}"
  ]
}
```

## Updating compile_commands.json
If the build system were to change, note that the compilation database was generated using [ʕ·ᴥ·ʔ Build ear](https://github.com/rizsotto/bear) :
```bash
bear -- mbed compile -m ZEST_CORE_STM32L4A6RG -t GCC_ARM
```