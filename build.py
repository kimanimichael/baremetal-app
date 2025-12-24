"""
Requires:
    - st-utils v1.6.0
    - arm-none-eabi-size
"""

import os
import shutil
import colorama
import click

colorama.init()

IMAGE = "cmake-build/{APP_TARGET}.bin"
ELF_IMAGE = "cmake-build/{APP_TARGET}.elf"

APPLICATION = "./application.bin"

CMAKE_GEN_CMD = '''cmake -S . -B cmake-build/ -G Ninja'''
CMAKE_BUILD_CMD = '''cmake --build cmake-build/'''

CMAKE_GEN_BOOTLOADER_CMD = '''cmake -S ./bootloader/  -B bootloader/cmake-build/ -G Ninja'''
CMAKE_BUILD_BOOTLOADER_CMD = '''cmake --build bootloader/cmake-build/'''

FLASH_APPLICATION_CMD = '''st-flash write 
{image} 0x8000000 && st-info --reset
'''

def build_bootloader():
    print(colorama.Fore.CYAN, "Building Bootloader .." + colorama.Style.RESET_ALL)

    cmake_gen_cmd = CMAKE_GEN_BOOTLOADER_CMD.format()
    cmake_build_cmd = CMAKE_BUILD_BOOTLOADER_CMD.format()

    print(cmake_gen_cmd)
    print(cmake_build_cmd)

    ret = os.system(" ".join(cmake_gen_cmd.split("\n")))
    if ret != 0:
        raise Exception("CMake configuration generation failed!")

    ret = os.system(" ".join(cmake_build_cmd.split("\n")))
    if ret != 0:
        raise Exception("CMake build failed!")
    print(colorama.Fore.GREEN + "Compiled bootloader successfully" + colorama.Style.RESET_ALL)

    bin_name = "baremetal_bootloader.bin"
    source_path = os.path.join("bootloader", "cmake-build", bin_name)
    destination_dir = os.path.join("dev_tools")
    destination_path = os.path.join(destination_dir, bin_name)

    os.makedirs(destination_dir, exist_ok=True)

    if not os.path.isfile(source_path):
        raise FileNotFoundError(f"Expected binary not found at: {source_path}")
    shutil.copy2(source_path, destination_path)
    print(colorama.Fore.GREEN + f"Copied {bin_name} to {destination_path}" + colorama.Style.RESET_ALL)

def build_image():
    print(colorama.Fore.CYAN, "Building Image .." + colorama.Style.RESET_ALL)

    cmake_gen_cmd = CMAKE_GEN_CMD.format()
    cmake_build_cmd = CMAKE_BUILD_CMD.format()

    print(cmake_gen_cmd)
    print(cmake_build_cmd)

    ret = os.system(" ".join(cmake_gen_cmd.split("\n")))
    if ret != 0:
        raise Exception("CMake configuration generation failed!")

    ret = os.system(" ".join(cmake_build_cmd.split("\n")))
    if ret != 0:
        raise Exception("CMake build failed!")

    print(colorama.Fore.GREEN + "Compiled firmware successfully" + colorama.Style.RESET_ALL)

def flash_image(image=APPLICATION):
    cmd = FLASH_APPLICATION_CMD.format(image=image)
    print(cmd)

    if not os.path.exists(image):
        raise FileNotFoundError(image)

    ret = os.system(" ".join(cmd.split("\n")))
    if ret != 0:
        msg = "Unable to flash application"
        print(colorama.Fore.RED + msg + colorama.Style.RESET_ALL)
        raise Exception(msg)
    print(colorama.Fore.GREEN + "Application flashed" + colorama.Style.RESET_ALL)

@click.group()
def cli():
    """
    Firmware build tool
    """
    print(colorama.Fore.GREEN + "[CLI]\n" + colorama.Style.RESET_ALL)
    pass

@cli.command()
def compile_bootloader():
    """
    compiles bootloader
    """
    build_bootloader()

@cli.command()
def compile_image():
    """
    compiles image
    """
    build_image()


@cli.command()
def flash_application():
    """
    flashes application
    """
    flash_image()

@cli.command()
@click.option("--bootloader", is_flag=True, help="compile bootloader, default False")
@click.option("--compile", is_flag=True, help="compile image, default False")
@click.option("--flash", is_flag=True, help="flash application, default False")
def build_application(bootloader, compile, flash):
    if bootloader:
        build_bootloader()
    if compile:
        build_image()
    if flash:
        flash_image("cmake-build/baremetal_app.bin")

if __name__ == "__main__":
    cli()
