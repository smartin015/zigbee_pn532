"""
PlatformIO extra script — creates symlinks for Zigbee role-specific
libraries that the pioarduino platform does not link automatically.

The stub libespressif__esp-zigbee-lib.a lacks the role-dependent
implementations (esp_zb_init, esp_zb_device_register, …).
Those live in <framework-libs>/<chip>/lib/libesp_zb_api.<role>.a
and the matching ZBOSS stack + port libs.
"""

import os
from os.path import join, isdir, islink, exists

Import("env")


def setup_zigbee_role_libs():
    proj_dir = env.subst("$PROJECT_DIR")
    dest_dir = join(proj_dir, "lib", "zigbee_router")

    # Resolve the framework-libs package path for the current board
    platform = env.PioPlatform()
    fw_libs_dir = platform.get_package_dir("framework-arduinoespressif32-libs")
    chip = env.BoardConfig().get("build.mcu", "").lower()
    src_dir = join(fw_libs_dir, chip, "lib")

    if not isdir(src_dir):
        print(f"zigbee_libs: WARNING — {src_dir} not found, skipping")
        return

    os.makedirs(dest_dir, exist_ok=True)

    # Role-specific libraries needed for ZCZR (Coordinator / Router)
    libs = [
        ("libesp_zb_api.zczr.a", "libesp_zb_api_zczr.a"),
        ("libzboss_stack.zczr.a", "libzboss_stack_zczr.a"),
        ("libzboss_port.native.a", "libzboss_port_native.a"),
    ]

    for src_name, link_name in libs:
        link_path = join(dest_dir, link_name)
        src_path = join(src_dir, src_name)
        if not islink(link_path) and not exists(link_path):
            os.symlink(src_path, link_path)
            print(f"zigbee_libs: linked {link_name} -> {src_path}")


setup_zigbee_role_libs()
