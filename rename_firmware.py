Import("env")  # noqa: F821
import re, shutil, os
from datetime import datetime

def get_version_from_source():
    with open("include/version.h", "r") as f:
        match = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', f.read())
        return match.group(1) if match else "0.0.0"

version = get_version_from_source()
project_name = env.GetProjectOption("custom_name", env["PIOENV"])
build_time = datetime.now().strftime("%Y%m%d_%H%M%S")

# Generate build_info.h
with open("include/build_info.h", "w") as f:
    f.write('#pragma once\n')
    f.write(f'#define FIRMWARE_VERSION  "{version}"\n')
    f.write(f'#define BUILD_TIMESTAMP   "{build_time}"\n')
    f.write(f'#define PROJECT_NAME      "{project_name}"\n')

def rename_firmware(source, target, env):
    src = str(target[0])

    # Timestamped copy in build dir
    build_name = f"{project_name}_v{version}_{build_time}.bin"
    build_copy = os.path.join(os.path.dirname(src), build_name)
    shutil.copy(src, build_copy)
    print(f"Firmware build:   {build_copy}")

    # Version-only copy in releases folder (one file per version, safe to commit)
    release_name = f"{project_name}_v{version}.bin"
    releases_dir = os.path.join(env["PROJECT_DIR"], "releases")
    os.makedirs(releases_dir, exist_ok=True)
    release_copy = os.path.join(releases_dir, release_name)
    shutil.copy(src, release_copy)
    print(f"Firmware release: {release_copy}")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", rename_firmware)