#!/usr/bin/env python3
"""Build board-specific firmware and a source bundle using pinned dependencies."""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import tarfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BOARDS = {
    "leonardo": "arduino:avr:leonardo",
    "micro": "arduino:avr:micro",
    "promicro-5v16": "SparkFun:avr:promicro:cpu=16MHzatmega32U4",
}
VERSIONS = {"arduino_cli": "1.5.1", "arduino_avr": "1.8.8", "sparkfun_avr": "1.1.13",
            "HX711_ADC": "1.2.12", "ArduinoJoystickLibrary": "2.1.1"}
JOYSTICK_COMMIT = "12cf2bbdb8910619d32ba3bf4b7d669c8e813b99"
SPARKFUN_INDEX = "https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json"


def run(args, capture=False):
    return subprocess.run([str(arg) for arg in args], check=True, cwd=ROOT,
                          text=True, stdout=subprocess.PIPE if capture else None).stdout


def version():
    return re.search(r'#define PEDALS_VERSION "([^"]+)"',
                     (ROOT / "ArduinoTec-Pedals/version.h").read_text()).group(1)


def validate_release(tag, default_branch="master"):
    if not re.fullmatch(r"v\d+\.\d+\.\d+(?:-(?:rc|beta)\.\d+)?", tag):
        raise ValueError("Use a version tag such as v0.2.0 or v0.2.0-rc.1")
    if tag[1:] != version():
        raise ValueError("Tag must match PEDALS_VERSION in version.h")
    run(["git", "check-ref-format", f"refs/heads/{default_branch}"])
    run(["git", "merge-base", "--is-ancestor", "HEAD", f"origin/{default_branch}"])
    if run(["git", "rev-parse", "HEAD"], True) != run(["git", "rev-parse", f"{tag}^{{commit}}"], True):
        raise ValueError("Build checkout does not match the release tag")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", default="arduino-cli")
    parser.add_argument("--config-file", type=Path)
    parser.add_argument("--setup", action="store_true", help="Download pinned board cores and libraries first")
    parser.add_argument("--release-tag", help="Validate version and default-branch ancestry before building")
    parser.add_argument("--default-branch", default="master")
    args = parser.parse_args()
    if args.release_tag:
        validate_release(args.release_tag, args.default_branch)
    config = args.config_file
    if config is None:
        toolchain = ROOT / "build/toolchain"
        toolchain.mkdir(parents=True, exist_ok=True)
        config = toolchain / "arduino-cli.yaml"
        config.write_text(json.dumps({"directories": {
            "data": str(toolchain / "data"), "downloads": str(toolchain / "downloads"),
            "user": str(toolchain / "user")}}))
    cli = [args.cli, "--config-file", config]
    cli_version = json.loads(run(cli + ["version", "--format", "json"], True))["VersionString"]
    if cli_version != VERSIONS["arduino_cli"]:
        raise ValueError(f"Use Arduino CLI {VERSIONS['arduino_cli']}, found {cli_version}")
    directories = json.loads(run(cli + ["config", "dump", "--format", "json"], True))["config"]["directories"]
    libraries = Path(directories["user"]) / "libraries"
    joystick = libraries / "ArduinoJoystickLibrary"
    if args.setup:
        run(cli + ["core", "update-index", "--additional-urls", SPARKFUN_INDEX])
        run(cli + ["core", "install", "arduino:avr@" + VERSIONS["arduino_avr"]])
        run(cli + ["core", "install", "SparkFun:avr@" + VERSIONS["sparkfun_avr"], "--additional-urls", SPARKFUN_INDEX])
        run(cli + ["lib", "install", "HX711_ADC@" + VERSIONS["HX711_ADC"]])
        if not joystick.exists():
            libraries.mkdir(parents=True, exist_ok=True)
            run(["git", "clone", "--quiet", "--depth", "1", "--branch", "v2.1.1",
                 "https://github.com/MHeironimus/ArduinoJoystickLibrary.git", joystick])
    if run(["git", "-C", joystick, "rev-parse", "HEAD"], True).strip() != JOYSTICK_COMMIT:
        raise ValueError("Joystick library revision does not match the pinned dependency")
    if run(["git", "-C", joystick, "status", "--porcelain"], True).strip():
        raise ValueError("Joystick library has local changes; use a clean toolchain")
    installed = json.loads(run(cli + ["core", "list", "--format", "json"], True))["platforms"]
    cores = {p["id"]: p["installed_version"] for p in installed}
    for core, expected in (("arduino:avr", VERSIONS["arduino_avr"]), ("SparkFun:avr", VERSIONS["sparkfun_avr"])):
        if cores.get(core) != expected:
            raise ValueError(f"Install {core}@{expected} with --setup")
    if f"version={VERSIONS['HX711_ADC']}" not in (libraries / "HX711_ADC/library.properties").read_text().splitlines():
        raise ValueError("HX711_ADC version mismatch; run --setup")

    # A fresh output directory prevents stale artifacts entering a new release.
    output = ROOT / "build/dist"
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    firmware_version = version()
    if (ROOT / ".git").exists():
        commit = run(["git", "rev-parse", "HEAD"], True).strip()
        dirty = bool(run(["git", "status", "--porcelain"], True).strip())
    else:
        # Release source archives deliberately omit Git metadata.
        previous = json.loads((ROOT / "manifest.json").read_text())
        commit, dirty = previous["commit"], True
    manifest = {"version": firmware_version, "commit": commit, "dirty": dirty,
                "dependencies": VERSIONS, "boards": BOARDS,
                "hardware_tested": False,
                "configuration": "3 pedals; saved calibration required; optional brake pot disabled"}
    if args.release_tag and manifest["dirty"]:
        raise ValueError("Release builds require a clean working tree")
    for board, fqbn in BOARDS.items():
        build = ROOT / "build" / board
        run(cli + ["compile", "--fqbn", fqbn, "--output-dir", build, ROOT / "ArduinoTec-Pedals"])
        shutil.copyfile(build / "ArduinoTec-Pedals.ino.hex", output / f"pedals-{board}.hex")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    package = output / f"pedals-{firmware_version}.zip"
    with zipfile.ZipFile(package, "w", zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(output.iterdir()):
            if file.suffix in (".hex", ".json"):
                archive.write(file, file.name)
        for name in ("README.md", "LICENSE", "Circuit.png"):
            archive.write(ROOT / name, name)
        for folder in ("tools", "docs"):
            for file in sorted((ROOT / folder).rglob("*")):
                if file.is_file() and "__pycache__" not in file.parts:
                    archive.write(file, file.relative_to(ROOT))
    # Include library/core sources alongside our sources, with their notices.
    source = output / f"pedals-{firmware_version}-source.tar.gz"
    def exclude_metadata(info):
        return None if ".git" in Path(info.name).parts or "__pycache__" in Path(info.name).parts else info
    with tarfile.open(source, "w:gz") as archive:
        for name in ("ArduinoTec-Pedals", "tools", "tests", "docs", ".github", "README.md", "LICENSE", "Circuit.png", "Circuit.fzz", "CPPLINT.cfg", ".pre-commit-config.yaml", ".gitignore"):
            archive.add(ROOT / name, arcname=name, filter=exclude_metadata)
        archive.add(output / "manifest.json", arcname="manifest.json")
        archive.add(joystick, arcname="vendor/ArduinoJoystickLibrary", filter=exclude_metadata)
        archive.add(libraries / "HX711_ADC", arcname="vendor/HX711_ADC", filter=exclude_metadata)
        for vendor, ver in (("arduino", VERSIONS["arduino_avr"]), ("SparkFun", VERSIONS["sparkfun_avr"])):
            archive.add(Path(directories["data"]) / "packages" / vendor / "hardware/avr" / ver,
                        arcname=f"vendor/{vendor}-avr", filter=exclude_metadata)
    hashes = []
    for file in sorted(output.iterdir()):
        hashes.append(f"{hashlib.sha256(file.read_bytes()).hexdigest()}  {file.name}\n")
    (output / "SHA256SUMS").write_text("".join(hashes))
    print(f"Release files prepared in {output}")


if __name__ == "__main__":
    main()
