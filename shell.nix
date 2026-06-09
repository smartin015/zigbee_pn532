# FHS environment for PlatformIO — enter with:
#   nix-shell --no-sandbox --extra-experimental-features flakes
# or, for a single command:
#   nix-shell -p platformio --no-sandbox --extra-experimental-features flakes --run "pio run"

{ pkgs ? import (builtins.fetchTarball {
    url = "https://github.com/NixOS/nixpkgs/archive/nixos-unstable.tar.gz";
  }) {}
}:

(pkgs.buildFHSEnv {
  name = "platformio-fhs";
  targetPkgs = pkgs: with pkgs; [
    platformio
    python3
    udev
  ];
  runScript = "bash";
}).env
