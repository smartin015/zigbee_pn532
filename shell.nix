{ pkgs ? import <nixpkgs> {} }:

(pkgs.buildFHSEnv {
  name = "platformio-fhs";
  targetPkgs = pkgs: with pkgs; [
    platformio
    python3
    udev
  ];
  runScript = "bash";
}).env
