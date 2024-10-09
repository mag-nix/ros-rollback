{ config, lib, nixpkgs, nix-ros-overlay, ... }:
let
  pkgs = (import nixpkgs) { system = "x86_64-linux"; };
in
{
  imports = [
    ./ros.nix
  ];

  ros-module.overlay = nix-ros-overlay;

  boot.kernelParams = [ "console=tty0" ];

  users.users.robotix = {
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    initialPassword = "test";
  };

  networking.hostName = "nixos";

  services.sshd.enable = true;

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  system.stateVersion = "24.05";
}
