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

  users.users."robotix" = {
    isNormalUser = true;
    extraGroups = [ "wheel" ];
    initialPassword = "robot";
    openssh.authorizedKeys.keys = [
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAII85Hgi5bFo5FAkZh4CxQoyWk4f7AoxpUawXnmuQWJUI jeising@pdemu1cml000342"
    ];
    packages = [
      # Additional packages for robotix user
    ];
  };

  services.getty.autologinUser = "robotix";

  networking.hostName = "nixos";

  networking.useDHCP = false;
  networking.interfaces.ens2.ipv4.addresses = [ {
    address = "192.168.122.10";
    prefixLength = 24;
  } ];

  services.sshd.enable = true;

  nix.settings = {
    experimental-features = [ "nix-command" "flakes" ];
    trusted-users = [ "robotix" ];
  };

  security.sudo.wheelNeedsPassword = false;

  system.stateVersion = "24.05";
}
