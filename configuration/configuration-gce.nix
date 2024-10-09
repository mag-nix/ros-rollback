{ config, lib, nixpkgs, nix-ros-overlay, ... }:
{
  # Usage in GCE
  # FIXME: <..> is not the proper way due to impure behavior
  imports = [
    <nixpkgs/nixos/modules/virtualisation/google-compute-image.nix>
    ./ros.nix
  ];

  ros-module.overlay = nix-ros-overlay;

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  system.stateVersion = "24.05";
}
