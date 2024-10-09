{ config, lib, nixpkgs, nix-ros-overlay, ... }:
{
  # Usage in GCE
  # FIXME: <..> is not the proper way due to impure behavior
  imports = [
    <nixpkgs/nixos/modules/virtualisation/google-compute-image.nix>
  ];

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  services.ros = (import ./ros.nix);

  system.stateVersion = "24.05";
}
