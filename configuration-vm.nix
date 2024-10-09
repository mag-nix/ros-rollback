{ config, lib, nixpkgs, nix-ros-overlay, ... }:
{
  users.users.nixosvmtest.isSystemUser = true ;
  users.users.nixosvmtest.initialPassword = "test";
  users.users.nixosvmtest.group = "nixosvmtest";
  users.groups.nixosvmtest = {};

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  services.ros = (import ./ros.nix);

  system.stateVersion = "24.05";
}
