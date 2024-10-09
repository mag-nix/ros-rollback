
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # Usage of different nixpkgs for fixes and adaptions
  };

  outputs = { self, nixpkgs, nix-ros-overlay, nixos-generators, ... }@attrs: {
    nixosConfigurations.nixos-robot-1 = nixpkgs.lib.nixosSystem {
      system = "x86_64-linux";
      specialArgs = attrs;
      modules = [
        nix-ros-overlay.nixosModules.default
        ./configuration-gce.nix
      ];
    };
    packages.x86_64-linux = {
      vmware = nixos-generators.nixosGenerate {
        system = "x86_64-linux";
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration-vm.nix
        ];
        format = "vmware";
      };
      vbox = nixos-generators.nixosGenerate {
        system = "x86_64-linux";
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration-vm.nix
        ];
        format = "virtualbox";
      };
    };
  };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
