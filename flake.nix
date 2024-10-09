
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    # Generation for multiple targets
    nixos-generators = {
      url = "github:nix-community/nixos-generators";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
    # For accessing `deploy-rs`'s utility Nix functions
    deploy-rs.url = "github:serokell/deploy-rs";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # Usage of different nixpkgs for fixes and adaptions
  };

  outputs = { self, nixpkgs, nix-ros-overlay, nixos-generators, deploy-rs, ... }@attrs: {

    # NixOS Systems
    nixosConfigurations.nixos-robot-1 = nixpkgs.lib.nixosSystem {
      system = "x86_64-linux";
      specialArgs = attrs;
      modules = [
        nix-ros-overlay.nixosModules.default
        ./configuration/configuration-gce.nix
      ];
    };
    nixosConfigurations.nixos-vm = nixpkgs.lib.nixosSystem {
      system = "x86_64-linux";
      specialArgs = attrs;
      modules = [
        ./hardware-configuration/vm-hardware-configuration.nix
        nix-ros-overlay.nixosModules.default
        ./configuration/configuration-vm.nix
      ];
    };

    # Nix Packages
    packages.x86_64-linux = {
      vmware = nixos-generators.nixosGenerate {
        system = "x86_64-linux";
        specialArgs = attrs;
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration/configuration-vm.nix
        ];
        format = "vmware";
      };
      vbox = nixos-generators.nixosGenerate {
        system = "x86_64-linux";
        specialArgs = attrs;
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration/configuration-vm.nix
        ];
        format = "virtualbox";
      };
      install-iso = nixos-generators.nixosGenerate {
        system = "x86_64-linux";
        specialArgs = attrs;
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration/configuration-iso.nix
        ];
        format = "install-iso";
      };
    };

    deploy.nodes.local-vm = {
      hostname = "localhost";
      profiles.system = {
        sshUser = "robotix";
        sshOpts = [ "-p" "3022" ];
        path = deploy-rs.lib.x86_64-linux.activate.nixos self.nixosConfigurations.nixos-vm;
      };
    };

    # This is highly advised, and will prevent many possible mistakes
    checks = builtins.mapAttrs (system: deployLib: deployLib.deployChecks self.deploy) deploy-rs.lib;
  };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
