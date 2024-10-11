
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

  outputs = { self, nixpkgs, nix-ros-overlay, nixos-generators, deploy-rs, ... }@attrs:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {
    # NixOS System
    nixosConfigurations.nixos-vm = nixpkgs.lib.nixosSystem {
      inherit system;
      specialArgs = attrs;
      modules = [
        ./hardware-configuration/vm-hardware-configuration.nix
        nix-ros-overlay.nixosModules.default
        ./configuration/configuration-vm.nix
      ];
    };

    # Nix Packages
    packages.${system} = {
      # Supported formats at https://github.com/nix-community/nixos-generators
      vbox = nixos-generators.nixosGenerate {
        inherit system;
        specialArgs = attrs;
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration/configuration-vm.nix
        ];
        format = "virtualbox";
      };
      qemu = nixos-generators.nixosGenerate {
        inherit system;
        specialArgs = attrs;
        modules = [
          nix-ros-overlay.nixosModules.default
          ./configuration/configuration-vm.nix
        ];
        format = "qcow";
      };
      install-iso = nixos-generators.nixosGenerate {
        inherit system;
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
        remoteBuild = false;
        autoRollback = false;
        magicRollback = false;
        sshUser = "robotix";
        sshOpts = [ "-p" "3022" ];
        user = "root";
        path = deploy-rs.lib.x86_64-linux.activate.nixos self.nixosConfigurations.nixos-vm;
      };
    };

    # This is highly advised, and will prevent many possible mistakes
    checks = builtins.mapAttrs (system: deployLib: deployLib.deployChecks self.deploy) deploy-rs.lib;

    devShell.${system} = pkgs.mkShell {
      buildInputs = [ pkgs.deploy-rs ];
      inputsFrom = [ ];
    };

  };

  nixConfig = {
    extra-substituters = [
      "https://ros.cachix.org"
      "https://magnix.cachix.org"
    ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "magnix.cachix.org-1:S2LwuWtB3Di2YlymYH8avFhVdiNNWD42uV3eH3VYeGI="
    ];
  };
}
