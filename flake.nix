{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    vscode-server.url = "github:nix-community/nixos-vscode-server";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; 
  };

  outputs = { self, nixpkgs, nix-ros-overlay, vscode-server, ... }@inputs:
    let
        system = "aarch64-linux";
        pkgs = import nixpkgs { 
          inherit system; 
          overlays = [ nix-ros-overlay.overlays.default ]; 
        };

    in { 
      devShells.${system}.default = pkgs.mkShell {
        name = "Polyflow";
        buildInputs=[pkgs.bashInteractive];
        packages = [
          pkgs.colcon
          # ... other non-ROS packages
          (with pkgs.rosPackages.humble; buildEnv {
            paths = [
              ros-core
              # ... other ROS packages
            ];
          })
        ];
        shellHook = ''
          echo "Welcome to the Polyflow ROS environment!"
          export SHELL=${nixpkgs.lib.getExe pkgs.bashInteractive}
        '';
      };

      nixosConfigurations."68e1380ed3be775eefb929ce" = nixpkgs.lib.nixosSystem {
          system = system;
          modules = [
            # Base NixOS modules
            ./configuration.nix 
            vscode-server.nixosModules.default
            nix-ros-overlay.nixosModules.default
            # Add the nix-ros-overlay to your system overlays
            # You may also need to include nixos-hardware for specific Raspberry Pi 4 hardware support
            # nixos-hardware.nixosModules.raspberry-pi-4 
          ];
          # Further configuration specific to your Raspberry Pi and ROS needs
        };
        substituters = https://ros.cachix.org;
        trusted-public-keys = cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=;
    };
}