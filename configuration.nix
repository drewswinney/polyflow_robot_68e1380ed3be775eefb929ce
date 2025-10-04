{ config, pkgs, lib, ... }:

let
  user = "admin";
  password = "password";
  interface = "end0";
  hostname = "68e1380ed3be775eefb929ce";
in {
  nixpkgs.overlays = [
    (final: super: {
      makeModulesClosure = x:
        super.makeModulesClosure (x // { allowMissing = true; });
    })
  ];

  imports = [
     "${builtins.fetchGit { url = "https://github.com/NixOS/nixos-hardware.git"; rev="26ed7a0d4b8741fe1ef1ee6fa64453ca056ce113"; }}/raspberry-pi/4"
  ];
  
  boot = {
    kernelPackages = pkgs.linuxKernel.packages.linux_rpi4;
    initrd.availableKernelModules = [ "xhci_pci" "usbhid" "usb_storage" ];
    loader = {
      grub.enable = false;
      generic-extlinux-compatible.enable = true;
    };
  };

  fileSystems = {
    "/" = {
      device = "/dev/disk/by-label/NIXOS_SD";
      fsType = "ext4";
      options = [ "noatime" ];
    };
  };

  system.autoUpgrade.flags = ["--max-jobs" "1" "--cores" "1"];

  networking = {
    hostName = "68e1380ed3be775eefb929ce";
    networkmanager.enable = true;
    nftables.enable = true;
  };

  environment.etc."nixos/configuration.nix" = {
    source = ./configuration.nix;
    mode = "0644";
  };

  environment.systemPackages = with pkgs; with rosPackages.humble; [ vim git wget inetutils ros-base ros-core gh python3 python3Packages.pip colcon ];

  services.openssh.enable = true;

  users = {
    mutableUsers = false;
    users."${user}" = {
      isNormalUser = true;
      password = password;
      extraGroups = [ "wheel" ];
    };
  };

  # Services
  systemd.services.polyflow_startup = {
    description = "Clone the robot git repository and start ROS";
    wantedBy = [ "multi-user.target" ]; # Or a more specific target if needed
    after = [ "network-online.target" ]; # Ensure network is available
    serviceConfig = {
      Type = "oneshot";
      User = "admin";
      Group = "users";
      ExecStart = "${pkgs.writeShellScript "clone-repo" ''
      export HOME=/home/${user}
      DIRECTORY="/home/${user}/polyflow_robot_68e1380ed3be775eefb929ce"
      if [[ -d "$DIRECTORY" ]];
        then
          cd "$DIRECTORY"
          ${pkgs.git}/bin/git pull
        else
          cd /home/${user}
          ${pkgs.git}/bin/git config --global --unset https.proxy
          ${pkgs.git}/bin/git clone https://github.com/drewswinney/polyflow_robot_68e1380ed3be775eefb929ce.git
          chown -R ${user}:users /home/${user}/polyflow_robot_68e1380ed3be775eefb929ce
          
          cd /home/${user}/polyflow_robot_68e1380ed3be775eefb929ce/workspace/src/webrtc
          ${pkgs.python3Packages.pip}/bin/pip install -r requirements.txt

          cd /home/${user}/polyflow_robot_68e1380ed3be775eefb929ce/workspace
          ${pkgs.colcon}/bin/colcon build --packages-select webrtc
          source install/setup.bash

          ${pkgs.rosPackages.humble.ros-core}/bin/roslaunch webrtc webrtc.launch.py
        fi
      ''}";
      StandardError = "inherit"; # Merges stderr with stdout
    };
  };

  services.ros2 = {
    enable = true;
    distro = "humble";
  };

  services.vscode-server.enable = true;

  nix.settings.experimental-features = ["nix-command" "flakes" ];

  hardware.enableRedistributableFirmware = true;
  system.stateVersion = "23.11";
}