# NixOS ROS Generation & Rollback

## NixOS Generation & Virtualization

- This installs ros and the ros packages declared in `systemPackages` of ros.nix for multiple targets
- Sets up systemd services for the `roscore`, `talker` and `listener`

The targets are:

- Google Cloud Platform
- VirtualBox
- VMWare
- .iso

## Usage of Virtual Box

``` bash
nix build .#vbox
```

- Install Virtual Box
- Import Appliance
- Use `nixos-*-x86_64-linux.ova` file for new vm

![](img/2024-10-09-15-40-29.png)

- Set port forwarding from local host to vm

![](img/2024-10-09-15-39-51.png)

## ssh into vm

``` bash
ssh -p 3022 robotix@127.0.0.1
```

Use the password `robot` defined in `configuration/configuration-vm.nix`

``` nix
initialPassword = "robot";
```

Check the services using

``` bash
systemctl status talker.service
systemctl status listener.service
systemctl status roscore.service
```

Or use ROS for introspection

``` bash
rostopic echo /chatter
```

## Add your public ssh key

The configuration is to be defined in `configuration/configuration-vm.nix`

``` nix
openssh.authorizedKeys.keys = [
    "ssh-ed25519 AAAAbbbzzz name@machine"
];
```

## Deploy to vm using deploy-rs **without** rollbacks

Use `nix flake check` to see whether your configuration is good to go.

Get into a shell with `deploy-rs` installed using `flake.nix`

``` nix
devShell.${system} = pkgs.mkShell {
    buildInputs = [ pkgs.deploy-rs ];
    inputsFrom = [ ];
};
```

``` bash
nix develop
deploy .#local-vm
```

## Deploy to vm using deploy-rs **with** rollbacks

Turn rollbacks on in `flake.nix`

``` nix
autoRollback = true;
magicRollback = true;
```

And break the ssh connection by disabling the ssh daemon service in `configuration/configuration-vm.nix`

``` nix
services.sshd.enable = false;
```

Now you should be able to observe the magic rollback (default: 30 sec)

``` bash
deploy .#local-vm
```

If you like to have prove; turn on monitoring of the ssh service in the virtual box terminal:

``` bash
watch --interval 1 systemctl status sshd.service
```

Watch it disappear and reappear in accordance to the progress of the deployment

## Deploy a new configuration

Lets fix `sshd` by setting it to true again and change ros. For example use different talker and listener:

``` nix
nodes = {
    talker = {
    # ROS Package naming
    package = "rospy_tutorials";
    # ROS Node
    # node = "talker";
    node = "talker_header.py";
    };

    listener = {
    package = "rospy_tutorials";
    # node = "listener";
    node = "listener_header.py";
    };
};
```

Deploy asks nix to send this new configuration to the robot. Nix knows to prepare new systemd services and turn of the no longer needed services. In that way the roscore.service was not restarted since it was not changed (!)

``` bash
deploy .#local-vm
```
