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

ssh into vm

``` bash
ssh -p 3022 robotix@127.0.0.1
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

## Deploy using deploy-rs with rollback


