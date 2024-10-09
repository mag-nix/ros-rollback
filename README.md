# NixOS ROS systemd

- This installs ros and the ros packages declared in `systemPackages`
- Sets up systemd services for the `roscore`, `talker` and `listener`

## Usage

- Log into NixOS GCE VM

``` bash
git clone https://github.com/mag-nix/ros-systemd.git
sudo nixos-rebuild switch --flake .#nixos-robot-1 --impure
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

## Open points

- [ ] Properly configure and use GCE (currently `--impure` is used)
- [ ] Installation into nix containers might be interesting for separation for robustness and resource limitation
- [ ] Declare multiple ros services instead of one with all nodes and launch files
