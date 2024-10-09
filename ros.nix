enable = true;
distro = "noetic";

masterUri = "http://localhost:11311/";
hostname = "localhost";

overlays = [
  nix-ros-overlay.overlays.default
];

# Nix Package naming
systemPackages = p: with p; [ rosbash roslaunch rostopic rospy-tutorials ];

nodes = {
  talker = {
    # ROS Package naming
    package = "rospy_tutorials";
    # ROS Node
    node = "talker";
  };

  listener = {
    package = "rospy_tutorials";
    node = "listener";
  };
};