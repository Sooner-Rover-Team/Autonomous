# Autonomous Code

## IMPORTANT

Almost all of this documentation below is no longer relevant.

## Installation

Follow [this link](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) to set up ROS 2 (Iron) locally.

Then run:

```bash
source /opt/ros/iron/setup.bash
```

## Building

Run this commmand to install locally. The `--symlink-install` parameter allows python files to be edited without having to run the build command again.

```bash
colcon build --symlink-install
```

Then run this command to be able to run the correct packages:

```bash
source install/setup.bash
```


### Documentation

This following SoRo components are good candidates for documentation. If you know markdown, and have a good idea about what's going on here, please feel free to [make a new page about one of these in the docs](https://sooner-rover-team.github.io/soro-documentation/html/new-page-guide.html)! :)
