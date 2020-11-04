# Alex-Open-Source-Bipedal-Robot
"There is no too short, too tall, too heavy, too warm, too wet, or too humid. There is just one excuse: too weak."

Installation Instructions
1. Clone the repository including submodules
git clone --recurse-submodules -j8 https://github.com/HJW012/Alex-Open-Source-Bipedal-Robot

2. Move the 'Packages' folder to src folder in catkin workspace

3. catkin_make to build all packages

Troubleshooting
- Some packages require dependencies that may not be install on your machine. These can be installed manually or by using the following command from your catkin_ws directory.
rosdep install -i -y --from-paths "PATH TO PACKAGE"

- If running on a slow machine and encountering lock-ups when using catkin_make, run the catkin_make command sequentially rather than in parallel. This can be done with the following command in your catkin_ws directory. This will take much longer to build, but shouldn't lock-up/crash
catkin_make -j1

- User does not haev permission to access Orientus IMU. In this case, the current user must be added to the linux 'dialout' group to allow access to USB ports. This can be done by running the following command.
sudo adduser "USERNAME" dialout