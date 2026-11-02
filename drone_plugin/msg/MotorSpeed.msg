# Commanded speed for each rotor, consumed by the drone Gazebo plugin.
# name[i] is a rotor link name in the URDF (prop_1 .. prop_4);
# velocity[i] is its signed angular speed (rad/s). The sign gives the spin
# direction: thrust is always upward, reaction torque follows the sign.
string[] name
float32[] velocity
