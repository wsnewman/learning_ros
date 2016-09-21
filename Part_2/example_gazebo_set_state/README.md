# example_gazebo_set_state
A simple node that shows how to communicate with Gazebo to set the state of a model.
This node assumes the model in exmpl_models/rect_prism has been loaded into Gazebo.  
Running this node performs a single service call to /gazebo/set_model_state, then returns.
The service call causes the model "rect_prism" to
be initialized to a specified pose, plus initialized to a specified twist.  For a non-zero twist, the model
will continue to move in Gazebo (unless collisions and friction bring the object to rest).

## Example usage

Start Gazebo with:
`roslaunch gazebo_ros empty_world.launch`
An empty world will appear in a Gazebo window.

In this Gazebo window, in the "world" tab, select "physics" and set the gravity z component to 0.  This will
remove gravity, emulating free-fall conditions.

Add a simple model (a rectangular prism) with the example launch file:
`roslaunch exmpl_models add_rect_prism.launch`

A rectangular prism will appear, hovering above the ground plane (with gravity turned off--else it
will fall to the ground).

Under the Gazebo "world" tab, expand "Models", which will show "rect_prism" as the model just added.  The prism
model properties can be seen by expanding elements of the model in this sub-window.

The gazebo service "set_model_state" can be used to change the state (position and twist) of a named model.
This can be done from the command line, e.g. using:
`rosservice call /gazebo/set_model_state '{model_state: {model_name: rect_prism, pose: {position:{z: 4.0}}}}'`
which moves the model up to a height of 4m.  The model can be commanded to rotate about its z axis at 1 rad/sec with:
`rosservice call /gazebo/set_model_state '{model_state: {model_name: rect_prism, twist: {angular:{z: 1.0}}}}'`

Equivalently, run a node to invoke this service call:
`rosrun example_gazebo_set_state example_gazebo_set_prism_state`
This will cause the prism to have an initial angular velocity and translational velocity.  Running:
`rostopic echo gazebo/model_states`
displays that the model "rect_prism" has a twist corresponding to an x-velocity of 0.02 m/s and an angular velocity about z of 1.0 rad/sec.

Adding an additional model--a cylinder--can be performed with:
`roslaunch exmpl_models add_cylinder.launch`
With the prism translating and rotating, the prism and the cylinder will eventually collide, resulting in changes
to the linear and angular momenta of both objects.

A set of N (currently N=11) blocks can be added to a world with the launch file:
`roslaunch exmpl_models add_blocks.launch`
The blocks will be known as "block0", "block1", etc.

The node:
`rosrun example_gazebo_set_state set_block_state`
will start a node with a service that will place named (by number) blocks at (constrained) random poses.
E.g., command with:
`rosservice call set_block_state 5`
which will place block5 in a randomized pose (chosen to be on the cafe table, reachable by Baxter).

    
