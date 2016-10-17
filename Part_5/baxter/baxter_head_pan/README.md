# baxter_head_pan
Example code to control head pan motion of Baxter.

## Example usage
demo program, prompts user for amplitude and frequency, and commands head pan
to oscillate indefinitely at specified amplitude and frequency.  Start up Baxter 
simulator with:
`roslaunch baxter_gazebo baxter_world.launch`

Wait for the simulation to finish launching, then enable the motors with:
`rosrun baxter_tools enable_robot.py -e`

Then run one of the head-pan control nodes, e.g.:
`rosrun baxter_head_pan baxter_head_pan`
respond to the prompts with amplitude/frequency, then observe the head oscillating in Gazebo.

Alternatively, to send head pan to zero angle, run:
`rosrun baxter_head_pan baxter_head_pan_zero`
This node runs to completion after approx 1 sec.
    