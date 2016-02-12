# mobot_gazebo_state
This node is a stand-in for a robot localization system, to be used for 
learning and code-development purposes only.  It assumes a mobile-robot model
called "mobot" is known to gazebo.  It subscribes to the topic
gazebo/model_states.  It searches the list of model names to find a match to "mobot".
It extracts the corresponding Pose from the model states, and re-publishes this
pose to the topic "gazebo_mobot_pose."  

A steering algorithm can consult this topic for use in feedback, relative to
some path of interest.

## Example usage
`rosrun mobot_gazebo_state mobot_gazebo_state`

    
