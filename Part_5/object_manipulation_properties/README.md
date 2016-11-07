# object_manipulation_properties

Simple library to encapsulate object ID codes and grasp properties, w/ func to get 
required properties.

Alternative: service to respond to queries re/ object manipulation.

## Example usage
see example main: object_manipulation_properties_test_main.cpp
`rosrun object_manipulation_properties object_manipulation_properties_test_main`

then follow the prompt to enter and object_id and get the manipulation properties

For service (newer):
`rosrun object_manipulation_properties object_manipulation_query_svc`

To test this service, run:
`rosrun object_manipulation_properties example_object_manip_query_client`
refer to codes in "objectManipulationQuery.srv" as well as object codes in "object_ID_codes.h" and
gripper codes in "gripper_ID_codes.h"

See notes in these files re/ how to add functionality (new grippers, new objects)

Intent of service is to return a grasp transform, expressed as object pose w/rt gripper frame.
Multiple options can exist, and client can choose among these options.

Manipulation options include: approach and grasp along gripper-z axis, or approach sliding between fingers;
 can grab from above (e.g. block), or grab from the side (e.g. bottle)
 
 Use this fnc to plan manipulation without reference to a specific manipulator.



    
