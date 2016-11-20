# part_fetcher
This package is intended to support an action server, part_fetcher, that
accepts single-part fetch requests.  The request is specified in the goal message
defined in this package, PartFetcher.action, and includes a part ID, a current part pose,
and a desired part pose.  The action server should invoke the necessary actions to fetch
the specified part from the origin and place it at the desired destination.

## Example usage
A dummy action-server interface is defined:
`rosrun part_fetcher part_fetcher_exmpl`
It responds to an example client:
`rosrun part_fetcher example_part_fetcher_client`

The example part fetcher should incorporate code from object_grabber/block_grabber_action_client.cpp in order
to convert the part request into commanded manipulation actions.

    
