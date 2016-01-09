# example_action_server

This example illustrates how to design an action server and an action client pair.

The action server "example_action_server" implements a "simple" action server.  This example is documented in the document "Introduction to action servers and clients:  designing your own action server."

The complementary "example_action_client" can work with any of the action servers.


## Example usage
Start up roscore (or gazebo).  Execute:
`rosrun example_action_server example_action_server` to start the server, and:
`rosrun example_action_server example_action_client` to start the client.

You can monitor the communications with:
`rostopic echo example_action/goal`
and
`rostopic echo example_action/result`


## Running tests/demos
Start the server and the client, in either order (though client will time out if you wait too long to start the server).  The display output should
show that the client and server agree on how many requests have been serviced.  However, if you halt the client and restart it, the client service
count will be out of sync with the server's service count.  For this demo, this mis-match causes the server to halt (for debug purposes only).
