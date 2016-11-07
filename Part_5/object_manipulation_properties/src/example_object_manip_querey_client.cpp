//example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <example_ros_service/ExampleServiceMsg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_ros_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::ExampleServiceMsg>("lookup_by_name");
    example_ros_service::ExampleServiceMsg srv;
    bool found_on_list = false;
    string in_name;
    while (ros::ok()) {
        cout<<endl;
        cout << "enter a name (x to quit): ";
        cin>>in_name;
        if (in_name.compare("x")==0)
            return 0;
        //cout<<"you entered "<<in_name<<endl;
        srv.request.name = in_name; //"Ted";
        if (client.call(srv)) {
            if (srv.response.on_the_list) {
                cout << srv.request.name << " is known as " << srv.response.nickname << endl;
                cout << "He is " << srv.response.age << " years old" << endl;
                if (srv.response.good_guy)
                    cout << "He is reported to be a good guy" << endl;
                else
                    cout << "Avoid him; he is not a good guy" << endl;
            } else {
                cout << srv.request.name << " is not in my database" << endl;
            }

        } else {
            ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    }
    return 0;
}
