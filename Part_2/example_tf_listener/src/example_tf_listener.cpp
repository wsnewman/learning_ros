//example_tf_listener.cpp:
//wsn, March 2016
//illustrative node to show use of tf listener, with reference to the simple mobile-robot model
// specifically, frames: odom, base_frame, link1 and link2

// this header incorporates all the necessary #include files and defines the class "DemoTfListener"
#include "example_tf_listener.h"
using namespace std;

//main pgm to illustrate transform operations

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "demoTfListener"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type DemoTfListener");
    DemoTfListener demoTfListener(&nh); //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    tf::StampedTransform stfBaseToLink2, stfBaseToLink1, stfLink1ToLink2;
    tf::StampedTransform testStfBaseToLink2;

    tf::Transform tfBaseToLink1, tfLink1ToLink2, tfBaseToLink2, altTfBaseToLink2;

    demoTfListener.tfListener_->lookupTransform("base_link", "link1", ros::Time(0), stfBaseToLink1);
    cout << endl << "base to link1: " << endl;
    demoTfListener.printStampedTf(stfBaseToLink1);
    tfBaseToLink1 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink1);

    demoTfListener.tfListener_->lookupTransform("link1", "link2", ros::Time(0), stfLink1ToLink2);
    cout << endl << "link1 to link2: " << endl;
    demoTfListener.printStampedTf(stfLink1ToLink2);
    tfLink1ToLink2 = demoTfListener.get_tf_from_stamped_tf(stfLink1ToLink2);

    demoTfListener.tfListener_->lookupTransform("base_link", "link2", ros::Time(0), stfBaseToLink2);
    cout << endl << "base to link2: " << endl;
    demoTfListener.printStampedTf(stfBaseToLink2);
    tfBaseToLink2 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink2);
    cout << endl << "extracted tf: " << endl;
    demoTfListener.printTf(tfBaseToLink2);

    altTfBaseToLink2 = tfBaseToLink1*tfLink1ToLink2;
    cout << endl << "result of multiply tfBaseToLink1*tfLink1ToLink2: " << endl;
    demoTfListener.printTf(altTfBaseToLink2);

    if (demoTfListener.multiply_stamped_tfs(stfBaseToLink1, stfLink1ToLink2, testStfBaseToLink2)) {
        cout << endl << "testStfBaseToLink2:" << endl;
        demoTfListener.printStampedTf(testStfBaseToLink2);
    }
    cout << endl << "attempt multiply of stamped transforms in wrong order:" << endl;
    demoTfListener.multiply_stamped_tfs(stfLink1ToLink2, stfBaseToLink1, testStfBaseToLink2);

    geometry_msgs::PoseStamped stPose, stPose_wrt_base;
    stPose = demoTfListener.get_pose_from_transform(stfLink1ToLink2);
    cout << endl << "pose link2 w/rt link1, from stfLink1ToLink2" << endl;
    demoTfListener.printStampedPose(stPose);

    demoTfListener.tfListener_->transformPose("base_link", stPose, stPose_wrt_base);
    cout << endl << "pose of link2 transformed to base frame:" << endl;
    demoTfListener.printStampedPose(stPose_wrt_base);

    return 0;
}