
//derived class

class RobotSpecificFK : public FwdSolver {
private:
    UR10FwdSolver ur10FwdSolver_;  //instantiate a solver specific to target robot
    Eigen::Affine3d affine_fk_;
    Eigen::MatrixXd jacobian_;
public:
    //int dummy();
    //RobotSpecificFK(): FwdSolver() {};
    //~RobotSpecificFK() {};

    Eigen::Affine3d fwd_kin_solve(Eigen::VectorXd const& q_vec) {
        return (ur10FwdSolver_.fwd_kin_solve(q_vec)); //call the robot-specific fk fnc
    };

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q_vec) {
        return (ur10FwdSolver_.jacobian(q_vec)); //call the robot-specific jacobian fnc
    }
};

class RobotSpecificIK : public IKSolver {
private:
    UR10IkSolver ur10IkSolver_;  //instantiate the robot-specific solver object
    Eigen::Affine3d affine_fk_;
public:

    RobotSpecificIK() {
    }; //constructor
    //~RobotSpecificIK() {};

    int ik_solve(Eigen::Affine3d const& desired_hand_pose, std::vector<Eigen::VectorXd> &q_ik_solns) {
        return (ur10IkSolver_.ik_solve(desired_hand_pose, q_ik_solns)); //call the robot-specific IK solver
    }
    //do-nothing fnc, since soln is analytic, don't need numerical refinement
    void ik_refine(std::vector<Eigen::Affine3d> cartesian_affine_samples, std::vector<Eigen::VectorXd> &optimal_path) {
        return;
    };
};

//these are global--but convenient to put them here:
const int njnts = 6;

//instantiate an object of derived class:
RobotSpecificFK robotSpecificFK;
RobotSpecificIK robotSpecificIK;
//have FwdSolver point to functions defined in derived class
FwdSolver * pFwdSolver = &robotSpecificFK;
IKSolver * pIKSolver = &robotSpecificIK;

