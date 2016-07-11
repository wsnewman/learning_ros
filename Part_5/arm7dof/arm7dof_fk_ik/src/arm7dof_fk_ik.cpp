// Arm7dof_fk_ik library implementation file; start w/ fwd kin

#include <arm7dof_fk_ik/arm7dof_kinematics.h> 
using namespace std;


Eigen::Matrix4d compute_A_of_DH(int i, double q_ang) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    double a = DH_a_params[i];
    double d = DH_d_params[i];
    double alpha = DH_alpha_params[i];
    double q = q_ang + DH_q_offsets[i];

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q);
    double sq = sin(q);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    //cout<<"A: "<<endl;
    //cout<<A<<endl;
    return A;
}

Arm7dof_fwd_solver::Arm7dof_fwd_solver() {
    //construct the tool transform from defined constants
    Eigen::Matrix3d R_hand;
    Eigen::Vector3d O_hand;

    O_hand(0) = Lx_hand;
    O_hand(1) = 0.0;
    O_hand(2) = Lz_hand;

    R_hand(0, 0) = cos(theta_yaw_hand);
    R_hand(0, 1) = -sin(theta_yaw_hand); //% - sin(q(i))*cos(alpha);
    R_hand(0, 2) = 0.0; //
    R_hand(1, 0) = -R_hand(0, 1);
    R_hand(1, 1) = R_hand(0, 0); //
    R_hand(1, 2) = 0.0; //%
    R_hand(2, 0) = 0.0;
    R_hand(2, 1) = 0.0;
    R_hand(2, 2) = 1.0;

    // set values for the tool transform, from flange to tool frame  
    //A_tool_to_flange_ = ...;
    A_tool_wrt_flange_.linear() = R_hand;
    A_tool_wrt_flange_.translation() = O_hand;
    A_tool_wrt_flange_inv_ = A_tool_wrt_flange_.inverse();
}

Eigen::Affine3d Arm7dof_fwd_solver::fwd_kin_tool_wrt_base_solve(const Vectorq7x1& q_vec) {
    Eigen::Affine3d A_flange_wrt_base;
    Eigen::Affine3d A_tool_wrt_base;
    A_flange_wrt_base = fwd_kin_flange_wrt_base_solve(q_vec);
    A_tool_wrt_base = A_flange_wrt_base*A_tool_wrt_flange_;
    return A_tool_wrt_base;
}

Eigen::Affine3d Arm7dof_fwd_solver::fwd_kin_flange_wrt_base_solve(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d M;
    M = fwd_kin_solve_(q_vec);
    Eigen::Affine3d A(M);
    return A;
}





Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_point(const Vectorq7x1& q_vec) {
    fwd_kin_solve_(q_vec);
    Eigen::Affine3d A(A_mat_products_[5]);
    Eigen::Vector3d w;
    w = A.translation();
    return w;
}

//this version assumes A matrices have already been computed
Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_point() {
    Eigen::Affine3d A(A_mat_products_[5]);
    Eigen::Vector3d w;
    w = A.translation();
    return w;
}

//fwd kin from frame 1 to wrist pt

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_coords_wrt_frame1(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d A_shoulder_to_wrist;
    fwd_kin_solve_(q_vec);
    A_shoulder_to_wrist = A_mats_[1] * A_mats_[2] * A_mats_[3] * A_mats_[4];
    Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
    return w_wrt_1;
}

//this version assumes A matrices have already been computed--use with care

Eigen::Vector3d Arm7dof_fwd_solver::get_wrist_coords_wrt_frame1() {
    Eigen::Matrix4d A_shoulder_to_wrist;
    //fwd_kin_solve_(q_vec);
    A_shoulder_to_wrist = A_mats_[1] * A_mats_[2] * A_mats_[3] * A_mats_[4];
    Eigen::Vector3d w_wrt_1 = A_shoulder_to_wrist.block<3, 1>(0, 3);
    return w_wrt_1;
}

//Jacobian, 6x7
//angular part is just b axis of each frame;
//translational part depends on cross products, including vec from frame origin to endpoint
// and joint-axis vector

Eigen::MatrixXd Arm7dof_fwd_solver::Jacobian(Eigen::VectorXd q_vec) {
    Eigen::MatrixXd Jacobian(6,7); // = Eigen::Zeros(6,7);  
    //Eigen::Affine3d affine_flange; 
    Eigen::MatrixXd Origins(3,7);
    Eigen::Matrix4d A4x4_flange;
    Eigen::MatrixXd J_ang(3,7), J_trans(3,7);
    Eigen::Vector3d zvec, Oi,wvec,rvec;

    //affine_flange = fwd_kin_flange_wrt_base_solve(q_vec); //compute all the A matrices and their products
    A4x4_flange = fwd_kin_solve_(q_vec);
    wvec= A4x4_flange.block<3,1>(0,3); // get vector from base to endpoint


    //compute the angular Jacobian, using z-vecs from each frame; first frame is just [0;0;1]
    zvec << 0, 0, 1;
    //cout<<"setting first ang vec in J_ang: "<<endl;
    J_ang.block<3, 1>(0, 0) = zvec; // and populate J_ang with them; at present, this is not being returned
    //cout<<"setting first origin: "<<endl;
    Oi << 0, 0, 0;
    Origins.block<3, 1>(0, 0) = Oi;
    for (int i = 1; i < 7; i++) {
        zvec = A_mat_products_[i - 1].block<3, 1>(0, 2); //%strip off z axis of each previous frame; note subscript slip  
        J_ang.block<3, 1>(0, i) = zvec; // and populate J_ang with them;
        Oi = A_mat_products_[i - 1].block<3, 1>(0, 3); //origin of i'th frame
        Origins.block<3, 1>(0, i) = Oi;
    }
    //now, use the zvecs to help compute J_trans
    for (int i=0;i<7;i++) {
        zvec = J_ang.block<3, 1>(0, i); //%recall z-vec of current axis     
        Oi =Origins.col(i); //block<3, 1>(0, i); //origin of i'th frame
        rvec = wvec - Oi; //%vector from origin of i'th frame to wrist pt 
        //Rvecs.block<3, 1>(0, i) = rvec; //save these?
        //t1 = zvecs.block<3, 1>(0, i);
        //t2 = rvecs.block<3, 1>(0, i);
        J_trans.block<3, 1>(0, i) = zvec.cross(rvec);  
        //cout<<"frame "<<i<<": zvec = "<<zvec.transpose()<<"; Oi = "<<Oi.transpose()<<endl;
    }
    //assemble into combined Jacobian:
    Jacobian.block<3,7>(0,0) = J_trans;
    Jacobian.block<3,7>(3,0) = J_ang;
    return Jacobian;
}


//Wrist Jacobian:  
/*
Eigen::Matrix3d Arm7dof_fwd_solver::get_wrist_Jacobian_3x3(double q_s1, double q_humerus, double q_elbow, double q_forearm) {
    Vectorq7x1 q_vec;
    for (int i=0;i<7;i++) q_vec(i)=0.0;
    q_vec(1) = q_s1;
    q_vec(2) = q_humerus;
    q_vec(3) = q_elbow;
    q_vec(4) = q_forearm;
    
    Eigen::Matrix4d A_mats_3dof[5];
    Eigen::Matrix4d A_mat_products_3dof[5];

    //Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Ai;
    Eigen::Matrix3d R;
    //Eigen::Vector3d p,t1,t2;;
    Eigen::Matrix3d Jw1_trans;
    Eigen::Matrix3d Jw1_ang;
    Eigen::Matrix3d Origins;
    //Eigen::Matrix3d Rvecs;
    Eigen::Vector3d zvec,rvec,wvec,Oi;
    //populate 5 A matrices and their products; need 5 just to get to wrist point, but can assume q_forearm=0, q_wrist_bend=0
    // note--starting from S1 frame, skipping frame 0
    for (int i=0;i<5;i++) {
        A_mats_3dof[i] = compute_A_of_DH(i+1, q_vec(i+1));
    }
        
    A_mat_products_3dof[0] = A_mats_3dof[0];
    //cout<<"A_mat_products_3dof[0]"<<endl;
    //cout<<A_mat_products_3dof[0]<<endl;
    for (int i=1;i<5;i++) {
        A_mat_products_3dof[i] = A_mat_products_3dof[i-1]*A_mats_3dof[i];
    }
    wvec = A_mat_products_3dof[4].block<3, 1>(0, 3); //strip off wrist coords
    //cout<<"wvec w/rt frame1: "<<wvec.transpose()<<endl;
    
    //compute the angular Jacobian, using z-vecs from each frame; first frame is just [0;0;1]
    zvec<<0,0,1;
    Jw1_ang.block<3, 1>(0, 0) = zvec; // and populate J_ang with them; at present, this is not being returned
    Oi<<0,0,0;
    Origins.block<3, 1>(0, 0) = Oi;
    for (int i=1;i<3;i++) {
        zvec = A_mat_products_3dof[i-1].block<3, 1>(0, 2); //%strip off z axis of each previous frame; note subscript slip  
        Jw1_ang.block<3, 1>(0, i) = zvec; // and populate J_ang with them;
        Oi = A_mat_products_3dof[i-1].block<3, 1>(0, 3); //origin of i'th frame
        Origins.block<3, 1>(0, i) = Oi;
    }    
    //now, use the zvecs to help compute J_trans
    for (int i=0;i<3;i++) {
        zvec = Jw1_ang.block<3, 1>(0, i); //%recall z-vec of current axis     
        Oi =Origins.block<3, 1>(0, i); //origin of i'th frame
        rvec = wvec - Oi; //%vector from origin of i'th frame to wrist pt 
        //Rvecs.block<3, 1>(0, i) = rvec; //save these?
        //t1 = zvecs.block<3, 1>(0, i);
        //t2 = rvecs.block<3, 1>(0, i);
        Jw1_trans.block<3, 1>(0, i) = zvec.cross(rvec);  
        //cout<<"frame "<<i<<": zvec = "<<zvec.transpose()<<"; Oi = "<<Oi.transpose()<<endl;
    }     
    //cout<<"J_ang: "<<endl;
    //cout<<Jw1_ang<<endl;
    return Jw1_trans;
}
 */

//inner fwd-kin fnc: computes tool-flange frame w/rt base
//return soln out to tool flange; would still need to account for tool transform for gripper

Eigen::Matrix4d Arm7dof_fwd_solver::fwd_kin_solve_(const Vectorq7x1& q_vec) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    for (int i = 0; i < 7; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec[i]);
        A_mats_[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats_[i] << std::endl;
    }

    A_mat_products_[0] = A_mats_[0];

    for (int i = 1; i < 7; i++) {
        A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
    }

    return A_mat_products_[6]; //tool flange frame
}

//equiv fnc that accepts VectorXd type input
Eigen::Matrix4d Arm7dof_fwd_solver::fwd_kin_solve_(Eigen::VectorXd q_vec) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    for (int i = 0; i < 7; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec[i]);
        A_mats_[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats_[i] << std::endl;
    }

    A_mat_products_[0] = A_mats_[0];

    for (int i = 1; i < 7; i++) {
        A_mat_products_[i] = A_mat_products_[i - 1] * A_mats_[i];
    }

    return A_mat_products_[6]; //tool flange frame
}


//-------------------------IK methods---------------------------------

Arm7dof_IK_solver::Arm7dof_IK_solver() {
    //constructor: 
    //ROS_INFO("Arm7dof_IK_solver constructor");
}

//given angle of shoulder yaw (turret), compute the origin of frame2

Eigen::Vector3d Arm7dof_IK_solver::get_frame2_origin_of_shoulder_yaw(double q_yaw) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:

    Eigen::Matrix4d A_1_wrt_0, A_2_wrt_1, A_2_wrt_0;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
    A_2_wrt_1 = compute_A_of_DH(1, 0.0); //origin of frame 2 does not depend on q2
    A_2_wrt_0 = A_1_wrt_0*A_2_wrt_1;
    Eigen::Vector3d O2_wrt_base = A_2_wrt_0.block<3, 1>(0, 3);
    return O2_wrt_base;
}

//compute elbow solutions given wrist pt w/rt base frame and q_yaw (q0)
//return "true" if at least 1 viable soln
//solutions are put in vector q_elbow_solns

bool Arm7dof_IK_solver::solve_for_elbow_ang(Eigen::Vector3d w_wrt_0, double q_yaw, std::vector<double> &q_elbow_solns) {
    Eigen::Vector3d O2_wrt_base;
    O2_wrt_base = get_frame2_origin_of_shoulder_yaw(q_yaw);
    // see eqn 19 of An, Clement, Reed, aim2014
    Eigen::Vector3d wrist_wrt_2;
    wrist_wrt_2 = w_wrt_0 - O2_wrt_base;

    double d25 = wrist_wrt_2.norm(); //dist from shoulder to wrist
    //cout<<"d25 = "<<d25<<endl;
    double den = 2.0 * DH_d3*DH_d5;
    double num = d25 * d25 - (DH_d4)*(DH_d4) - DH_d3 * DH_d3 - DH_d5*DH_d5;
    //test viability here...
    double c4 = num / den;
    if (c4 > 1.0) {
        ROS_WARN("w_des out of reach at full elbow extension, q_yaw = %f", q_yaw);
        return false;
    }
    //cout<<"num, den, c4 = "<<num<<", "<<den<<", "<<c4<<endl;
    double s4 = sqrt(1 - c4 * c4);
    //cout<<"s4= "<<s4<<endl;
    double q4a = atan2(s4, c4);
    //cout<<"q4a = "<<q4a<<endl;
    q_elbow_solns.clear();
    // test limits DH_q_max4>= q4 >= DH_q_min4
    bool valid_soln = false;
    if (fit_q_to_range(q_lower_limits[3], q_upper_limits[3], q4a)) {
        valid_soln = true;
        q_elbow_solns.push_back(q4a);
        //cout<<"q4a = "<<q4a<<endl;
    }
    double q4b = atan2(-s4, c4);
    if (fit_q_to_range(q_lower_limits[3], q_upper_limits[3], q4b)) {
        valid_soln = true;
        q_elbow_solns.push_back(q4b);
        //cout<<"q4b = "<<q4b<<endl;
    }
    return valid_soln;
}

//solve the eqn r = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid
bool Arm7dof_IK_solver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns) {
    double r, cphi, sphi, phi, gamma;
    double KTOL = 0.000001; //arbitrary tolerance
    r = sqrt(A * A + B * B);
    phi = atan2(B, A);
    q_solns.clear();
    if (fabs(K) > fabs(r)) {
        ROS_WARN("K/r is too large for a cos/sin soln"); 
        return false; //if |K/r|>1, no solns
    }
    //could still have trouble w/ K=0...
    if (fabs(K) < KTOL) {
        ROS_WARN("K is too small for A,B,K trig soln: user error? "); 
        return false; //illegal use of this fnc
    }
    gamma = acos(K / r);
    double soln1 = phi + gamma;
    double soln2 = phi - gamma;
    q_solns.push_back(soln1);
    q_solns.push_back(soln2);
    //test/DEBUG
    /*
    cout<<"K = "<<K<<endl;
    cout<<"soln1="<<soln1<<endl;
    double test = A*cos(soln1) + B*sin(soln1);
    cout<<"Acos(q1) + Bsin(q1) = "<<test<<endl;
    cout<<"soln2="<<soln2<<endl;
    test = A*cos(soln2) + B*sin(soln2);
    cout<<"Acos(q2) + Bsin(q2) = "<<test<<endl; 
     */
    return true;
}

//given qmin, qmax and q, coerce q into a periodic soln between q_min and q_max, if possible
//return "false" if not possible
bool Arm7dof_IK_solver::fit_q_to_range(double q_min, double q_max, double &q) {
    //cout<<"fit_q_to_range: q_min = "<<q_min<<", q_max = "<<q_max<<", q_in = "<<q<<endl;
    while (q < q_min) {
        q += 2.0 * M_PI;
    }
    while (q > q_max) {
        q -= 2.0 * M_PI;
    }
    //cout<<"q_fit = "<<q<<endl;
    if (q < q_min)
        return false;
    else
        return true;
}

//provide q_yaw and q_elboe and desired wrist point; solve for q_humerus
bool Arm7dof_IK_solver::solve_for_humerus_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_elbow, std::vector<double> &q_humerus_solns) {
    // z_wrist_wrt_1 = -d5*s3*s4 - c3*d4 +d2
    Eigen::Matrix4d A_1_wrt_0, A_w_wrt_1;
    Eigen::Matrix3d R;
    Eigen::Vector3d p, w_wrt_1;
    double SOLN_TOL_Z = 0.001; // tolerance for confirming soln
    A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
    Eigen::Affine3d affine_1_wrt_0(A_1_wrt_0);
    w_wrt_1 = affine_1_wrt_0.inverse() * w_wrt_0;
    //cout<<"w_wrt_0= "<<w_wrt_0.transpose()<<endl;
    //cout<<"w_wrt_1= "<<w_wrt_1.transpose()<<endl;

    double w_z_wrt_1 = w_wrt_1[2];
    //cout<<"w_z_wrt_1 = "<<w_z_wrt_1<<endl;    
    double A, B, K;
    K = w_z_wrt_1 - DH_d2;
    A = DH_d4;
    B = DH_d5 * sin(q_elbow);
    std::vector<double> q_humerus_temp;
    q_humerus_temp.clear();
    bool valid_soln = solve_K_eq_Acos_plus_Bsin(K, A, B, q_humerus_temp);
    if (!valid_soln) return false; //give up
    //have 1 or 2 solns in q_humerus_temp; test them;
    valid_soln = false;
    //cout<<"consider "<<q_humerus_temp.size()<<" humerus angle candidates"<<endl;
    q_humerus_solns.clear();
    double q_test = q_humerus_temp[0];
    bool does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_test);
    if (does_fit) { // look further:
        // z_wrist_wrt_1 = -d5*s3*s4 - c3*d4 +d2
        double w_z_test = A * cos(q_test) + B * sin(q_test);
        //double w_z_test = -DH_d5*sin(q_test)*sin(q_elbow)-DH_d4*cos(q_test)+DH_d2;
        //double w_z_err = fabs(w_z_test - w_z_wrt_1);
        double w_z_err = fabs(w_z_test - K);
        //cout<<"w_z_err = "<<w_z_err<<endl;
        if (w_z_err < SOLN_TOL_Z) {
            //cout<<"saving humerus soln q3= "<<q_test<<endl;
            q_humerus_solns.push_back(q_test);
            valid_soln = true; //have at least one good soln
        }
    }
    // check if another soln to eval:
    if (q_humerus_temp.size() > 1) {
        q_test = q_humerus_temp[1];
        does_fit = fit_q_to_range(q_lower_limits[2], q_upper_limits[2], q_test);
        if (does_fit) { // look further:
            double w_z_test = DH_d5 * sin(q_elbow) * sin(q_test) + DH_d4 * cos(q_test) + DH_d2;
            double w_z_err = fabs(w_z_test - w_z_wrt_1);
            //cout<<"w_z_err for 2nd q3= "<<w_z_err<<endl;
            if (w_z_err < SOLN_TOL_Z) {
                //cout<<"saving soln q3 = "<<q_test<<endl;
                q_humerus_solns.push_back(q_test);
                valid_soln = true; //have at least one good soln
            }
        }
    }
    return valid_soln;
}

//solve for joint2 (q[1]):  need to provide q[0],q[2],q[3]
bool Arm7dof_IK_solver::solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus, double q_elbow, std::vector<double> &q_shoulder_solns) {
    // z_wrist_wrt_1 = -d5*s3*s4 - c3*d4 +d2
    Eigen::Matrix4d A_1_wrt_0, A_w_wrt_1;
    Eigen::Matrix3d R;
    Eigen::Vector3d p, w_wrt_1;
    double SOLN_TOL_Z = 0.001; // tolerance for confirming soln
    A_1_wrt_0 = compute_A_of_DH(0, q_yaw);
    Eigen::Affine3d affine_1_wrt_0(A_1_wrt_0);
    w_wrt_1 = affine_1_wrt_0.inverse() * w_wrt_0;
    //cout<<"w_wrt_0= "<<w_wrt_0.transpose()<<endl;
    //cout<<"w_wrt_1= "<<w_wrt_1.transpose()<<endl;

    double w_x_wrt_1 = w_wrt_1[0];
    //cout<<"w_x_wrt_1 = "<<w_x_wrt_1<<endl;   
    //w_x_wrt_1 = -d5 c2 c3 s4 -d5 s2 c4 + d4 c2 s3 -d3 s2
    //   = (-d5*c3*s4+d4*s3)*c2 + (-d5*c4 -d3)*s2
    double A, B, K; //solve K = A*cos(q) + B*sin(q) for q
    K = w_x_wrt_1;
    A = -DH_d5 * cos(q_humerus) * sin(q_elbow) + DH_d4 * sin(q_humerus); //-d5*c3*s4+d4*s3
    B = -DH_d5 * cos(q_elbow) - DH_d3; //(-d5*c4 -d3)
    std::vector<double> q_shoulder_temp;
    q_shoulder_temp.clear();
    bool valid_soln = solve_K_eq_Acos_plus_Bsin(K, A, B, q_shoulder_temp);
    if (!valid_soln) return false; //give up
    //have 1 or 2 solns in q_shoulder_temp; test them;
    valid_soln = false;
    //cout<<"consider "<<q_shoulder_temp.size()<<" shoulder pitch angle candidates"<<endl;
    q_shoulder_solns.clear();
    double q_test = q_shoulder_temp[0];
    bool does_fit = fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_test);
    if (does_fit) { // look further:
        // z_wrist_wrt_1 = -d5*s3*s4 - c3*d4 +d2
        double w_x_test = A * cos(q_test) + B * sin(q_test);
        //double w_z_test = -DH_d5*sin(q_test)*sin(q_elbow)-DH_d4*cos(q_test)+DH_d2;
        //double w_z_err = fabs(w_z_test - w_z_wrt_1);
        double w_x_err = fabs(w_x_test - K);
        //cout<<"w_x_err = "<<w_x_err<<endl;
        if (w_x_err < SOLN_TOL_Z) {
            //cout<<"saving shoulder soln q2= "<<q_test<<endl;
            q_shoulder_solns.push_back(q_test);
            valid_soln = true; //have at least one good soln
        }
    }
    // check if another soln to eval:
    if (q_shoulder_temp.size() > 1) {
        q_test = q_shoulder_temp[1];
        does_fit = fit_q_to_range(q_lower_limits[1], q_upper_limits[1], q_test);
        if (does_fit) { // look further:
            //double w_x_test = DH_d5*sin(q_elbow)*sin(q_test)+DH_d4*cos(q_test)+DH_d2;
            double w_x_test = -DH_d5 * cos(q_test) * cos(q_humerus) * sin(q_elbow)
                    + DH_d4 * cos(q_test) * sin(q_humerus)
                    - DH_d3 * sin(q_test); //-d5 c2 c3 s4 -d5 s2 c4 + d4 c2 s3 -d3 s2
            double w_x_err = fabs(w_x_test - w_x_wrt_1);
            //cout<<"w_x_err for 2nd q2= "<<w_x_err<<endl;
            if (w_x_err < SOLN_TOL_Z) {
                // cout<<"saving soln q2 = "<<q_test<<endl;
                q_shoulder_solns.push_back(q_test);
                valid_soln = true; //have at least one good soln
            }
        }
    }
    return valid_soln;
}


//here is a main IK fnc: 
//given q_yaw and given coords of desired wrist pt w/rt base, find all of the solutions q0,q1,q2,q3
// typically 4 solns?
// solns are tested for being within joint ranges

bool Arm7dof_IK_solver::ik_wrist_solns_of_q0(Eigen::Vector3d wrist_pt, double q_yaw, std::vector<Eigen::VectorXd> &q_solns) {
    bool valid_wrist_soln = false;
    bool valid_q_elbow = false;
    bool valid_q_humerus = false;
    bool valid_q_shoulder = false;  
    double q_elbow, q_humerus, q_shoulder;    
    std::vector<double> q_elbow_solns;
    std::vector<double> q_humerus_angs;
    std::vector<double> q_shoulder_pitch_angs;    
    
    Eigen::VectorXd q_vec_test; //, q_vec_err;
    Vectorq7x1 q_vec7x1;
    //cout<<"ik_wrist_solns..."<<endl;

    q_vec7x1 << 0, 0, 0, 0, 0, 0, 0;
    q_vec_test = q_vec7x1;

    
    q_elbow_solns.clear();
    q_solns.clear();
    //cout<<"computing elbow solns..."<<endl;
            
    valid_q_elbow = solve_for_elbow_ang(wrist_pt, q_yaw, q_elbow_solns);

    if (!valid_q_elbow) return false;
    //if here, have at least 1 elbow solution...drill deeper

    Eigen::Vector3d wrist_pt_test_soln, w_soln_err;

    if (q_elbow_solns.size() > 0) {
        //cout << "num elbow solns = " << q_elbow_solns.size() << endl;
        //consider the first elbow option:
        q_elbow = q_elbow_solns[0];
        //cout << "q_elbow_soln_a = " << q_elbow << endl;
        q_vec_test[3] = q_elbow; //test w/ this elbow soln; solve for q_humerus   
        q_humerus_angs.clear();
        valid_q_humerus = solve_for_humerus_ang(wrist_pt, q_yaw, q_elbow, q_humerus_angs);
        if (valid_q_humerus) {
            q_humerus = q_humerus_angs[0]; //consider first humerus option
            //cout << "q_humerus_soln_a=" << q_humerus << endl;
            q_shoulder_pitch_angs.clear();
            valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
            if (valid_q_shoulder) { //at least one good shoulder angle
                //cout << "found " << q_shoulder_pitch_angs.size() << " shoulder solns" << endl;
                q_shoulder = q_shoulder_pitch_angs[0]; //consider the first option
                q_vec_test[0] = q_yaw;
                q_vec_test[1] = q_shoulder;
                q_vec_test[2] = q_humerus;
                q_vec_test[3] = q_elbow;
                wrist_pt_test_soln = get_wrist_point(q_vec_test);
                //cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                w_soln_err = wrist_pt - wrist_pt_test_soln;
                
                //cout << w_soln_err.transpose() << endl;
                //cout << "q: " << q_vec_test.transpose() << endl;
                if (w_soln_err.norm() < W_ERR_TOL) {
                    //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                    q_solns.push_back(q_vec_test);
                    valid_wrist_soln = true;
                }
                else ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                //another shoulder option?
                   if (q_shoulder_pitch_angs.size() > 1) {
                        q_shoulder = q_shoulder_pitch_angs[1];
                        q_vec_test[1] = q_shoulder;
                        wrist_pt_test_soln = get_wrist_point(q_vec_test);
                        //cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        //
                        if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                        }       
                        else ROS_WARN("wrist soln err: %f",w_soln_err.norm());
                    }                
            } //q_shoulder tests          
               if (q_humerus_angs.size() > 1) { //still first elbow soln, but 2nd humerus soln:
                    q_humerus = q_humerus_angs[1];
                    q_vec_test[2] = q_humerus;
                    //cout << endl << "q_humerus_soln_b=" << q_humerus << endl;
                    valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                    if (valid_q_shoulder) {             
                        q_shoulder = q_shoulder_pitch_angs[0];
                        q_vec_test[1] = q_shoulder;
                        wrist_pt_test_soln = get_wrist_point(q_vec_test);
                        //cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                        }
                        //test 2nd shoulder soln:
                        if (q_shoulder_pitch_angs.size() > 1) {
                            q_shoulder = q_shoulder_pitch_angs[1];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = get_wrist_point(q_vec_test);
                            //cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                            if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                            }
                        } //done 2nd shoulder soln test
                    } //done checking shoulder solns for 2nd humerus soln 
               }  //done w/ 1st elbow soln options (4 cases)
        } //end q_humerus tests       
    //check if have another elbow option to explore:
    if (q_elbow_solns.size() > 1) {
                q_elbow = q_elbow_solns[1];
                //cout << "q_elbow_soln_b = " << q_elbow << endl;
                q_humerus_angs.clear();
                valid_q_humerus = solve_for_humerus_ang(wrist_pt, q_yaw, q_elbow, q_humerus_angs);
                if (valid_q_humerus) { //this elbow ang has q_humerus options, so explore deeper
                    q_humerus = q_humerus_angs[0];
                    //cout << "q_humerus_soln_a=" << q_humerus << endl;
                    q_shoulder_pitch_angs.clear();
                    //solve_for_shoulder_pitch_ang(Eigen::Vector3d w_wrt_0, double q_yaw, double q_humerus, double q_elbow, std::vector<double> &q_shoulder_solns)
                    valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                    if (valid_q_shoulder) { //test solns w/ FK:
                        //cout << "found " << q_shoulder_pitch_angs.size() << " shoulder solns" << endl;
                        q_shoulder = q_shoulder_pitch_angs[0];
                        q_vec_test[0] = q_yaw;
                        q_vec_test[1] = q_shoulder;
                        q_vec_test[2] = q_humerus;
                        q_vec_test[3] = q_elbow;
                        wrist_pt_test_soln = get_wrist_point(q_vec_test);
                        //cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                        w_soln_err = wrist_pt - wrist_pt_test_soln;
                        if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                        } 
                        //test 2nd shoulder soln, if it exists:
                        if (q_shoulder_pitch_angs.size() > 1) {
                            q_shoulder = q_shoulder_pitch_angs[1];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = get_wrist_point(q_vec_test);
                            //cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                            if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                            } 
                        }
                    }
                    if (q_humerus_angs.size() > 1) {
                        q_humerus = q_humerus_angs[1];
                        //cout << "q_humerus_soln_b=" << q_humerus << endl;
                        q_vec_test[2] = q_humerus;
                        q_shoulder_pitch_angs.clear();
                        valid_q_shoulder = solve_for_shoulder_pitch_ang(wrist_pt, q_yaw, q_humerus, q_elbow, q_shoulder_pitch_angs);
                        if (valid_q_shoulder)  { //test solns w/ FK:               
                            q_shoulder = q_shoulder_pitch_angs[0];
                            q_vec_test[1] = q_shoulder;
                            wrist_pt_test_soln = get_wrist_point(q_vec_test);
                            //cout << "soln: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                            w_soln_err = wrist_pt - wrist_pt_test_soln;
                           if (w_soln_err.norm() < W_ERR_TOL) {
                                //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                q_solns.push_back(q_vec_test);
                                valid_wrist_soln = true;
                            }                             
                            //test 2nd shoulder soln:
                            if (q_shoulder_pitch_angs.size() > 1) {
                               q_shoulder = q_shoulder_pitch_angs[1];
                                q_vec_test[1] = q_shoulder;
                                wrist_pt_test_soln = get_wrist_point(q_vec_test);
                                //cout << "shoulder soln2: wrist pt wrt frame0: " << wrist_pt_test_soln.transpose() << endl;
                                w_soln_err = wrist_pt - wrist_pt_test_soln;
                                if (w_soln_err.norm() < W_ERR_TOL) {
                                   //if here, then current q0,q1,q2,q3 solution satisfies the wrist goal; save it;
                                   q_solns.push_back(q_vec_test);
                                   valid_wrist_soln = true;
                                 }                                  
                            } // done testing 2nd shoulder soln
                        }//done w/ both shoulder solns
                    }//done w/ 2nd humerus soln
                }//done w/ both humerus solns
    } // done checking 2nd elbow soln
    } //end q_elbow tests
    return valid_wrist_soln;
}

// compute the wrist point from the desired flange frame--independent of what
// reference frame is used for the flange frame.  Wrist coords will be in the same reference frame
Eigen::Vector3d Arm7dof_IK_solver::wrist_pnt_from_flange_frame(Eigen::Affine3d affine_flange_frame) {
    Eigen::Vector3d flange_z_axis;
    Eigen::Vector3d flange_origin;
    Eigen::Vector3d wrist_pt;
    Eigen::Matrix3d R_flange=affine_flange_frame.linear();
    
    flange_origin = affine_flange_frame.translation(); 
    flange_z_axis = R_flange.col(2); 
    wrist_pt = flange_origin-flange_z_axis*DH_d7;
    return wrist_pt;
}

//find the wrist solns to fit desired orientation, given q0 through q3.
// provide q0 through q3 in q_in; fnc will populate q_solns w/ 0, 1 or 2 complete 7-dof solns
// note: if q5 (wrist bend) is near zero, then at a wrist singularity; 
// inf solutions of q4+D, q6-D
// use q0, q1, q2, q3 from q_in; copy these values to q_solns, and tack on the two solutions q4, q5, q6
bool Arm7dof_IK_solver::solve_spherical_wrist(Vectorq7x1 q_in,Eigen::Matrix3d R_des, std::vector<Vectorq7x1> &q_solns) {
    bool is_singular = false;
    bool at_least_one_valid_soln = false;
    Eigen::Matrix4d A01,A12,A23,A04;
    Eigen::Matrix4d  A45,A05,A56,A06;
    //A01 = compute_A_of_DH(0, q_in[0]);
    //A12 = compute_A_of_DH(1, q_in[1]);
    //A23 = compute_A_of_DH(2, q_in[2]);
    //A03 = A01*A12*A23;  
    fwd_kin_solve_(q_in);
    A04 = A_mat_products_[3];
    Eigen::Vector3d n6,t6,b6; //axes of frame6; b6 is same as b_des
    Eigen::Vector3d n5,t5,b5; // axes of frame5;    
    Eigen::Vector3d n4,t4,b4; // axes of frame4

    Eigen::Vector3d n_des,b_des; // desired x-axis and z-axis of flange frame
    n4 = A04.col(0).head(3);
    t4 = A04.col(1).head(3);    
    b4 = A04.col(2).head(3);  
    b_des = R_des.col(2);
    n_des = R_des.col(0);
    b5 = b4.cross(b_des);
    double q4,q5,q6; //these are: q_forearm, q_wrist_bend, q_tool_flange_spin
    Vectorq7x1 q_soln;
    q_solns.clear();
    
      if (b5.norm() <= 0.001) { // CHOOSE A SINGULARITY TOLERANCE, e.g. 0.001 rad
                q4=0;
                is_singular = true; // do something with this value
                ROS_WARN("WRIST SINGULARITY!");
      }
      else {
            double cq4= b5.dot(-t4);
            double sq4= b5.dot(n4); 
            q4= atan2(sq4, cq4);
        }
    
    // choose the positive-most forearm-rotation solution;
    // there should be another soln, 180-deg from here
    while (q4<q_upper_limits[4] ) {
        q4+= M_PI;
    }
    q4 -= M_PI; // this should be the most positive q4 soln
    if (q4<q_lower_limits[4]) {
        ROS_WARN("no q4 solns"); // this should never happen
        return false;
    }

    // use the + q4 soln to find q5, q6
    A45 = compute_A_of_DH(4, q4);
    A05 = A04*A45;
    n5 = A05.col(0).head(3);
    t5 = A05.col(1).head(3); 
    double cq5 = b_des.dot(t5);
    double sq5 = b_des.dot(-n5);
    q5 = atan2(sq5,cq5); //+DH_q_offsets[5]; // 
    //std::cout<<"wrist bend = "<<q5<<std::endl;

    //solve for q6
    A56 = compute_A_of_DH(5, q5);
    A06 = A05*A56;
    n6 = A06.col(0).head(3);
    t6 = A06.col(1).head(3);   
        
    double cq6=n_des.dot(-n6);
    double sq6=n_des.dot(-t6);
    q6 =atan2(sq6, cq6)+M_PI;
    //ROS_INFO("q4,q5,q6 = %f, %f, %f",q4,q5,q6);
    if (fit_q_to_range(q_lower_limits[4],q_upper_limits[4],q4)) {
        if (fit_q_to_range(q_lower_limits[5],q_upper_limits[5],q5)) {
            if (fit_q_to_range(q_lower_limits[6],q_upper_limits[6],q6)) {
                q_soln = q_in;
                q_soln[4] = q4;
                q_soln[5] = q5;
                q_soln[6] = q6;
                q_solns.push_back(q_soln);
                at_least_one_valid_soln = true;   
            }
        }
    }

    //2nd wrist soln: 
    q4 -= M_PI;
    q5 *= -1.0; // flip wrist opposite direction
    q6 += M_PI; // fix the periodicity later; 
    if (fit_q_to_range(q_lower_limits[4],q_upper_limits[4],q4)) {
        if (fit_q_to_range(q_lower_limits[5],q_upper_limits[5],q5)) {
            if (fit_q_to_range(q_lower_limits[6],q_upper_limits[6],q6)) {
                q_soln = q_in;
                q_soln[4] = q4;
                q_soln[5] = q5;
                q_soln[6] = q6;
                q_solns.push_back(q_soln);
                at_least_one_valid_soln = true;   
            }
        }
    }    
    
       // ROS_INFO("alt q4,q5,q6 = %f, %f, %f",q_soln[3],q_soln[4],q_soln[5]);
       
    return is_singular;
}

// compute all reachable 7dof solns for specified q_yaw and desired tool-flange pose
// expect from 0 to 8 solutions at given q_yaw
//APPEND these to q_solns
//but return the number of solutions at THIS q_yaw
int Arm7dof_IK_solver::ik_solve_given_qs0(Eigen::Affine3d const& desired_flange_pose_wrt_base,double q_yaw, std::vector<Vectorq7x1> &q_solns) {
    Eigen::Matrix3d Rdes = desired_flange_pose_wrt_base.linear();
    Eigen::Vector3d wrist_pt= wrist_pnt_from_flange_frame(desired_flange_pose_wrt_base);
    std::vector<Vectorq7x1>  q_solns_w_wrist;
    std::vector<Eigen::VectorXd> q_wristpt_solns;
    Vectorq7x1 q_soln;
    bool valid_wrist_solns=false;
    int num_solns = 0;
    //q_solns.clear(); //no!  just append!
    
    //compute all solutions q0,q1,q2,q3 to satisfy wrist-point requirement:
    valid_wrist_solns= ik_wrist_solns_of_q0(wrist_pt, q_yaw, q_wristpt_solns);
    if (!valid_wrist_solns) {
        ROS_WARN("no wrist-point solns for this pose and q_yaw= %f",q_yaw);
        return 0;
    }
    
    int n_wrist_pt_solns = q_wristpt_solns.size();
    //now, compute corresponding orientation solns and add successful results to list of 7dof solns:   
    for (int i=0; i< n_wrist_pt_solns; i++)
    {
        q_soln = q_wristpt_solns[i];
        //cout<<"q_soln123: "<<q_soln.transpose()<<endl;
        bool singular_wrist = solve_spherical_wrist(q_soln,Rdes, q_solns_w_wrist); 
        for (int j=0;j<q_solns_w_wrist.size();j++) {
            q_solns.push_back(q_solns_w_wrist[j]); //accumulate all of the valid 7-dof solns
            num_solns++;
        }
        //if singular solns, should augment num solns here, q4+q6 = const
        //if (singular_wrist) {
        // }
    }          
    //cout<<"there are "<<q_solns.size()<<" solutions with wrist options"<<endl;
    return num_solns; // return number of solutions found at THIS q_yaw            
}

//computed IK solns over samples of q0, sampled at resolution DQ_YAW
int Arm7dof_IK_solver::ik_solns_sampled_qs0(Eigen::Affine3d const& desired_flange_pose_wrt_base,std::vector<Vectorq7x1> &q_solns) {
       q_solns.clear();
        int n7dof_solns;
        for (double q_yaw_samp = q_lower_limits[0]; q_yaw_samp < q_upper_limits[0]; q_yaw_samp += DQ_YAW) {
            //cout << "calling ik_solve_given_qs0" << endl;
            n7dof_solns= ik_solve_given_qs0(desired_flange_pose_wrt_base, q_yaw_samp, q_solns);
            cout << "num solns found = " << n7dof_solns << " at q_yaw = " << q_yaw_samp << endl; 
        }
   return  q_solns.size();
}
