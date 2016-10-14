// wsn: program to illustrate use of Eigen library to fit a plane to a collection of points

#include<ros/ros.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
//using namespace Eigen; //if you get tired of typing Eigen:: everywhere, uncomment this.
                         // but I'll leave this as required, for now, to highlight when Eigen classes are being used
    double g_noise_gain = 0.1; //0.1; //0.1; //0.1; //decide how much noise to add to points; start with 0.0, and should get precise results

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_eigen_plane_fit"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ros::Rate sleep_timer(1.0); //a timer for desired rate, e.g. 1Hz

    //xxxxxxxxxxxxxxxxxx  THIS PART IS JUST TO GENERATE DATA xxxxxxxxxxxxxxxxxxxx
    //xxxxxxxxxxxxxxxxxx  NORMALLY, THIS DATA WOULD COME FROM TOPICS OR FROM GOALS xxxxxxxxx
    // define a plane and generate some points on that plane
    // the plane can be defined in terms of a normal vector and a distance from the origin
    Eigen::Vector3d normal_vec(1,2,3); // here is an arbitrary normal vector, initialized to (1,2,3) upon instantiation
    ROS_INFO("creating example noisy, planar data...");
    cout<<"normal: "<<normal_vec.transpose()<<endl; //.transpose() is so I can display the components on the same line, instead of as a column
    normal_vec/=normal_vec.norm(); // make this vector unit length
    cout<<"unit length normal: "<<normal_vec.transpose()<<endl;
    double dist = 1.23;  // define the plane to have this distance from the origin.  Note that if we care about positive/negative sides of a plane,
                         // then this "distance" could be negative, as measured from the origin to the plane along the positive plane normal
    cout<<"plane distance from origin: "<<dist<<endl;
    
    // to generate points on the plane, construct a pair of axes perpendicular to the plane normal
    Eigen::Vector3d v1,v2; //need to fill in values for these
    // we'll use an Eigen "Matrix" type to help out.  Define a 3x3 "double precision" matrix, Matrix3d
    //define a rotation about the z axis of 90 deg; elements of this matrix are:
    // [0,-1,0; 
    //  1, 0, 0; 
    //  0;0;1]
    Eigen::Matrix3d Rot_z;
    Rot_z.row(0)<<0,-1,0;  // populate the first row--shorthand method
    Rot_z.row(1)<<1, 0,0;  //second row
    Rot_z.row(2)<<0, 0,1;  // yada, yada
    cout<<"Rot_z: "<<endl;  

    cout<<Rot_z<<endl;  // Eigen matrices and vectors are nicely formatted; better: use ROS_INFO_STREAM() instead of cout

    ROS_INFO_STREAM(endl<<Rot_z);
    // we need another vector to generate two desired vectors in our plane.
    // start by generating a new vector that is a rotation of our normal vector, rotated about the z-axis
    // this hack will NOT work if our normal_vec = [0,0,1], 
    v1 = Rot_z*normal_vec; //here is how to multiply a matrix times a vector
        //although Rot_z and normal_vec are both objects (with associated data and member methods), multiplication is defined,
        // resulting in the data members being altered or generated as expected for matrix-vector multiplies
    ROS_INFO_STREAM("v1: "<<v1.transpose()<<endl);


    // let's look at the dot product between v1 and normal_vec, using two approaches
    double dotprod = v1.dot(normal_vec); //using the "dot()" member function
    double dotprod2 = v1.transpose()*normal_vec;// alt: turn v1 into a row vector, then multiply times normal_vec

    cout<<"v1 dot normal: "<<dotprod<<"; v1.transpose()*normal_vec: "<<dotprod2<<endl; //yields the same answer
    cout<<"(should be identical)"<<endl;
    
    // let's look at the cross product, v1 X normal_vec; use the member fnc cross()
    v2 = v1.cross(normal_vec);
    v2/=v2.norm(); // normalize the output, i.e. make v2 unit length
    cout<<"v2: "<<v2.transpose()<<endl;
    
    //because v2 was generated from the cross product of v1 and normal_vec, it should be perpendicular to both
    // i.e., the dot product with v1 or normal_vec should be = 0
    dotprod = v2.dot(normal_vec);
    cout<<"v2 dot normal_vec = "<<dotprod<<"  (should be zero)"<<endl;

    v1 = v2.cross(normal_vec);  // re-use v1; make it the cross product of v2 into normal_vec
      // thus, v1 should now also be perpendicular to normal_vec (and thus it is parallel to our plane)
      // and also perpendicular to v2 (so both v1 and v2 lie in the plane and are perpendicular to each other)
    cout<<"v1= "<<v1.transpose()<<endl;
    cout<<" v1 dot v2 = "<<v1.dot(v2)<<"; v1 dot normal_vec = "<<v1.dot(normal_vec)<<endl;
    cout<<"(these should also be zero)"<<endl;
    // we now have two orthogonal vectors, both perpendicular to our defined plane's normal
    // we'll use these to generate some points that lie in our defined plane:
        
    int npts= 10; // create this many planar points
    Eigen::MatrixXd points_mat(3,npts);  // create a matrix, double-precision values, 3 rows and npts columns
            // we will populate this with 3-D points, column by column
    Eigen::Vector3d point; //a 3x1 vector
    Eigen::Vector2d rand_vec; //a 2x1 vector
    //generate random points that all lie on plane defined by distance and normal_vec
    for (int ipt = 0;ipt<npts;ipt++) {
    	// MatrixXd::Random returns uniform random numbers in the range (-1, 1).
    	rand_vec.setRandom(2,1);  // populate 2x1 vector with random values
    	//cout<<"rand_vec: "<<rand_vec.transpose()<<endl; //optionally, look at these random values
    	//construct a random point ON the plane normal to normal_vec at distance "dist" from origin:
        // a point on the plane is a*x_vec + b*y_vec + c*z_vec, where we may choose
        // x_vec = v1, y_vec = v2 (both of which are parallel to our plane) and z_vec is the plane normal
        // choose coefficients a and b to be random numbers, but "c" must be the plane's distance from the origin, "dist"
    	point =  rand_vec(0)*v1 + rand_vec(1)*v2 + dist*normal_vec;
	//save this point as the i'th column in the matrix "points_mat"
	points_mat.col(ipt) = point;
    }

    //all of the above points are identically on the plane defined by normal_vec and dist

    cout<<"random points on plane (in columns): "<<endl; // display these points; only practical for relatively small number of points
    cout<<points_mat<<endl;
    
    
    // add random noise to these points in range [-0.1,0.1]
    Eigen::MatrixXd Noise = Eigen::MatrixXd::Random(3,npts);

    cout<<"noise_gain = "<<g_noise_gain<<"; edit this as desired"<<endl;
    // add two matrices, term by term.  Also, scale all points in a matrix by a scalar: Noise*g_noise_gain
    points_mat = points_mat + Noise*g_noise_gain;  
    cout<<"random points on plane (in columns) w/ noise: "<<endl;
    cout<<points_mat<<endl;
    //xxxxxxxxxxxxxxxxxx  DONE CREATING PLANAR DATA xxxxxxxxxxxxxxxxxx
    // xxxxxxxxxxxxxxx   NOW, INTERPRET THE DATA TO DISCOVER THE UNDERLYING PLANE xxxxxxxx

    //now let's see if we can discover the plane from the data:
    cout<<endl<<endl;
    ROS_INFO("starting identification of plane from data: ");
    // first compute the centroid of the data:
    // here's a handy way to initialize data to all zeros; more variants exist
    // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    Eigen::Vector3d centroid = Eigen::MatrixXd::Zero(3,1);
    
    //add all the points together:
    npts = points_mat.cols(); // number of points = number of columns in matrix; check the size
    cout<<"matrix has ncols = "<<npts<<endl;
    for (int ipt =0;ipt<npts;ipt++) {
	centroid+= points_mat.col(ipt); //add all the column vectors together
    }
    centroid/=npts; //divide by the number of points to get the centroid
    cout<<"centroid: "<<centroid.transpose()<<endl;
    
    
    // subtract this centroid from all points in points_mat:
    Eigen::MatrixXd points_offset_mat = points_mat;
    for (int ipt =0;ipt<npts;ipt++) {
        points_offset_mat.col(ipt)  = points_offset_mat.col(ipt)-centroid;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3d CoVar;
    CoVar = points_offset_mat*(points_offset_mat.transpose());  //3xN matrix times Nx3 matrix is 3x3
    cout<<"covariance: "<<endl;
    cout<<CoVar<<endl;
    
    // here is a more complex object: a solver for eigenvalues/eigenvectors;
    // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
    Eigen::EigenSolver<Eigen::Matrix3d> es3d(CoVar);
    
    Eigen::VectorXd evals; //we'll extract the eigenvalues to here
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    cout<<"(these should be real numbers, and one of them should be zero)"<<endl;
    cout << "The matrix of eigenvectors, V, is:" << endl;
    cout<< es3d.eigenvectors() << endl << endl;
    cout<< "(these should be real-valued vectors)"<<endl;
    // in general, the eigenvalues/eigenvectors can be complex numbers
    //however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
    // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

    evals= es3d.eigenvalues().real(); // grab just the real parts
    cout<<"real parts of evals: "<<evals.transpose()<<endl;

    // our solution should correspond to an e-val of zero, which will be the minimum eval
    //  (all other evals for the covariance matrix will be >0)
    // however, the solution does not order the evals, so we'll have to find the one of interest ourselves
    
    double min_lambda = evals[0]; //initialize the hunt for min eval
    Eigen::Vector3cd complex_vec; // here is a 3x1 vector of double-precision, complex numbers
    Eigen::Vector3d est_plane_normal;
    complex_vec=es3d.eigenvectors().col(0); // here's the first e-vec, corresponding to first e-val
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    est_plane_normal = complex_vec.real();  //strip off the real part
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal=0;
    //loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
    for (int ivec=1;ivec<3;ivec++) {
        lambda_test = evals[ivec];
    	if (lambda_test<min_lambda) {
		min_lambda =lambda_test;
                i_normal= ivec; //this index is closer to index of min eval
		est_plane_normal = es3d.eigenvectors().col(ivec).real();
        }
    }
    // at this point, we have the minimum eval in "min_lambda", and the plane normal
    // (corresponding evec) in "est_plane_normal"/
    // these correspond to the ith entry of i_normal
    cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    cout<<"corresponding evec (est plane normal): "<<est_plane_normal.transpose()<<endl;
    cout<<"correct answer is: "<<normal_vec.transpose()<<endl;
    double est_dist = est_plane_normal.dot(centroid);
    cout<<"est plane distance from origin = "<<est_dist<<endl;
    cout<<"correct answer is: "<<dist<<endl;
    cout<<endl<<endl;
           
    
    //xxxx  one_vec*dist = point.dot(nx,ny,nz)
    // so, one_vec = points_mat.transpose()*x_vec, where x_vec = [nx;ny;nz]/dist (does not work if dist=0)
    // this is of the form: b = A*x, an overdetermined system of eqns
    // solve this using one of many Eigen methods
    // see: http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    
    ROS_INFO("2ND APPROACH b = A*x SOLN");
    Eigen::VectorXd  ones_vec= Eigen::MatrixXd::Ones(npts,1); // this is our "b" vector in b = A*x
    Eigen::MatrixXd A = points_mat.transpose(); // make this a Nx3 matrix, where points are along the rows
    // we'll pick the "full pivot LU" solution approach; see: http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    // a matrix in "Eigen" has member functions that include solution methods to this common problem, b = A*x
    // use: x = A.solution_method(b)
    Eigen::Vector3d x_soln = A.fullPivLu().solve(ones_vec);
    //cout<<"x_soln: "<<x_soln.transpose()<<endl;
    double dist_est2 = 1.0/x_soln.norm();
    x_soln*=dist_est2;
    cout<<"normal vec, 2nd approach: "<<x_soln.transpose()<<endl;
    cout<<"plane distance = "<<dist_est2<<endl;
    
    

    return 0;

   // while (ros::ok()) {
   //     sleep_timer.sleep();
   // }
}

