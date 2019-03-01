/*
**   ikin.cpp
**
**   Basic node that only provides the inverse kinematics computation.
**
**   Service:   /ikin
*/
#include "ros/ros.h"
#include "moveto/IKin.h"


/*
**   Length Definitions
**
**   In general, we could/should pull these from the URDF...
*/
#define LENA    (0.15748)
#define LENB    (0.15748)
#define LENC    (0.18542)


/*
**   Iterative Inverse Kinematics
*/
#include <Eigen/Dense>

using namespace Eigen;

#define ITERATIONS  (15)
#define EPSILON     (0.01)
#define MAXJNTSTEP  (20.0 * M_PI/180.0)

bool ikin_iterative(double x, double y, double z, double pitch, double roll,
		    double qsolution[5])
{
  int i;                                // Iteration counter

  // Joint values
  Matrix<double, 5, 1> q;               // Current estimate of joint values
  Matrix<double, 5, 1> qe;              // Joint error/delta (step)

  // Kinematic values (1-5 = joints 1-5, t=tip, d=desired)
  Matrix3d R1, R2, R3, R4, R5, Rt;      // Rotations expressed in base frame
  Vector3d e1, e2, e3, e4, e5;          // Joint axes expressed in base frame
  Vector3d p1, p2, p3, p4, p5, pt, pd;  // Locations expressed in base frame
  Vector3d                     nt, nd;  // Normal expressed in base frame

  // Combined Tip values (position and orientation)
  Matrix<double,6,5> J;                 // Jacobian expressed in base frame
  Matrix<double,6,1> xe;                // Tip error/delta exp. in base frame

  // "P" = Prime versions rotated to tip WITHOUT the last row
  Matrix<double,5,5> Jp;                // Jacobian expressed in tip frame
  Matrix<double,5,1> xep;               // Tip error/delta exp in tip frame
  Matrix<double,5,5> A;                 // Jp^T * Jp + epsilon^2


  // Store the desired tip values.  In this example, construct the
  // normal instead of passing as an argument.
  pd << x,y,z;
  nd = (AngleAxisd(atan2(-x,y), Vector3d::UnitZ()) *
	AngleAxisd(pitch,       Vector3d::UnitX()) *
	AngleAxisd(roll,        Vector3d::UnitY()) *
	Vector3d::UnitZ());

  // Pick some reasonable initial joint values.  This simply assumes
  // the wrist joints are frozen at zero and computes J1-3 from the
  // position.  Gets you pretty close, at least on the "big joints".
  q(3) = 0.0;
  q(4) = 0.0;
  q(0) = atan2(-x, y);
  double cgamma = ((x*x + y*y + z*z - LENA*LENA - (LENB+LENC)*(LENB+LENC)) /
		   (2.0 * LENA * (LENB+LENC)));
  if      (cgamma <= -1.0)   cgamma = -1.0;
  else if (cgamma >=  1.0)   cgamma =  1.0;
  q(2) = -acos(cgamma);
  q(1) = (atan2(z, sqrt(x*x+y*y)) -
	  atan2((LENB+LENC)*sin(q(2)), LENA + (LENB+LENC)*cos(q(2))));

  // Alternatively, just put the arm in the middle of the workspace.
  // Especially J3 should be away from the singularity on the
  // elbow-down side that you like.  But if this is far from the
  // final, you may see the convergence deterioriate!
  q(0) = 0.0;
  q(1) = 0.5  * M_PI;
  q(2) = -0.5 * M_PI;
  q(3) = 0.0;
  q(4) = 0.0;

  // Iterate on the joint values.  Enforcing a max number of
  // iterations helps situations that don't converge.  We could break
  // when the error is sufficiently small (~1e-6?).
  for (i = 0 ; i < ITERATIONS ; i++)
    {
      // Walk the kinematic chain.  Computing orientation (rotation
      // matrix), joint axis, and location of each joint.
      R1 =      AngleAxisd(q(0), Vector3d::UnitZ());
      R2 = R1 * AngleAxisd(q(1), Vector3d::UnitX());
      R3 = R2 * AngleAxisd(q(2), Vector3d::UnitX());
      R4 = R3 * AngleAxisd(q(3), Vector3d::UnitX());
      R5 = R4 * AngleAxisd(q(4), Vector3d::UnitY());
      Rt = R5;

      e1 =      Vector3d::UnitZ();
      e2 = R1 * Vector3d::UnitX();
      e3 = R2 * Vector3d::UnitX();
      e4 = R3 * Vector3d::UnitX();
      e5 = R4 * Vector3d::UnitY();

      p1 = Vector3d::Zero();
      p2 = p1;
      p3 = p2 + R2 * Vector3d::UnitY() * LENA;
      p4 = p3 + R3 * Vector3d::UnitY() * LENB;
      p5 = p4;
      pt = p5 + R5 * Vector3d::UnitY() * LENC;

      nt = Rt * Vector3d::UnitZ();

      // Collect the information in the "regular" 6xN Jacobian.
      J.block<3,1>(0,0) = e1.cross(pt-p1);  J.block<3,1>(3,0) = e1;
      J.block<3,1>(0,1) = e2.cross(pt-p2);  J.block<3,1>(3,1) = e2;
      J.block<3,1>(0,2) = e3.cross(pt-p3);  J.block<3,1>(3,2) = e3;
      J.block<3,1>(0,3) = e4.cross(pt-p4);  J.block<3,1>(3,3) = e4;
      J.block<3,1>(0,4) = e5.cross(pt-p5);  J.block<3,1>(3,4) = e5;

      // Compute the 6x1 tip (pos/orientation) error/delta.
      xe.block<3,1>(0,0) = pd - pt;
      xe.block<3,1>(3,0) = nt.cross(nd);

      // Normally this would now be qe = J^-1 * xe...

      // Rotate the Jacobian and tip error/delta with respect to the tip
      // frame.  Also remove the last row as the normal is the Z axis in
      // the tip frame and rotations about Z are thus meaningless.
      // Practically, we should embed the Rt^T in the above...
      Jp.block<3,5>(0,0)  = Rt.transpose()                 * J.block<3,5>(0,0);
      Jp.block<2,5>(3,0)  = Rt.transpose().block<2,3>(0,0) * J.block<3,5>(3,0);
      xep.block<3,1>(0,0) = Rt.transpose()                 * xe.block<3,1>(0,0);
      xep.block<2,1>(3,0) = Rt.transpose().block<2,3>(0,0) * xe.block<3,1>(3,0);

      // Now we can compute qe = Jp^-1 * xep!
      A = Jp.transpose()*Jp + (EPSILON*EPSILON)*Matrix<double,5,5>::Identity();
      qe = A.colPivHouseholderQr().solve(Jp.transpose()*xep);

      // Scale the joint step to a max of 20deg.
      double scale = MAXJNTSTEP / qe.array().abs().maxCoeff();
      if (scale < 1.0)
	qe = scale * qe;

      // Update the current joint values.
      q += qe;

      // Also enforce the -180deg...0deg range on J3.
      if      (q(2) < -M_PI)   q(2) = -M_PI;
      else if (q(2) >  0.0 )   q(2) =  0.0 ;

      // Report the tip error to watch convergence.  We could break if
      // the error is small (< 1e-6?).
      ROS_INFO("IKin: Iter %d error norm %f", i, xe.norm());
    }

  // Copy the joint solution.
  for (i = 0 ; i < 5 ; i++)
    qsolution[i] = q(i);

  // And check whether the solution converged.
  return (xe.norm() > 1e-6);
}


/*
**   Analytic Inverse Kinematics
*/
bool ikin_analytic(double x, double y, double z, double pitch, double roll,
		   double q[5])
{
  double  rtip;		// Radius of tip
  double  rwrist;	// Radius of wrist
  double  zwrist;	// Height of wrist
  double  cgamma;       // Temporary cosine value
  double  beta;         // Temporary angle

  // Assume the solution will be non-singular, unless set below.
  bool singular = false;

  // Solve for J1 and convert into polar coordinates.
  if ((x == 0) && (y == 0))
    singular = true;
  q[0] = atan2(-x, y);
  rtip = - sin(q[0])*x + cos(q[0])*y;

  // "Trick" = Undo the pitch and compute wrist point.
  rwrist = rtip - cos(pitch) * LENC;
  zwrist = z    - sin(pitch) * LENC;

  // Compute the elbow angle J3, assuming elbow-down (J3 < 0)
  cgamma = ((rwrist*rwrist + zwrist*zwrist - LENA*LENA - LENB*LENB) /
	    (2.0 * LENA * LENB));
  if      (cgamma <= -1.0)   { cgamma = -1.0; singular = true; }
  else if (cgamma >=  1.0)   { cgamma =  1.0; singular = true; }
  q[2] = -acos(cgamma);

  // Compute the angle by which the bent elbow affects the shoulder.
  // This is only singular when the elbow is singular.
  beta = atan2(LENB*sin(q[2]), LENA+LENB*cos(q[2]));

  // Finally compute the shoulder angle J2.
  if ((zwrist == 0) && (rwrist == 0))
    singular = true;
  q[1] = atan2(zwrist, rwrist) - beta;

  // Add the orientation.
  q[3] = pitch - q[1] - q[2];
  q[4] = roll;

  // Return the singular flag.
    q[1] = -q[1];
	q[3] = -q[3];
  return singular;
}


/*
**   Inverse Kinematics Callback
*/
bool ikinCallback(moveto::IKin::Request  &req,
		  moveto::IKin::Response &res)
{
  // Report the requested tip location and orientation.
  ROS_INFO("IKin: Heard [%f, %f, %f, %f, %f]",
	   req.tip.x, req.tip.y, req.tip.z, req.tip.pitch, req.tip.roll);

  // Compute the inverse.
  // res.singular = ikin_iterative(req.tip.x, req.tip.y, req.tip.z,
		// 		req.tip.pitch, req.tip.roll,
		// 		&res.joints.joint[0]);

  // ROS_INFO("IKin: Iterative = %f, %f, %f, %f, %f (%d)",
	 //   res.joints.joint[0], res.joints.joint[1],
	 //   res.joints.joint[2], res.joints.joint[3],
	 //   res.joints.joint[4], res.singular);

  res.singular = ikin_analytic(req.tip.x, req.tip.y, req.tip.z,
			       req.tip.pitch, req.tip.roll,
			       &res.joints.joint[0]);

  ROS_INFO("IKin: Analytic  = %f, %f, %f, %f, %f (%d)",
	   res.joints.joint[0], res.joints.joint[1],
	   res.joints.joint[2], res.joints.joint[3],
	   res.joints.joint[4], res.singular);

  return true;
}


/*
**   Main Node and Initialization Code
*/
int main(int argc, char **argv)
{
  // Initialize ROS and create a node handle.
  ros::init(argc, argv, "ikin");
  ros::NodeHandle nh;

  // Create the service.
  ros::ServiceServer service = nh.advertiseService("/ikin", ikinCallback);
  
  // Run the node until shutdown.
  ROS_INFO("IKin: Standing by...");
  ros::spin();

  return 0;
}
