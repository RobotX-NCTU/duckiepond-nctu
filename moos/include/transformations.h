#ifndef TRANSFORMATIONSH
#define TRANSFORMATIONSH

#include "types.h"
#include <Eigen/Dense>

using namespace Eigen;

#ifndef PI
#define PI 3.1415
#endif

typedef Matrix<double, 4, 4> Matrix4d;

namespace robotx{
  using namespace std;
// This class will hold all of the various transformations between frames.
// Right now they are mostly place holders since we don't have the exact configuration of the sensors onboard.



class Transformations{
 
Matrix4d B2V; // Body to velodyne
Matrix4d B2C; // Body to camera
Matrix3d K;


 public: 
  Transformations(){
    // homongeneous transformation for velodyne
    double v_phi = 0.0; // relative roll 
    double v_theta = 0.0; // relative pitch
    double v_psi = PI; // relative yaw
    double v_x = 0.0; // relative x
    double v_y = 0.0; // relative y
    double v_z = 0.0; // relative z
    B2V = homogeneous_matrix(v_phi, v_theta, v_psi, v_x, v_y, v_z);

    // homogeneous transformation for camera
    double c_phi = 0.0; // relative roll 
    double c_theta = 0.0; // relative pitch
    double c_psi = 0.0; // relative yaw
    double c_x = 0.0; // relative x
    double c_y = 0.0; // relative y
    double c_z = 0.0; // relative z
    B2C = homogeneous_matrix(c_phi, c_theta, c_psi, c_x, c_y, c_z);

    // intrinsic params for camera
    double alpha_x = 548.33032; // x value of focal point in pixels
    double alpha_y = 549.64008; // y value of focal point in pixels
    double s = 0; // skew
    double x_0 = 318.86506; // num pixels in x / 2
    double y_0 = 202.29561; // num pixels in y / 2


    K << alpha_x , s , x_0 
      , 0 , alpha_y  , y_0
      , 0 , 0 , 1;
    

  };

  void setB2V(double trans[6]){
    B2V=homogeneous_matrix(trans[0],trans[1],trans[2],trans[3],trans[4],trans[5]);
  }
  void setB2C(double trans[6]){
    B2C=homogeneous_matrix(trans[0],trans[1],trans[2],trans[3],trans[4],trans[5]);
  }

 private:
  Matrix4d homogeneous_rotation_matrix(double phi, double theta, double psi){

    Matrix4d M;
    double c_phi   = cos(phi);
    double s_phi   = sin(phi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_psi   = cos(psi);
    double s_psi   = sin(psi);

    M << c_psi*c_theta ,     -s_psi*c_phi + c_psi*s_theta*s_phi ,           s_psi*s_phi + c_phi*s_phi*s_theta ,           0
      , s_psi*c_theta ,       c_psi*c_phi + s_phi*s_theta*s_psi ,           -c_psi*s_phi + s_theta*s_psi*c_phi,           0 
      , -s_theta   ,           c_theta*s_phi ,                             c_theta*c_phi                               , 0
      , 0 , 0 , 0 , 1.0 ;
    return M;
  };
 

  Matrix4d homogeneous_matrix(double phi, double theta, double psi,
			      double x, double y, double z){
    Matrix4d M = homogeneous_rotation_matrix(phi,theta,psi);
    M(0,3) = x;
    M(1,3) = y;
    M(2,3) = z;
    return M;
  };
  
  

    
 public:

Point3 velodyne2body(Point3 p_velo){
  Vector4d p_velo_homo(p_velo.x,p_velo.y,0,1);
  Vector4d p_body_homo = B2V.inverse()*p_velo_homo;
  Point3 p_body(p_body_homo(0),p_body_homo(1),p_body_homo(2));
  return p_body;
  
};

Point3 body2velodyne(Point3 p_body){
  Vector4d p_body_homo(p_body.x,p_body.y,0,1);
  Vector4d p_body_velo = B2V*p_body_homo;
  Point3 p_velo(p_body_velo(0),p_body_velo(1),p_body_velo(2));
  return p_velo;
};

Point camera2body(Point p_cam){
  Vector4d p_cam_homo(p_cam.x,p_cam.y,0,1);
  Vector4d p_body_homo=B2C.inverse()*p_cam_homo;
  Point p_body(p_body_homo(0),p_body_homo(1));
  return p_body;

};

 Point3 body2camera(Point3 p_body){
   Vector4d p_body_homo(p_body.x,p_body.y,p_body.z,1);
   Vector4d p_cam_homo=B2C*p_body_homo;
   Point3 p_cam(p_cam_homo(0),p_cam_homo(1),p_cam_homo(2));
   return p_cam;
};

 Point3 body2global(Point3 p_body, Pose3 nav){ // assume that nav is in MOOS coordinates (y up x to the right and psi is rotation from y towards x)
   Matrix4d R = homogeneous_rotation_matrix(nav.phi,nav.theta,nav.psi);
   Vector4d p_body_homo(p_body.x, p_body.y,0,1);
   Vector4d p = R*p_body_homo;
   Point3 p_glob(p(1)+nav.x,p(0)+nav.y,p(2)+nav.z);
   return p_glob;
};

Point3 global2body(Point3 p_glob, Pose3 nav){
  Matrix4d R = homogeneous_rotation_matrix(-nav.phi,-nav.theta,-nav.psi);
  Vector4d p(p_glob.y-nav.y,p_glob.x-nav.x,p_glob.z-nav.z,1);
  Vector4d p_body_homo=R*p;
  Point3 p_body(p_body_homo(0),p_body_homo(1),p_body_homo(2));
  return p_body;
};



Pixel camera2pixel(Point3 p_cam){

  Matrix<double, 3, 4> I_O;
  I_O << 1 , 0 , 0 , 0 
    , 0 , 1 , 0 , 0
    , 0 , 0 , 1 , 0;

  Vector4d p_cam_homo(p_cam.y,p_cam.z,p_cam.x,1);
  Vector3d pix_homo = K*I_O*p_cam_homo; // pinhole camera projection model.
  Pixel pix(floor(pix_homo(0)/pix_homo(2)),floor(pix_homo(1)/pix_homo(2))); // remove normalizer
  return pix;
};

Pixel global2pixel(Point3 p_glob, Pose3 nav){
  return camera2pixel(body2camera(global2body(p_glob,nav)));
};

};
}
#endif
