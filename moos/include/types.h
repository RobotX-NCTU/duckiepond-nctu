#ifndef TYPESH
#define TYPESH

#include <cstdio>
#include <string>
#include "MBUtils.h"
#include <stdlib.h>
#include <vector>
namespace robotx{

class Point{
 public:
  double x;
  double y;

 public: 
 Point(double x_, double y_):x(x_),y(y_) {};
  Point(std::string in){
    std::vector<std::string> str_vec = parseString(in,',');
    for(unsigned int i=0; i<str_vec.size(); i++){
      std::string param = biteStringX(str_vec[i],'=');
      std::string value = str_vec[i];
      if(param == "x")
	x=atof(value.c_str());
      if(param == "y")
	y=atof(value.c_str());
    }
  };
  Point(){};
  ~Point(){};

  std::string Serialize(){
    return "x="+doubleToString(x)+",y="+doubleToString(y);
  }


};


class Point3{
 public:
  double x;
  double y;
  double z;

 public: 
 Point3(double x_, double y_, double z_):x(x_),y(y_),z(z_) {};
  Point3(std::string in){
    std::vector<std::string> str_vec = parseString(in,',');
    for(unsigned int i=0; i<str_vec.size(); i++){
      std::string param = biteStringX(str_vec[i],'=');
      std::string value = str_vec[i];
      if(param == "x")
	x=atof(value.c_str());
      if(param == "y")
	y=atof(value.c_str());
      if(param == "z")
	z=atof(value.c_str());
    }
  };
  Point3(){};
  ~Point3(){};

  std::string Serialize(){
    return "x="+doubleToString(x)+",y="+doubleToString(y)+",z="+doubleToString(z);
  }


};


 class Pose: public Point{
 public:
  float psi;
    
 public:
  Pose(){};
 Pose(double x_, double y_, double psi_):Point(x_,y_),psi(psi_){};
 Pose(Point p_, double psi_):Point(p_),psi(psi_){};
  ~Pose(){};
  
  public:

  std::string Serialize(){
    return Point::Serialize()+",psi="+doubleToString(psi);
  };

};



 class Pose3: public Point3{
 public:
   double phi; // roll
   double theta; // pitch
   double psi; // yaw
    
 public:
  Pose3(){};
 Pose3(double x_, double y_, double z_, double phi_, double theta_, double psi_):
  Point3(x_,y_,z_),phi(phi_),theta(theta_),psi(psi_){};
 Pose3(Point3 p_, double phi_, double theta_, double psi_):Point3(p_),phi(phi_),theta(theta_),psi(psi_){};
  ~Pose3(){};
  
  public:

  std::string Serialize(){
    return Point3::Serialize()+",phi="+doubleToString(phi)+",theta="+doubleToString(theta)+",psi="+doubleToString(psi);
  };

};

class Pixel{
 public:
  int u;
  int v;
 public:
  Pixel(){};
 Pixel(int u_, int v_):u(u_),v(v_){};
  ~Pixel(){};
};

} // namespace

#endif
