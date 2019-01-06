#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include "types.h"
#include "MBUtils.h"
#include <stdlib.h>

namespace robotx{

  class Feature: public Point{
  public:
    int size; // number of points in the point cloud?

  public:
    Feature(){};
  Feature(double size_, Point p_):size(size_),Point(p_){};
  Feature(double size_, double x_, double y_):Point(x_,y_),size(size_){};
  Feature(std::string in):Point(in){ // Format for incoming string: "x=12.2,y=13.5,size=12.3"
      std::vector<std::string> str_vec = parseString(in,',');
      for(unsigned int i=0; i<str_vec.size(); i++){
	std::string param = biteStringX(str_vec[i],'=');
	std::string value = str_vec[i];
	if(param == "size")
	  size = atoi(value.c_str());
      }
  };
  ~Feature(){};

 public:

  std::string Serialize(){
    return Point::Serialize() + ",size=" + intToString(size);
  };
};

  class PlacardFeature: public Point{
  public:
    std::string shape; // the type of shape on the placard

  public:
    PlacardFeature(){};
  PlacardFeature(std::string shape_, Point p_):shape(shape_),Point(p_){};
  PlacardFeature(std::string shape_, double x_, double y_):Point(x_,y_),shape(shape_){};
  PlacardFeature(std::string in):Point(in){ // Format for incoming string: "x=12.2,y=13.5,size=12.3"
      std::vector<std::string> str_vec = parseString(in,',');
      for(unsigned int i=0; i<str_vec.size(); i++){
	std::string param = biteStringX(str_vec[i],'=');
	std::string value = str_vec[i];
	if(param == "shape")
	  shape = value;
      }
  };
  ~PlacardFeature(){};

 public:

  std::string Serialize(){
    return Point::Serialize() + ",shape=" + shape;
  };
};


 enum Color {RED, WHITE, GREEN, UNKNOWN}; // special note: need to leave UNKNOWN as the last one because I use it to determine number of different color options in the enum

 class ColorFeature : public Feature{
 public:
  Color color;

 public:
  ColorFeature(){};
 ColorFeature(Feature feature_, Color color_):Feature(feature_),color(color_){};
 ColorFeature(std::string in):Feature(in){ // Format for incoming string: "x=12.2,y=13.5,size=12.3,color=1" // 1 = WHITE e.g. see the enum declaration for Color
    std::vector<std::string> str_vec = parseString(in,',');
    for(unsigned int i=0; i<str_vec.size(); i++){
      std::string param = biteStringX(str_vec[i],'=');
      std::string value = str_vec[i];
      if(param == "color")
	color = static_cast<Color>(atoi(value.c_str()));
    }

  };


 public:
  std::string Serialize(){
    return Feature::Serialize()+",color="+intToString(color);
  }
};

 static int feature_count = 0;
 // A TrackedFeature is one that is maintained within the pFeatureTracker application. 
 // Currently maintining an array of bins that correspond to the number of times it has been detected as each color along with methods for making a decision about what color it is
 class TrackedFeature : public ColorFeature{
 public:
   std::vector<int> color_bins; 
   std::vector<Point> point_history;
   std::vector<int> size_history;
   int id;

 TrackedFeature(std::string in):ColorFeature(in){
    std::vector<std::string> str_vec = parseString(in,',');
    for(unsigned int i=0; i<str_vec.size(); i++){
      std::string param = biteStringX(str_vec[i],'=');
      std::string value = str_vec[i];
      if(param == "label")
	id = atoi(value.c_str());
    }


   };
 TrackedFeature(ColorFeature feature_):ColorFeature(feature_){
     for (int i=0; i<UNKNOWN; i++){// Special note: I'm using the UNKNOWN type to determine the size of the array for color_bins. Can't think of a better way right now.  
       color_bins.push_back(0);
     }
     point_history.push_back(feature_);
     size_history.push_back(feature_.size);
     id = feature_count;
     feature_count++;
   }

 // update the "color" estimate based on the status of the color bins
   Color update_color(){
     Color c = UNKNOWN;
     int max_bin=0;
     for (int i=0; i<UNKNOWN; i++){
       if (color_bins[i] > max_bin){
	 c = static_cast<Color>(i);
	 max_bin = color_bins[i];
       }
     }  
     return c;
   }
   void update_info(ColorFeature fc){
     x = fc.x;
     y = fc.y;
     size = fc.size;
     point_history.push_back(fc);
     size_history.push_back(fc.size);
     color_bins[fc.color]++;
     update_color();
   }
  std::string Serialize(){
    return ColorFeature::Serialize()+",label="+intToString(id);
  }
   

 };
}

