#include "utils.h" // for norm
#include "ColorFeatureList.h"
#include <iostream>


// try to add a color_feature to the list. If it is added return true. Otherwise update the corresponding tracked feature and return false
bool ColorFeatureList::update(ColorFeature color_feature){
  for(int i=0; i<feature_list.size(); i++){
    //    std::cout << "Feature list size: " << feature_list.size() << std::endl;
    //std::cout << "norm: " << robotx::norm(color_feature, feature_list[i]) << std::endl;
    if(robotx::norm(color_feature, feature_list[i]) < min_feature_dist){ // we found a match
      feature_list[i].update_info(color_feature);
      std::cout << "Updated feature " << i << std::endl;
      return false;
    }
  }
  // no feature close enough make a new one
  std::cout << "Building new tracked feature" << std::endl;
  TrackedFeature tracked_feature(color_feature);
  tracked_feature.color_bins[color_feature.color]++;
  feature_list.push_back(tracked_feature);
  return true;
  
}

