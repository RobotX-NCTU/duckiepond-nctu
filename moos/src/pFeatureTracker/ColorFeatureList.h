#include "feature.h"

using namespace robotx;

class ColorFeatureList{
  std::vector<TrackedFeature> feature_list;
 public:
 ColorFeatureList(double _min_feature_dist):min_feature_dist(_min_feature_dist){};
  ~ColorFeatureList(){};

  bool update(ColorFeature);
  int size(){return feature_list.size();};
  TrackedFeature get(int i){return feature_list[i];};

 private:
  double min_feature_dist; // the minimum distance beyond which we will consider two features to be distint
};
