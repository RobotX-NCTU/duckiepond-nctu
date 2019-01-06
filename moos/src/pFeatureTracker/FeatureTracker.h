/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: FeatureTracker.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef FeatureTracker_HEADER
#define FeatureTracker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "ColorFeatureList.h"

class FeatureTracker : public AppCastingMOOSApp
{
 public:
   FeatureTracker();
   ~FeatureTracker();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool buildReport();

 private: // Configuration variables

 private: // State variables
   std::string color2String(robotx::Color);
   ColorFeatureList* feature_list;

};

#endif 
