
/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: FeatureTracker.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "FeatureTracker.h"
#include "feature.h"
#include "XYPoint.h"

using namespace std;

//---------------------------------------------------------
// Constructor

FeatureTracker::FeatureTracker()
{

}

//---------------------------------------------------------
// Destructor

FeatureTracker::~FeatureTracker()
{
  delete feature_list;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool FeatureTracker::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key=="COLOR_FEATURE"){
      feature_list->update(robotx::ColorFeature(sval));
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);

   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool FeatureTracker::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool FeatureTracker::Iterate()
{
  AppCastingMOOSApp::Iterate();

  std::cout << "Publishing tracked feature list" << std::endl;

  // publish colored feature list for autonomy
  for(int i =0; i<feature_list->size(); i++){
    TrackedFeature tf = feature_list->get(i);
    Notify("TRACKED_FEATURE",tf.Serialize());
    //publish some eye candy for pMarineViewer:
    XYPoint p(tf.x,tf.y);
    p.set_label("feature_"+intToString(tf.id));
    p.set_vertex_size(tf.size);
    
    p.set_vertex_color(color2String(tf.color));
    Notify("VIEW_POINT",p.get_spec());
  }

  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool FeatureTracker::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "MIN_FEATURE_DIST") {
        feature_list = new ColorFeatureList(atof(value.c_str()));
	reportEvent("Initializing feature dist with min dist = " + value);
      }
      else
	reportRunWarning("Unhandled param" + param);

    }
  }
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void FeatureTracker::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("COLOR_FEATURE","*", 0);
}

std::string FeatureTracker::color2String(robotx::Color c){
  std::string s;
  if (c==RED) s="red";
  else if (c==WHITE) s="white";
  else if (c==GREEN) s="green";
  else s="yellow";
  return s;
}


bool FeatureTracker::buildReport(){

  m_msgs << "Report \n" ;
  m_msgs << "Number of features: " << feature_list->size();
  
  return(true);

}
