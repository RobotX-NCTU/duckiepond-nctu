#--------------------------------------------------------
# The CMakeLists.txt for:                       pFeatureTracker
# Author(s):                              
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  ColorFeatureList.cpp
  FeatureTracker.cpp
  FeatureTracker_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pFeatureTracker ${SRC})

TARGET_LINK_LIBRARIES(pFeatureTracker
   ${MOOS_LIBRARIES}
   geometry
   mbutil
   m
   pthread)

