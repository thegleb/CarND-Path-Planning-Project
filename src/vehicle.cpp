//
//  vehicle.cpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/25/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#include "vehicle.hpp"

Vehicle::Vehicle() {
  state = "KL";
  target_v = TARGET_V;
}
Vehicle::~Vehicle() {}


vector<string> Vehicle::next_states() {
  vector<string> states;
  string state = this->state;
  
  // "lane change left/right" -> "lane change in progress"
  if (state.compare("LCR") == 0 || state.compare("LCL") == 0) {
    states.push_back("LCIP");
    return states;
  }
  
  // "lane change in progress" and lane change is not completed, keep that state
  if (state.compare("LCIP") == 0) {
    if (abs(this->d - (target_lane * 4 + 2)) > 0.1) {
      states.push_back("LCIP");
      return states;
    }
  }

  states.push_back("KL");
  if(state.compare("KL") == 0) {
//    states.push_back("PLCL");
//    states.push_back("PLCR");
//  } else if (state.compare("PLCL") == 0) {
    if (lane != 2) {
//      states.push_back("PLCL");
      states.push_back("LCR");
    }
//  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
//      states.push_back("PLCR");
      states.push_back("LCL");
    }
//  }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}
