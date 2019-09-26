//
//  vehicle.hpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/25/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#ifndef vehicle_hpp
#define vehicle_hpp

#include <stdio.h>

#include <map>
#include <string>
#include <vector>

//#include "helpers.h"

using std::map;
using std::string;
using std::vector;

class Vehicle {
public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS");
  
  // Destructor
  virtual ~Vehicle();

  vector<string> next_states();
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);

  map<int, vector<double>> generate_predictions(map<int, vector<double>> sensor_fusion, int num_steps);

  double x;
  double y;
  float s;
  float a;
  float yaw;
  float speed;
  int lane;
  string state;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
};

#endif /* vehicle_hpp */
