//
//  prediction.hpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/26/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#ifndef prediction_hpp
#define prediction_hpp

#include <string>
#include <stdio.h>
#include <vector>

#include "helpers.h"

using std::string;
using std::vector;

class Plan {
public:
  // Constructors
  Plan();
  virtual ~Plan();
  
  Plan(double s, double d, double v, string state);

  double s;
  double d;
  double v;
  int lane;
  string state;
  vector<vector<double>> trajectory;
};

#endif /* prediction_hpp */
