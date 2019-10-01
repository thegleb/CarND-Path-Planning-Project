//
//  prediction.cpp
//  CarND-Path-Planning-Project
//
//  Created by Gleb on 9/26/19.
//  Copyright © 2019 Gleb. All rights reserved.
//

#include "prediction.hpp"

Prediction::Prediction() {}
Prediction::~Prediction() {}

Prediction::Prediction(double s, double d, double v, string state) {
  this->state = state;
  this->s = s;
  this->d = d;
  this->v = v;
  this->lane = get_lane_from_d(d);
}
