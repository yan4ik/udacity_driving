#pragma once

#include <vector>

#include "Eigen/Dense"

using std::vector;

using Eigen::Vector4d;


namespace tools {

    Vector4d rmse(const vector<Vector4d>& estimations,
                  const vector<Vector4d>& ground_truth);

};
