#include "tools.h"

namespace tools {

    Vector4d rmse(const vector<Vector4d>& estimations,
                  const vector<Vector4d>& ground_truth) {

        Vector4d rmse;
        rmse << 0,0,0,0;

        
        Vector4d residual;
        for (int i=0; i < estimations.size(); ++i){

            residual = estimations[i] - ground_truth[i];
            residual = residual.array() * residual.array();

            rmse += residual;

        }

        rmse = rmse / estimations.size();

        rmse = rmse.array().sqrt();

        return rmse;

    }

};
