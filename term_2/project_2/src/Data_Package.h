#pragma once


#include "Eigen/Dense"


class Data_Package {

public:
    
    long timestamp_;

    enum Sensor_Type {   
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd data_;

};
