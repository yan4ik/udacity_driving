#pragma once

#include "Eigen/Dense"

#include "Data_Package.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;


class Fusion_EKF {

public:
    
    Fusion_EKF();

    virtual ~Fusion_EKF();

    void process_measurement(const Data_Package& measurement_pack);


    // state vector (x, y, vx, vy)
    Vector4d state_;


private:

    // check whether the tracking toolbox was initiallized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long previous_timestamp_;

    // covariance matrix
    Matrix4d covariance_;

    // state transistion matrix
    Matrix4d state_transition_;

    // process covariance matrix
    Matrix4d process_covariance_;

    // acceleration noise components
    double noise_ax;
    double noise_ay;

    // laser measurement matrix
    MatrixXd laser_measurement_;

    // laser measurement covariance matrix
    Matrix2d laser_covariance_;

    // radar jacobian matrix
    MatrixXd radar_jacobian_;

    // radar covariance matrix;
    Matrix3d radar_covariance_;


    void initialize(const Data_Package& measurement_pack);

    void predict();

    void update_laser(const VectorXd& laser_data);
    
    void update_radar(const VectorXd& radar_data);
    Vector3d predict_measurement();
    void recalc_jacobian();

    
    // truncate every double beoynd this value
    double how_low_can_you_go;

};
