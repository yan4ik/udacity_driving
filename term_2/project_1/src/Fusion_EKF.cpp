#include <iostream>

#include "Fusion_EKF.h"

using Eigen::Vector3d;


Fusion_EKF::Fusion_EKF() {
    
    is_initialized_ = false;

    previous_timestamp_ = 0;

    how_low_can_you_go = 0.001;


    state_ = Vector4d();

    covariance_ = Matrix4d();
    covariance_ << 1, 0,    0,    0,
                   0, 1,    0,    0,
                   0, 0, 1000,    0,
                   0, 0,    0, 1000;

    state_transition_ = Matrix4d();
    state_transition_ << 1, 0, 1, 0,  // x_new  = x + vx * dt
                         0, 1, 0, 1,  // y_new  = y + vy * dt
                         0, 0, 1, 0,  // vx_new = vx
                         0, 0, 0, 1;  // vy_new = vy

    process_covariance_ = Matrix4d();
    noise_ax = 9;
    noise_ay = 9;

    laser_measurement_ = MatrixXd(2, 4);
    laser_measurement_ << 1, 0, 0, 0,
                          0, 1, 0, 0;

    laser_covariance_ = Matrix2d();
    laser_covariance_ << 0.0225, 0,
                         0,      0.0225;

    radar_jacobian_ = MatrixXd(3, 4);

    radar_covariance_ = Matrix3d();
    radar_covariance_ << 0.09, 0,      0,
                         0,    0.0009, 0,
                         0,    0,      0.09;
}


Fusion_EKF::~Fusion_EKF() {}


void Fusion_EKF::process_measurement(const Data_Package& measurement_pack) {

    if (!is_initialized_) {
        
        initialize(measurement_pack);

        return;
    }


    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // Modify the state transition matrix 
    // so that the time is integrated
    state_transition_(0, 2) = dt;
    state_transition_(1, 3) = dt;

    // set the process covariance matrix
    process_covariance_ <<  noise_ax * dt_4 / 4, 0, noise_ax * dt_3 / 2, 0,
                            0, noise_ay * dt_4 / 4, 0, noise_ay * dt_3 / 2,
                            noise_ax * dt_3 / 2, 0, noise_ax * dt_2, 0, 
                            0, noise_ay * dt_3 / 2, 0, noise_ay * dt_2;


    predict();

    
    if (measurement_pack.sensor_type_ == Data_Package::RADAR) {

        update_radar(measurement_pack.data_);

    } else if (measurement_pack.sensor_type_ == Data_Package::LASER) {
        
        update_laser(measurement_pack.data_);

    }

   
    return;
}


void Fusion_EKF::initialize(const Data_Package& measurement_pack) {

    double x = 0;
    double y = 0;
    double vx = 0;
    double vy = 0;

    if (measurement_pack.sensor_type_ == Data_Package::RADAR) {
            
        double ro = measurement_pack.data_[0];
        double phi = measurement_pack.data_[1];

        x = cos(phi) * ro; 
        y = sin(phi) * ro;

    } else if (measurement_pack.sensor_type_ == Data_Package::LASER) {
            
        x = measurement_pack.data_[0];
        y = measurement_pack.data_[1];

    }

    state_ << x, y, vx, vy;
    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;

    return;
}


void Fusion_EKF::predict() {

    state_ = state_transition_ * state_;
    covariance_ = state_transition_ * covariance_ * state_transition_.transpose() +
                  process_covariance_;


    return;
}


void Fusion_EKF::update_laser(const VectorXd& laser_data) {

    VectorXd data_pred = laser_measurement_ * state_;
    VectorXd y = laser_data - data_pred;

    MatrixXd laser_measurement_T = laser_measurement_.transpose();
    MatrixXd S = laser_measurement_ * covariance_ * laser_measurement_T + 
                 laser_covariance_;
    MatrixXd K = covariance_ * laser_measurement_T * S.inverse();

    state_ = state_ + (K * y);
    Matrix4d I = Matrix4d::Identity();
    covariance_ = (I - K * laser_measurement_) * covariance_;


    return;
}


void Fusion_EKF::update_radar(const VectorXd& radar_data) {


    Vector3d data_pred = predict_measurement();

    Vector3d y = radar_data - data_pred;

    recalc_jacobian();

    MatrixXd radar_jacobian_T = radar_jacobian_.transpose();
    MatrixXd S = radar_jacobian_ * covariance_ * radar_jacobian_T + 
                 radar_covariance_;
    MatrixXd K = covariance_ * radar_jacobian_T * S.inverse();

    state_ = state_ + (K * y);
    Matrix4d I = Matrix4d::Identity();
    covariance_ = (I - K * radar_jacobian_) * covariance_;


    return;
}


Vector3d Fusion_EKF::predict_measurement() {

    Vector3d data_pred;
    
    double px = state_[0];
    if (fabs(px) < how_low_can_you_go) {
        px = how_low_can_you_go;
    }
    double py = state_[1];
    double vx = state_[2];
    double vy = state_[3];

    
    double ro = sqrt(pow(px, 2) + pow(py, 2));
    
    double phi = atan(py / px);

    double ro_dot = (px * vx + py * vy) / ro;

    
    data_pred << ro, phi, ro_dot;  

    return data_pred;
}


void Fusion_EKF::recalc_jacobian() {

    double px = state_[0];
    if (fabs(px) < how_low_can_you_go) {
        px = how_low_can_you_go;
    }
    double py = state_[1];
    double vx = state_[2];
    double vy = state_[3];


    double c1 = pow(px, 2) + pow(py, 2);
    double c2 = sqrt(c1);
    double c3 = (c1*c2);

   
    radar_jacobian_ << (px/c2), (py/c2), 0, 0,
                       -(py/c1), (px/c1), 0, 0,
                       py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return;
}
