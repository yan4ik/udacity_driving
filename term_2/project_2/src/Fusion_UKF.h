#pragma once

#include "Eigen/Dense"

#include "Data_Package.h"

using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;


class Fusion_UKF {

public:
    
    Fusion_UKF();

    virtual ~Fusion_UKF();

    void process_measurement(const Data_Package& measurement_pack);

    // state vector: [px, py, velocity, yaw, yaw_d]
    VectorXd state_;

    double NIS_laser_;
    double NIS_radar_;

private:

    // used for numerical stability
    const double MAX_DT = 0.1;

    bool is_initialized_;

    long previous_timestamp_;

    MatrixXd covariance_;

    // process noise standard deviation longitudal acceleration
    double noise_std_a_;
    // process noise stantard deviation yaw acceleration
    double noise_std_yaw_dd_;


    // laser measurement matrix
    MatrixXd laser_measurement_;

    // laser measurement covariance matrix
    Matrix2d laser_covariance_;

    // radar covariance matrix;
    Matrix3d radar_covariance_;


    VectorXd weights;

    void initialize(const Data_Package& measurement_pack);

    void predict(double dt,
                 MatrixXd* sigma_points_images);

    void gen_augmented_sigma_points(MatrixXd* sigma_points);

    void ctrv_sigma_points_prediction(const MatrixXd& sigma_points,
                                      double dt,
                                      MatrixXd* sigma_images);

    void update_state_and_covariance(const MatrixXd& sigma_images);


    void update_laser(const VectorXd& laser_data);
    
    void update_radar(const VectorXd& radar_data,
                      const MatrixXd& sigma_images);
    
    // truncate every double beoynd this value
    double how_low_can_you_go;

};
