#include <iostream>

#include "Fusion_UKF.h"

using Eigen::Vector3d;


Fusion_UKF::Fusion_UKF() {
    
    is_initialized_ = false;

    previous_timestamp_ = 0;

    how_low_can_you_go = 0.001;


    state_ = VectorXd(5);

    covariance_ = MatrixXd(5, 5);
    covariance_ << 1, 0,   0,  0, 0,
                   0, 1,   0,  0, 0,
                   0, 0, 100,  0, 0,
                   0, 0,   0,  1, 0,
                   0, 0,   0,  0, 1.1;


    noise_std_a_ = 0.5;
    noise_std_yaw_dd_ = 1.;


    laser_measurement_ = MatrixXd(2, 5);
    laser_measurement_ << 1, 0, 0, 0, 0,
                          0, 1, 0, 0, 0;

    laser_covariance_ = Matrix2d();
    laser_covariance_ << 0.0225, 0,
                         0,      0.0225;

    radar_covariance_ = Matrix3d();
    radar_covariance_ << 0.1, 0       , 0,
                         0,   0.000025, 0,
                         0,   0       , 0.3;
    
    
    int n_state = state_.rows();
    int n_state_augmented = n_state + 2;
    double lambda = 3 - n_state_augmented;

    weights = VectorXd(2 * n_state_augmented + 1);

    weights(0) =  lambda / (lambda + n_state_augmented);
    
    for (int i = 1; i < weights.size(); ++i) {

        weights(i) = 0.5 / (lambda + n_state_augmented);

    }


}


Fusion_UKF::~Fusion_UKF() {}


void Fusion_UKF::process_measurement(const Data_Package& measurement_pack) {

    if (!is_initialized_) {
        initialize(measurement_pack);

        return;
    }


    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;


    MatrixXd sigma_points_images = MatrixXd(state_.rows(),
                                            2 * (state_.rows() + 2) + 1);

    // thanks to Wolfgang_Steiner on the forums
    // for this awesome way to solve numerical instability problems
    while (dt > MAX_DT) {
        predict(MAX_DT, &sigma_points_images);
        dt -= MAX_DT;
    }
    predict(dt, &sigma_points_images);
    
    if (measurement_pack.sensor_type_ == Data_Package::RADAR) {

        update_radar(measurement_pack.data_,
                     sigma_points_images);

    } else if (measurement_pack.sensor_type_ == Data_Package::LASER) {
        
        update_laser(measurement_pack.data_);

    }

   
    return;
}


void Fusion_UKF::initialize(const Data_Package& measurement_pack) {

    double px = 0;
    double py = 0;
    double velocity = 0;
    double yaw = 0;
    double yaw_d = 0;

    if (measurement_pack.sensor_type_ == Data_Package::RADAR) {
            
        double ro = measurement_pack.data_[0];
        double phi = measurement_pack.data_[1];

        px = cos(phi) * ro; 
        py = sin(phi) * ro;

    } else if (measurement_pack.sensor_type_ == Data_Package::LASER) {
            
        px = measurement_pack.data_[0];
        py = measurement_pack.data_[1];

    }

    state_ << px, py, velocity, yaw, yaw_d;
    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;

    return;
}


void Fusion_UKF::gen_augmented_sigma_points(MatrixXd* sigma_points) {

    int n_state = state_.rows();
    int n_state_augmented = sigma_points->rows();
    double lambda = 3 - n_state_augmented;

    VectorXd state_augmented = VectorXd(n_state_augmented);
    state_augmented.fill(0);
    state_augmented.head(n_state) = state_;

    MatrixXd covariance_augmented = MatrixXd(n_state_augmented, 
                                             n_state_augmented);
    covariance_augmented.fill(0);
    covariance_augmented.topLeftCorner(n_state, n_state) = covariance_;
    covariance_augmented(n_state, n_state) = pow(noise_std_a_, 2);
    covariance_augmented(n_state + 1, n_state + 1) = pow(noise_std_yaw_dd_, 2);

    MatrixXd L = covariance_augmented.llt().matrixL();
    MatrixXd offset = sqrt(n_state_augmented + lambda) * L;
 
    sigma_points->col(0) = state_augmented;

    for (int i = 1; i <= n_state_augmented; ++i) {

        sigma_points->col(i) = state_augmented + 
                               offset.col(i - 1);
        
        sigma_points->col(n_state_augmented + i) = state_augmented - 
                                                   offset.col(i - 1);
    
    }


    return;
}


void Fusion_UKF::ctrv_sigma_points_prediction(const MatrixXd& sigma_points,
                                              double dt,
                                              MatrixXd* sigma_images) {

    double px = 0;
    double py = 0;
    double v = 0;
    double yaw = 0;
    double yaw_d = 0;
    double nu_a = 0;
    double nu_yaw_dd = 0;

    // predicted state values
    double px_predicted = 0;
    double py_predicted = 0;
    double v_predicted = 0;
    double yaw_predicted = 0;
    double yaw_d_predicted = 0;
    double nu_a_predicted = 0;
    double nu_yaw_dd_predicted = 0;

    for (int i = 0; i < sigma_points.cols(); ++i) {
    
    	px = sigma_points(0, i);
    	py = sigma_points(1, i);
    	v = sigma_points(2, i);
    	yaw = sigma_points(3, i);
    	yaw_d = sigma_points(4, i);
    	nu_a = sigma_points(5, i);
    	nu_yaw_dd = sigma_points(6, i);
    
    	if (fabs(yaw_d) > how_low_can_you_go) {
        	px_predicted = px + v/yaw_d * (sin(yaw + yaw_d * dt) - sin(yaw));
        	py_predicted = py + v/yaw_d * (cos(yaw) - cos(yaw + yaw_d * dt));
    	}
    	else {
        	px_predicted = px + v * dt * cos(yaw);
        	py_predicted = py + v * dt * sin(yaw);
    	}

 	   	v_predicted = v;
 	   	yaw_predicted = yaw + yaw_d * dt;
       	yaw_d_predicted = yaw_d;

    	// add noise
    	px_predicted += 0.5 * nu_a * pow(dt, 2) * cos(yaw);
    	py_predicted += 0.5 * nu_a * pow(dt, 2) * sin(yaw);
    	v_predicted += nu_a * dt;

    	yaw_predicted += 0.5 * nu_yaw_dd * pow(dt, 2);
    	yaw_d_predicted += nu_yaw_dd * dt;

    	(*sigma_images)(0, i) = px_predicted;
    	(*sigma_images)(1, i) = py_predicted;
    	(*sigma_images)(2, i) = v_predicted;
    	(*sigma_images)(3, i) = yaw_predicted;
    	(*sigma_images)(4, i) = yaw_d_predicted;
    }
 
    return;
}

void Fusion_UKF::update_state_and_covariance(const MatrixXd& sigma_images) {
    
    //predicted state mean
    state_.fill(0.0);
    
    for (int i = 0; i < sigma_images.cols(); ++i) {
        
        state_ += weights(i) * sigma_images.col(i);
    
    }

    //predicted state covariance matrix
    covariance_.fill(0.0);

    for (int i = 0; i < sigma_images.cols(); ++i) {

        // state difference
        VectorXd x_diff = sigma_images.col(i) - state_;
    
	    // angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        covariance_ += weights(i) * x_diff * x_diff.transpose();

    }

}


void Fusion_UKF::predict(double dt,
                         MatrixXd* sigma_points_images) {

    // generate sigma points

    MatrixXd augmented_sigma_points = MatrixXd(state_.rows() + 2,
                                               2 * (state_.rows() + 2) + 1);

    gen_augmented_sigma_points(&augmented_sigma_points);

    // predict sigma points images

    ctrv_sigma_points_prediction(augmented_sigma_points,
                                 dt,
                                 sigma_points_images);

    update_state_and_covariance(*sigma_points_images);

    return;
}


void Fusion_UKF::update_laser(const VectorXd& laser_data) {

    VectorXd data_pred = laser_measurement_ * state_;
    VectorXd y = laser_data - data_pred;

    MatrixXd laser_measurement_T = laser_measurement_.transpose();
    MatrixXd S = laser_measurement_ * covariance_ * laser_measurement_T + 
                 laser_covariance_;
    MatrixXd K = covariance_ * laser_measurement_T * S.inverse();

    state_ = state_ + (K * y);
    MatrixXd I = MatrixXd::Identity(5, 5);
    covariance_ = (I - K * laser_measurement_) * covariance_;

    NIS_laser_ = y.transpose() * S.inverse() * y;

    return;
}


void Fusion_UKF::update_radar(const VectorXd& radar_data,
                              const MatrixXd& sigma_images) {


    MatrixXd sigma_images_images = MatrixXd(radar_data.size(), sigma_images.cols());

    // transform sigma points into measurement space

    double px = 0;
    double py = 0;
    double v = 0;
    double yaw = 0;

    double vx = 0;
    double vy = 0;

    for (int i = 0; i < sigma_images_images.cols(); ++i) {

    	px = sigma_images(0, i);
    	if (fabs(px) < how_low_can_you_go) {
			px = how_low_can_you_go;
        }
        py = sigma_images(1, i);
    	v  = sigma_images(2, i);
    	yaw = sigma_images(3, i);

    	vx = cos(yaw) * v;
    	vy = sin(yaw) * v;

    	// measurement model

    	sigma_images_images(0, i) = sqrt(pow(px, 2) + pow(py, 2));
    	sigma_images_images(1, i) = atan2(py, px);
    	sigma_images_images(2, i) = (px * vx + py * vy ) / sqrt(pow(px, 2) + pow(py, 2));
    }    

    // predicted measurement mean
	
    VectorXd z_predicted = VectorXd(radar_data.size());
    z_predicted.fill(0.0);
    
    for (int i = 0; i < sigma_images_images.cols(); ++i) {
        
        z_predicted += weights(i) * sigma_images_images.col(i);
    
    }

    // predicted measurement covariance matrix

    MatrixXd S = MatrixXd(radar_data.size(), radar_data.size());
    S.fill(0.0);

    for (int i = 0; i < sigma_images_images.cols(); ++i) {

        // measurement difference
        VectorXd z_diff = sigma_images_images.col(i) - z_predicted;
    
	    // angle normalization
        while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S += weights(i) * z_diff * z_diff.transpose();

    }

    S += radar_covariance_;
    
    // measurement update

    MatrixXd Tc = MatrixXd(state_.rows(), radar_data.rows());
    Tc.fill(0.0);
	
    for (int i = 0; i < sigma_images_images.cols(); ++i) {

	    VectorXd z_diff = sigma_images_images.col(i) - z_predicted;
	    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
	    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

	    VectorXd x_diff = sigma_images.col(i) - state_;
	    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
	    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

	    Tc += weights(i) * x_diff * z_diff.transpose();

    }

    MatrixXd K = Tc * S.inverse();

    VectorXd z_diff = radar_data - z_predicted;
    while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    state_ += K * z_diff;
    covariance_ -= K * S * K.transpose();

    // NIS
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

    return;
}
