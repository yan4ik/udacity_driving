#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "Data_Package.h"
#include "Fusion_EKF.h"
#include "tools.h"

using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::string;
using std::getline;
using std::vector;

using Eigen::VectorXd;


void check_arguments(int argc, char* argv[]) {
   
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    // user provided wrong number of arguments
    if (argc != 3) {
        
        if (argc == 1) {

            cerr << usage_instructions << endl;
        
        } else if (argc == 2) {

            cerr << "Please include an output file." << endl
                 << usage_instructions << endl;
        
        } else if (argc > 3) {

            cerr << "Too many arguments." << endl
                 << usage_instructions << endl;

        }

        exit(EXIT_FAILURE);
    }

}


void check_files(const ifstream& in_file, const string& in_file_name,
                 const ofstream& out_file, const string& out_file_name) {

    if (!in_file.is_open()) {

        cerr << "Cannot open input file: " << in_file_name << endl;
        exit(EXIT_FAILURE);

    }

    if (!out_file.is_open()) {
      
        cerr << "Cannot open output file: " << out_file_name << endl;
        exit(EXIT_FAILURE);
  
    }

}


void input_data(ifstream* in_file,
                vector<Data_Package>* measurement_pack_list,
                vector<Data_Package>* gt_pack_list) {


    // prepare variables
    string sensor_type;
    long timestamp = 0;

    // laser variables
    double x = 0.;
    double y = 0.;

    // radar variables
    double ro = 0.;
    double phi = 0.;
    double ro_dot = 0.;

    // ground truth variables
    double x_gt = 0.;
    double y_gt = 0.;
    double vx_gt = 0.;
    double vy_gt = 0.;


    string line;
    while (getline(*in_file, line)) {
        
        istringstream iss(line);

        
        // read measurement package
        Data_Package measurement_package;

        iss >> sensor_type;

        // LASER MEASUREMENT
        if (sensor_type == "L") {

            measurement_package.sensor_type_ = Data_Package::LASER;
            
            measurement_package.data_ = VectorXd(2);
            iss >> x >> y;
            measurement_package.data_ << x, y;

            iss >> timestamp;
            measurement_package.timestamp_ = timestamp;

            measurement_pack_list->push_back(measurement_package);

        // RADAR MEASUREMENT
        } else if (sensor_type == "R") {
            
            measurement_package.sensor_type_ = Data_Package::RADAR;

            measurement_package.data_ = VectorXd(3);
            iss >> ro >> phi >> ro_dot;
            measurement_package.data_ << ro, phi, ro_dot;

            iss >> timestamp;
            measurement_package.timestamp_ = timestamp;

            measurement_pack_list->push_back(measurement_package);

        }

        
        // read ground truth data       
        Data_Package groud_truth_package;

        groud_truth_package.data_ = VectorXd(4);
        iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
        groud_truth_package.data_ << x_gt, y_gt, vx_gt, vy_gt;

        gt_pack_list->push_back(groud_truth_package);

    }

}


void output_step(const Fusion_EKF& fusion_ekf,
                 const Data_Package& measurement,
                 const Data_Package& ground_truth,
                 ofstream* out_file) {

    *out_file << fusion_ekf.state_(0) << "\t";
    *out_file << fusion_ekf.state_(1) << "\t";
    *out_file << fusion_ekf.state_(2) << "\t";
    *out_file << fusion_ekf.state_(3) << "\t";

    if (measurement.sensor_type_ == Data_Package::LASER) {
    
        *out_file << measurement.data_(0) << "\t";
        *out_file << measurement.data_(1) << "\t";

    } else if (measurement.sensor_type_ == Data_Package::RADAR) {

        double ro = measurement.data_(0);
        double phi = measurement.data_(1);

        *out_file << ro * cos(phi) << "\t";
        *out_file << ro * sin(phi) << "\t";

    }

    *out_file << ground_truth.data_(0) << "\t";
    *out_file << ground_truth.data_(1) << "\t";
    *out_file << ground_truth.data_(2) << "\t";
    *out_file << ground_truth.data_(3);

    *out_file << endl;

    return;
}


int main(int argc, char* argv[]) {
    
    check_arguments(argc, argv);

    // Setting up io files

    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);
    
    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, in_file_name_,
                out_file_, out_file_name_);

    // Reading data

    vector<Data_Package> measurement_pack_list;
    vector<Data_Package> gt_pack_list;

    input_data(&in_file_, &measurement_pack_list, &gt_pack_list);

    // Filtering

    Fusion_EKF fusion_ekf;

    vector<Vector4d> estimations;
	vector<Vector4d> ground_truth;

    size_t N = measurement_pack_list.size();
    for (size_t i = 0; i < N; ++i ) {

        fusion_ekf.process_measurement(measurement_pack_list[i]);

        output_step(fusion_ekf, 
                    measurement_pack_list[i], 
                    gt_pack_list[i],
                    &out_file_);

		estimations.push_back(fusion_ekf.state_);
		ground_truth.push_back(gt_pack_list[i].data_);

    }

    // Final steps

    cout << "Accuracy - RMSE:" << endl 
         << tools::rmse(estimations, ground_truth) << endl;

  	if (out_file_.is_open()) {
    	out_file_.close();
  	}

  	if (in_file_.is_open()) {
    	in_file_.close();
  	}  

    return 0;
}
