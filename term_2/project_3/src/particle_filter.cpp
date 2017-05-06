/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using std::vector;
using std::default_random_engine;
using std::normal_distribution;
using std::discrete_distribution;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    default_random_engine gen;
    normal_distribution<double> N_x_init(x, std[0]);
    normal_distribution<double> N_y_init(y, std[1]);
    normal_distribution<double> N_theta_init(theta, std[2]);
    
    num_particles = 100;

    particles = vector<Particle>(num_particles);
    weights = vector<double>(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        
        particles[i].id = i;
        particles[i].x = N_x_init(gen);
        particles[i].y = N_y_init(gen);
        particles[i].theta = N_theta_init(gen);
        particles[i].weight = 1.;

        weights[i] = particles[i].weight;

    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    default_random_engine gen;
    for (int i = 0; i < num_particles; ++i) {

        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

        double new_x = x + (velocity / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
        double new_y = y + (velocity / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
        double new_theta = theta + yaw_rate * delta_t;

        normal_distribution<double> N_x_init(new_x, std_pos[0]);
        normal_distribution<double> N_y_init(new_y, std_pos[1]);
        normal_distribution<double> N_theta_init(new_theta, std_pos[2]);
        
        particles[i].x = N_x_init(gen);
        particles[i].y = N_y_init(gen);
        particles[i].theta = N_theta_init(gen);

    }

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];

    double total_weight = 0.;

    for (int particle_id = 0; particle_id < num_particles; ++particle_id) {

        Particle particle = particles[particle_id];
        particle.weight = 1.;

        for (int i = 0; i < observations.size(); ++i) {

            LandmarkObs observation = observations[i];

            // transform coordinates
            double observations_x = observation.x;
            double observations_y = observation.y;

            observation.x = observations_x * cos(particle.theta) - 
                            observations_y * sin(particle.theta) +
                            particle.x;

            observation.y = observations_x * sin(particle.theta) + 
                            observations_y * cos(particle.theta) +
                            particle.y;

            // get closest landmark
            int landmark_id = -1;
            double current_best_distance = 0.;

            for (int l_id = 0; l_id < map_landmarks.landmark_list.size(); ++l_id) {

                Map::single_landmark_s landmark = map_landmarks.landmark_list[l_id];

                double distance = dist(observation.x, observation.y,
                                       landmark.x_f, landmark.y_f);

                if ((landmark_id == -1) || (current_best_distance > distance)) {

                    landmark_id = l_id;
                    current_best_distance = distance;

                }

            }

            Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_id];

            particle.weight *= exp(-(
                                     pow(observation.x - landmark.x_f, 2) / 
                                        (2 * pow(sigma_x, 2)) + 
                                     pow(observation.y - landmark.y_f, 2) /
                                        (2 * pow(sigma_y, 2))
                                    )
                                  );
            particle.weight /= 2 * M_PI * sigma_x * sigma_y;
        }

        particles[particle_id].weight = particle.weight;
        total_weight += particle.weight;

    }

    // renormalize weights
    for (int particle_id = 0; particle_id < num_particles; ++particle_id) {
        
        particles[particle_id].weight /= total_weight;
        weights[particle_id] = particles[particle_id].weight;
    
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        resampled[i] = particles[distribution(gen)];
    }

    particles = resampled;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
