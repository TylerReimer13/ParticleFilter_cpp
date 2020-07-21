#include <iostream>
#include <string>
#include <random>
#include <math.h>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"
#include "particle_filter.h"

#define PI 3.141592654

namespace plt = matplotlibcpp;

ParticleFilter::ParticleFilter(float pos[2], int num_particles, std::vector<Landmark> lndmrks, float sensor_noise, float x_bounds[2], float y_bounds[2]) {
    x_pos = pos[0];
    y_pos = pos[1];
    n_particles = num_particles;
    detected_landmarks = lndmrks;
    sensor_error = sensor_noise;
    X_MIN = x_bounds[0];
    X_MAX = x_bounds[1];
    Y_MIN = y_bounds[0];
    Y_MAX = y_bounds[1];
};

float ParticleFilter::normal_dist(float x, float mean, float var) {
    float likelihood = 1/(var*pow((2.*PI), .5)) * exp(-(pow((x-mean), 2) / (2.*pow(var, 2))));
    return likelihood;
};

void ParticleFilter::find_mean() {
    float sum_x=0.;
    float sum_y=0.;

    for (auto& prt: particles) {
        sum_x += prt[1];
        sum_y += prt[2];
    };
    mean_x = sum_x/n_particles;
    mean_y = sum_y/n_particles;
    std::cout<<"PREDICTED POS: "<<mean_x<<" "<<mean_y<<std::endl;
};

void ParticleFilter::place_particles() {
    std::uniform_real_distribution<float> x_unif(X_MIN, X_MAX);
    std::uniform_real_distribution<float> y_unif(Y_MIN, Y_MAX);

    for (int i=0; i<n_particles; i++) {
        float x = x_unif(re);
        float y = y_unif(re);
    
        float weight = 1./n_particles; 
        float pos_x = x; 
        float pos_y = y; 
        std::vector<float> this_particle {weight, pos_x, pos_y};
        particles.push_back(this_particle);
    };
    find_mean();
};

void ParticleFilter::update(float cmd_dist, float cmd_heading) {
    float dist_var = cmd_dist/4.;
    float heading_var = .075;
    std::normal_distribution<float> dist_distribution(0., dist_var);
    std::normal_distribution<float> heading_distribution(0., heading_var);

    // Update true position, with added gaussian noise
    float dist_noise = dist_distribution(re);
    float heading_noise = heading_distribution(re);

    float actual_dist = cmd_dist + dist_noise;
    float actual_heading = cmd_heading + heading_noise;

    x_pos += cos(actual_heading) * actual_dist;
    y_pos += sin(actual_heading) * actual_dist;

    std::cout<<"TRUE POS: "<<x_pos<<" "<<y_pos<<std::endl;

    // Update particles with same gaussian noise that was applied to true position
    for (auto& particle: particles) {
        float noisy_dist = cmd_dist + dist_distribution(re);
        float noisy_heading = cmd_heading + heading_distribution(re);
        particle[1] += cos(noisy_heading) * noisy_dist;
        particle[2] += sin(noisy_heading) * noisy_dist;
    };
};
 
void ParticleFilter::set_weights(std::vector<Landmark> landmarks) {
    detected_landmarks = landmarks;
    for (Landmark& lm: detected_landmarks) {
        float true_dist = std::hypot(lm.x-x_pos, lm.y-y_pos);
        for (auto& particle: particles) {
            float dist = std::hypot(lm.x-particle[1], lm.y-particle[2]);
            particle[0] *=  normal_dist(dist, true_dist, sensor_error);
        };
    }; 

};

void ParticleFilter::resample() {
    std::vector<float> weights;
    std::vector<std::vector<float>> resampled_particles;

    for (auto& particle: particles) {
        weights.push_back(particle[0]+TINY); //Add a small number to avoid a round by zero error
    };
    
    std::discrete_distribution<int> normalized(weights.begin(), weights.end());
    //for (double x: normalized.probabilities()) std::cout << x << std::endl;
    for (int i=0; i<n_particles; i++) {
        int index = normalized(gen);
        resampled_particles.push_back(particles[index]);
    };

    particles = resampled_particles;
    find_mean();

};

void ParticleFilter::run_filter(float vel, float hdg, std::vector<Landmark> lms) {
    update(vel, hdg);
    set_weights(lms);
    resample();
};

void ParticleFilter::plot_results(float x_lims[2], float y_lims[2], std::string save_path) {
    std::vector<float> x_data;
    std::vector<float> y_data;
    std::vector<float> true_x;
    std::vector<float> true_y;
    std::vector<float> x_pred;
    std::vector<float> y_pred;
    std::vector<float> landmark_x;
    std::vector<float> landmark_y;

    for (auto& particle: particles) {
        x_data.push_back(particle[1]);
        y_data.push_back(particle[2]);
        true_x.push_back(x_pos);
        true_y.push_back(y_pos);
	x_pred.push_back(mean_x);
        y_pred.push_back(mean_y);
    };
    
    for (auto& landmark: detected_landmarks) {
        landmark_x.push_back(landmark.x);
        landmark_y.push_back(landmark.y);
    };

    plt::xlim(x_lims[0], x_lims[1]);
    plt::ylim(y_lims[0], y_lims[1]);
    plt::scatter(x_data, y_data, 1.);
    plt::scatter(true_x, true_y, 6.);
    plt::scatter(x_pred, y_pred, 6.);
    plt::scatter(landmark_x, landmark_y, 6.);
    plt::title("Timestep "+std::to_string(plot_ctr));
    plt::save(save_path+std::to_string(plot_ctr));
    plt::close();
    //plt::show();
    plot_ctr++;
};

