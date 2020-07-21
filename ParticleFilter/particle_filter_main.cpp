#include <iostream>
#include <string>
#include <random>
#include <math.h>
#include <vector>
#include <cmath>
#include "particle_filter.cpp"

int main()
{
    std::string save_path_loc ("/home/tyler/Pictures/particle_filter_results/result") ;   

    float start_pos[2] {0., 0.};
    int num_particles = 3000;
    float sensor_noise = .85;
    float x_bounds[2] {-2, 10.};
    float y_bounds[2] {-5., 5.};

    float x_scope[2] {-1., 10.};
    float y_scope[2] {-5., 5.};

    float vel = .25;
    float hdg = 0.;

    Landmark lm1 = Landmark();
    lm1 = {1, 1.5, 2.};

    Landmark lm2 = Landmark();
    lm2 = {2, 0., -1.};

    Landmark lm3 = Landmark();
    lm3 = {3, 2.5, -.5};

    Landmark lm4 = Landmark();
    lm4 = {4, 6., 1.5};

    Landmark lm5 = Landmark();
    lm5 = {5, 7.5, -3.};

    Landmark lm6 = Landmark();
    lm5 = {6, 4, -4.};

    Landmark lm7 = Landmark();
    lm5 = {7, 7., -3.};

    std::vector<Landmark> lms{lm1, lm2, lm3, lm4, lm5, lm6, lm7};
 
    ParticleFilter filter(start_pos, num_particles, lms, sensor_noise, x_bounds, y_bounds);
    filter.place_particles();
    filter.plot_results(x_scope, y_scope, save_path_loc);

    for (int t=0; t<50; t++) {

        if (t >= 10) {
            hdg = .45;
        };

        if (t >= 25) {
            vel = .15;
            hdg = -1.5708;
        };

        filter.run_filter(vel, hdg, lms);
        filter.plot_results(x_scope, y_scope, save_path_loc);
    };
    
    return 0;
}
