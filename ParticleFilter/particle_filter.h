#include <iostream>
#include <string>
#include <random>
#include <vector>


struct Landmark {
    int id;
    float x;
    float y;
};

class ParticleFilter {
    private:
        float x_pos = 0.;
        float y_pos = 0.;
        float mean_x = 0.;
        float mean_y = 0.;
        int n_particles = 100;
        std::vector<std::vector<float>> particles;
        std::vector<Landmark> detected_landmarks;
        float sensor_error = 0.;
        float X_MIN = -5.;
        float X_MAX = 5.;
        float Y_MIN = -5.;
        float Y_MAX = 5.;
	float TINY = 1e-40;
        int plot_ctr = 1;
        std::random_device rdev{};
        std::default_random_engine re{rdev()};
        std::default_random_engine gen{rdev()};
    public:
        ParticleFilter(float pos[2], int num_particles, std::vector<Landmark> lndmrks, float sensor_noise, float x_bounds[2], float y_bounds[2]);
        float normal_dist(float x, float mean, float var);
        void find_mean();
        void place_particles();
        void update(float cmd_dist, float cmd_heading);
        void set_weights(std::vector<Landmark> landmarks);
        void resample();
        void run_filter(float vel, float hdg, std::vector<Landmark> lms);
	void plot_results(float x_lims[2], float y_lims[2], std::string save_path);
};
