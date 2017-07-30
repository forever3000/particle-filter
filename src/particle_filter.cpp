#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "./particle_filter.h"

// random number engine class that generates pseudo-random numbers.
static std::default_random_engine random_engine;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set the number of particles.
  num_particles = 101;

  // Define normal distribution values for x, y, and theta
  std::normal_distribution<double> x_norm(0.0, std[0]);
  std::normal_distribution<double> y_norm(0.0, std[1]);
  std::normal_distribution<double> theta_norm(0.0, std[2]);

  // Initialize all particles to first position
  // (based on estimates of x, y, theta & their uncertainties from GPS)
  // and all weights to 1.0
  for (size_t i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x + x_norm(random_engine);
    p.y = y + y_norm(random_engine);
    p.theta = theta + theta_norm(random_engine);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  // Define normal distributions for sensor noise
  std::normal_distribution<double> x_norm(0, std_pos[0]);
  std::normal_distribution<double> y_norm(0, std_pos[1]);
  std::normal_distribution<double> theta_norm(0, std_pos[2]);

  // Predict new state for each of the particles
  for (int i = 0; i < num_particles; i++) {
    // if yaw rate is very small, assume we are traveling a straight line
    // and use simpler motion model to avoid division by zero
    if (fabs(yaw_rate) < 1e-5) {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } else {
      particles[i].x += velocity / yaw_rate *
                        (sin(particles[i].theta + yaw_rate * delta_t) -
                         sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate *
                        (cos(particles[i].theta) -
                         cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // add noise
    particles[i].x += x_norm(random_engine);
    particles[i].y += y_norm(random_engine);
    particles[i].theta += theta_norm(random_engine);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
  // TODO(liam): Find the predicted measurement that is closest to each observed
  // measurement and assign the
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will
  // probably find it useful to
  //   implement this method and use it as a helper during the updateWeights
  //   phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
  // TODO(liam): Update the weights of each particle using a mult-variate
  // Gaussian distribution. You can read
  //   more about this distribution here:
  //   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your
  // particles are located
  //   according to the MAP'S coordinate system. You will need to transform
  //   between the two systems. Keep in mind that this transformation requires
  //   both rotation AND translation (but no scaling). The following is a good
  //   resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement
  //   (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
  // TODO(liam): Resample particles w replacement with probability proportional
  // to their weight. NOTE: You may find std::discrete_distribution helpful here
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
}

Particle ParticleFilter::SetAssociations(Particle particle,
                                         std::vector<int> associations,
                                         std::vector<double> sense_x,
                                         std::vector<double> sense_y) {
  // particle: the particle to assign each listed association, and association's
  // (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  // Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

std::string ParticleFilter::getAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

std::string ParticleFilter::getSenseX(Particle best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

std::string ParticleFilter::getSenseY(Particle best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
