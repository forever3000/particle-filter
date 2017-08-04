#include "./particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// random number engine class that generates pseudo-random numbers.
static std::default_random_engine random_engine;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set the number of particles.
  num_particles = 100;

  // Define normal distribution values for x, y, and theta
  std::normal_distribution<double> x_norm(0.0, std[0]);
  std::normal_distribution<double> y_norm(0.0, std[1]);
  std::normal_distribution<double> theta_norm(0.0, std[2]);

  // Initialize all particles to first position
  // (based on estimates of x, y, theta & their uncertainties from GPS)
  // and all weights to 1.0
  for (size_t i = 0; i < num_particles; ++i) {
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

  for (Particle &p : particles) {
    // if yaw rate is very small, assume we are traveling a straight line
    // and use simpler motion model to avoid division by zero
    if (fabs(yaw_rate) < 1e-5) {
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    } else {
      p.x += velocity / yaw_rate *
             (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += velocity / yaw_rate *
             (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;
    }

    // add noise
    p.x += x_norm(random_engine);
    p.y += y_norm(random_engine);
    p.theta += theta_norm(random_engine);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs> &observations) {
  // Find the predicted measurement that is closest to each observed
  // measurement and assign the observed measurement to this particular
  // landmark.
  for (LandmarkObs &o : observations) {
    // set initial nearest distance to very large number
    double nearest_dist = std::numeric_limits<double>::max();

    // find the predicted landmark nearest the current observed landmark
    // and set the observation's id to the nearest predicted landmark's id
    for (LandmarkObs &p : predicted) {
      double current_dist = dist(o.x, o.y, p.x, p.y);
      if (current_dist < nearest_dist) {
        nearest_dist = current_dist;
        o.id = p.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
  // Update the weights of each particle using a mult-variate
  // Gaussian distribution.
  for (Particle &p : particles) {
    double p_x = p.x;
    double p_y = p.y;
    double p_theta = p.theta;

    // initialize particle weight to 1.0
    p.weight = 1.0;

    // find the landmark locations predicted to be within
    // sensor range of the particle
    std::vector<LandmarkObs> predicted;
    for (auto &lm : map_landmarks.landmark_list) {
      if (dist(lm.x_f, lm.y_f, p_x, p_y)) {
        predicted.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
      }
    }

    // transform obvservations from vehicle coordinates to map coordinates
    std::vector<LandmarkObs> transformed_observations;
    for (LandmarkObs &o : observations) {
      double t_x = cos(p_theta) * o.x - sin(p_theta) * o.y + p_x;
      double t_y = sin(p_theta) * o.x + cos(p_theta) * o.y + p_y;

      transformed_observations.push_back(LandmarkObs{o.id, t_x, t_y});
    }

    dataAssociation(predicted, transformed_observations);

    for (LandmarkObs &o : transformed_observations) {
      // find the (x, y) coordinates of the predicted
      // landmark associated with the current observation
      double pr_x, pr_y;
      for (LandmarkObs &pr : predicted) {
        if (pr.id == o.id) {
          pr_x = pr.x;
          pr_y = pr.y;
          break;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      double weight = (1. / (2. * M_PI * std_x * std_y)) *
                      exp(-(pow(pr_x - o.x, 2) / (2 * std_x * std_x) +
                           (pow(pr_y - o.y, 2) / (2 * std_y * std_y))));

      // product of this observation weight with total observations weight
      p.weight *= weight;
    }
  }
}

void ParticleFilter::resample() {
  // Resample particles with probability proportional to their weight.
  std::vector<Particle> resampled;

  // get all of the current weights
  std::vector<double> weights;
  for (Particle &p : particles) {
    weights.push_back(p.weight);
  }

  // generate random starting index for resampling
  std::discrete_distribution<int> rand_index(0, num_particles - 1);
  int index = rand_index(random_engine);

  // new weight distribution
  double max_weight = *std::max_element(weights.begin(), weights.end());
  std::normal_distribution<double> weights_dist(0.0, max_weight);

  double beta = 0.0;
  for (size_t i = 0; i < num_particles; ++i) {
    beta += weights_dist(random_engine) * 2.0;
    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled.push_back(particles[index]);
  }

  particles = resampled;
}

Particle ParticleFilter::setAssociations(Particle particle,
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
