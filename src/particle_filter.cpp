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
  for (size_t i = 0; i < num_particles; i++) {
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
  // NOTE: this method will NOT be called by the grading code. But you will
  // probably find it useful to
  //   implement this method and use it as a helper during the updateWeights
  //   phase.

  // Find the predicted measurement that is closest to each observed
  // measurement and assign the observed measurement to this particular
  // landmark.
  for (size_t i = 0; i < observations.size(); i++) {
    // grab current observation
    LandmarkObs o = observations[i];

    // init minimum distance to maximum possible
    double min_dist = std::numeric_limits<double>::max();

    // init id of landmark from map placeholder to be associated with the
    // observation
    int map_id = -1;

    for (unsigned int j = 0; j < predicted.size(); j++) {
      // grab current prediction
      LandmarkObs p = predicted[j];

      // get distance between current/predicted landmarks
      double cur_dist = dist(o.x, o.y, p.x, p.y);

      // find the predicted landmark nearest the current observed landmark
      if (cur_dist < min_dist) {
        min_dist = cur_dist;
        map_id = p.id;
      }
    }

    // set the observation's id to the nearest predicted landmark's id
    observations[i].id = map_id;
  }
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

  // for each particle...
  for (size_t i = 0; i < num_particles; i++) {
    // get the particle x, y coordinates
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    // create a vector to hold the map landmark locations predicted to be within
    // sensor range of the particle
    std::vector<LandmarkObs> predictions;

    // for each map landmark...
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // get id and x,y coordinates
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;

      // only consider landmarks within sensor range of the particle (rather
      // than using the "dist" method considering a circular region around the
      // particle, this considers a rectangular region but is computationally
      // faster)
      if (fabs(lm_x - p_x) <= sensor_range &&
          fabs(lm_y - p_y) <= sensor_range) {
        // add prediction to vector
        predictions.push_back(LandmarkObs{lm_id, lm_x, lm_y});
      }
    }

    // create and populate a copy of the list of observations transformed from
    // vehicle coordinates to map coordinates
    std::vector<LandmarkObs> transformed_os;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double t_x = cos(p_theta) * observations[j].x -
                   sin(p_theta) * observations[j].y + p_x;
      double t_y = sin(p_theta) * observations[j].x +
                   cos(p_theta) * observations[j].y + p_y;
      transformed_os.push_back(LandmarkObs{observations[j].id, t_x, t_y});
    }

    // perform dataAssociation for the predictions and transformed observations
    // on current particle
    dataAssociation(predictions, transformed_os);

    // reinit weight
    particles[i].weight = 1.0;

    for (size_t j = 0; j < transformed_os.size(); j++) {
      // placeholders for observation and associated prediction coordinates
      double o_x, o_y, pr_x, pr_y;
      o_x = transformed_os[j].x;
      o_y = transformed_os[j].y;

      int associated_prediction = transformed_os[j].id;

      // get the x,y coordinates of the prediction associated with the current
      // observation
      for (size_t k = 0; k < predictions.size(); k++) {
        if (predictions[k].id == associated_prediction) {
          pr_x = predictions[k].x;
          pr_y = predictions[k].y;
        }
      }

      // calculate weight for this observation with multivariate Gaussian
      double s_x = std_landmark[0];
      double s_y = std_landmark[1];
      double obs_w = (1 / (2 * M_PI * s_x * s_y)) *
                     exp(-(pow(pr_x - o_x, 2) / (2 * pow(s_x, 2)) +
                           (pow(pr_y - o_y, 2) / (2 * pow(s_y, 2)))));

      // product of this observation weight with total observations weight
      particles[i].weight *= obs_w;
    }
  }
}

void ParticleFilter::resample() {
  // TODO(liam): Resample particles w replacement with probability proportional
  // to their weight.
  // NOTE: You may find std::discrete_distribution helpful here
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  std::vector<Particle> new_particles;

  // get all of the current weights
  std::vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  std::uniform_int_distribution<int> uniintdist(0, num_particles - 1);
  auto index = uniintdist(random_engine);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(random_engine) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
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
