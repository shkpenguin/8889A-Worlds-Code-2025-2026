#include "api.h"
#include "config.h"
#include "miku/util/utils.hpp"
#include "miku/devices/distance.hpp"
#include "miku/system/log.hpp"
#include "miku/system/odom.hpp"
#include "miku/system/drive.hpp"
#include "miku/libs/pcg32.h"
#include <cmath>
#include <random>

#define NUM_PARTICLES 500

Pose pose_delta;
Pose get_pose_delta() { return pose_delta; };
float get_x_delta() { return pose_delta.x; };
float get_y_delta() { return pose_delta.y; };
standard_radians get_theta_delta() { return pose_delta.theta; };

pcg32 rng(std::random_device{}());

inline float random_float(float a, float b) {
    return a + rng.nextFloat() * (b - a);
}

inline float normal_clt(float mean, float stddev) {
    float sum = 0.0f;
    for (int i = 0; i < 12; ++i) {
        sum += rng.nextFloat();
    }
    return mean + stddev * (sum - 6.0f);
}

std::ofstream file;
std::ostringstream log_buffer;

float max_distance_error = 12.0f;
float min_odom_noise = 0.5f;

void set_max_distance_error(float error) {
    max_distance_error = error;
}
void set_min_odom_noise(float noise) {
    min_odom_noise = noise;
}

enum WallID {
    LEFT_WALL,
    RIGHT_WALL,
    TOP_WALL,
    BOTTOM_WALL,
    NOT_IN_FIELD,
    BAD_INTERSECT
};

struct WallEstimate {
    WallID wall_id;
    float distance;
};

miku::Distance north_distance(0, 3, -3.4, 4.3, 0);
miku::Distance north_distance_2(1, 1, 3.4, 4.3, 0);
miku::Distance nw_distance(2, 15, -6, -2.3, -45);
miku::Distance ne_distance(3, 7, 6, -2.3, 45);
miku::Distance south_distance(4, 21, 2.75, -3.75, 180);
miku::Distance sw_distance(5, 6, 4.75, -4, -138);
miku::Distance se_distance(6, 16, -4.75, -4, 138);
miku::Distance west_distance(7, 5, -4, 5.25, -90);
miku::Distance east_distance(8, 2, 4, 5.25, 90);
std::vector<miku::Distance*> distance_sensors = {
    &north_distance,
    &north_distance_2,
    &nw_distance,
    &ne_distance,
    &south_distance,
    &sw_distance,
    &se_distance,
    &west_distance,
    &east_distance
};

struct Particle {
    Point position; 
    float weight; 
    std::vector<WallEstimate> sensor_readings;

    Particle() 
        : position(0,0),
        weight(1.0),
        sensor_readings(distance_sensors.size()) {}
};
std::vector<Particle> particles = std::vector<Particle>(NUM_PARTICLES);

std::vector<Shape*> field_objects = {
    new Circle({-47, -69}, 2.5),
    new Circle({47, -69}, 2.5),
    new Circle({-47, 69}, 2.5),
    new Circle({47, 69}, 2.5),

    new Polygon({{-71, -65}, {-65, -71}}),
    new Polygon({{-71, 65}, {-65, 71}}),
    new Polygon({{65, -71}, {71, -65}}),
    new Polygon({{65, 71}, {71, 65}})
};

void add_field_object(Shape* obj) {
    field_objects.push_back(obj);
}

WallEstimate get_expected_reading(Point particle_position, miku::Distance* sensor, float cos_theta, float sin_theta) {

    float sensor_y = particle_position.y - sensor->offset_x * cos_theta + sensor->offset_y * sin_theta;
    float sensor_x = particle_position.x + sensor->offset_x * sin_theta + sensor->offset_y * cos_theta;
    
    float dx, dy;
    
    // Compute ray direction by rotating robot forward vector (cos_theta, sin_theta)
    // by the sensor's orientation_angle. Using precomputed cos/sin avoids trig here.
    float ca = sensor->cos_orient;
    float sa = sensor->sin_orient;
    // Use (theta - alpha): dx = cos(theta - alpha), dy = sin(theta - alpha)
    dx = cos_theta * ca + sin_theta * sa;
    dy = sin_theta * ca - cos_theta * sa;

    WallID intersect;
    float tMin = std::numeric_limits<float>::infinity();

    // Check vertical walls
    if (fabs(dx) > 1e-6) {
        float t1 = (-HALF_FIELD - sensor_x) / dx;
        if (t1 > 0) {
            float y1 = sensor_y + t1 * dy;
            if (t1 > 0 && y1 >= -HALF_FIELD && y1 <= HALF_FIELD && t1 < tMin) {
                tMin = t1;
                intersect = (dx < 0) ? LEFT_WALL : RIGHT_WALL;
            }
        }
        float t2 = (HALF_FIELD - sensor_x) / dx;
        if (t2 > 0) {
            float y2 = sensor_y + t2 * dy;
            if (t2 > 0 && y2 >= -HALF_FIELD && y2 <= HALF_FIELD && t2 < tMin) {
                tMin = t2;
                intersect = (dx < 0) ? LEFT_WALL : RIGHT_WALL;
            }
        }
    }

    // Check horizontal walls
    if (fabs(dy) > 1e-6) {
        float t3 = (-HALF_FIELD - sensor_y) / dy;
        if (t3 > 0) {
            float x3 = sensor_x + t3 * dx;
            if (t3 > 0 && x3 >= -HALF_FIELD && x3 <= HALF_FIELD && t3 < tMin) {
                tMin = t3;
                intersect = (dy < 0) ? BOTTOM_WALL : TOP_WALL;
            }
        }
        float t4 = (HALF_FIELD - sensor_y) / dy;
        if (t4 > 0) {
            float x4 = sensor_x + t4 * dx;
            if (t4 > 0 && x4 >= -HALF_FIELD && x4 <= HALF_FIELD && t4 < tMin) {
                tMin = t4;
                intersect = (dy < 0) ? BOTTOM_WALL : TOP_WALL;
            }
        }
    }

    if (!std::isfinite(tMin)) return WallEstimate{NOT_IN_FIELD, 0}; // no valid hit

    float x_intersect = sensor_x + tMin * dx;
    float y_intersect = sensor_y + tMin * dy;

    for(const auto& obj : field_objects) {
        if(obj->ray_intersect(Point(sensor_x, sensor_y), dx, dy)) {
            return WallEstimate{BAD_INTERSECT, 0}; // ray hits an object before a wall
        }
    }

    return WallEstimate{intersect, tMin}; // distance along ray

};

void set_particles_point(Point center) {

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = center;
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

void set_particles_uniform(Point center, float length) {

    for(int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = Point(
            clamp(center.x + random_float(-length / 2, length / 2), -HALF_FIELD, HALF_FIELD), 
            clamp(center.y + random_float(-length / 2, length / 2), -HALF_FIELD, HALF_FIELD)
        );
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }

}

void reset(Pose new_pose) {
    set_pose(new_pose);
    set_particles_point(Point(new_pose.x, new_pose.y));
};
void reset(Point new_position) {
    set_position(new_position);
    set_particles_point(new_position);
};

#define DIST_RESET_MAX_ERROR 16.0f

void distance_reset(Point estimate, float particle_spread) {

    std::vector<WallEstimate> expected_readings;
    float cos_theta = cos(get_heading());
    float sin_theta = sin(get_heading());

    for(const auto& sensor : distance_sensors) {
        expected_readings.push_back(
            get_expected_reading(
                Point(estimate.x, estimate.y),
                sensor,
                cos_theta,
                sin_theta));
    }

    float best_dev_x = std::numeric_limits<float>::infinity();
    float best_dev_y = std::numeric_limits<float>::infinity();
    float best_x = estimate.x;
    float best_y = estimate.y;

    for(size_t i = 0; i < distance_sensors.size(); ++i) {

        miku::Distance* sensor = distance_sensors[i];
        sensor->update_reading();

        if(!sensor->get_enabled()) continue;
        if(!sensor->get_valid()) continue;

        WallEstimate expected = expected_readings[i];
        if(expected.wall_id == BAD_INTERSECT || expected.wall_id == NOT_IN_FIELD) continue;

        float reading = sensor->get_reading();
        float deviation = fabs(reading - expected.distance);
        if(deviation > DIST_RESET_MAX_ERROR) continue;

        float corrected_x = estimate.x;
        float corrected_y = estimate.y;

        float o_x = sensor->offset_x * sin_theta + sensor->offset_y * cos_theta;
        float o_y = -sensor->offset_x * cos_theta + sensor->offset_y * sin_theta;

        float dx, dy;
        float ca = sensor->cos_orient;
        float sa = sensor->sin_orient;
        dx = cos_theta * ca + sin_theta * sa;
        dy = sin_theta * ca - cos_theta * sa;

        switch(expected.wall_id) {
            case LEFT_WALL:
                corrected_x = -HALF_FIELD - reading * dx - o_x;
                if(deviation < best_dev_x) {
                    best_dev_x = deviation;
                    best_x = corrected_x;
                    std::cout << "   new best_x=" << best_x << " (LEFT)\n";
                }
                break;

            case RIGHT_WALL:
                corrected_x = HALF_FIELD - reading * dx - o_x;
                if(deviation < best_dev_x) {
                    best_dev_x = deviation;
                    best_x = corrected_x;
                    std::cout << "   new best_x=" << best_x << " (RIGHT)\n";
                }
                break;

            case TOP_WALL:
                corrected_y = HALF_FIELD - reading * dy - o_y;
                if(deviation < best_dev_y) {
                    best_dev_y = deviation;
                    best_y = corrected_y;
                    std::cout << "   new best_y=" << best_y << " (TOP)\n";
                }
                break;

            case BOTTOM_WALL:
                corrected_y = -HALF_FIELD - reading * dy - o_y;
                if(deviation < best_dev_y) {
                    best_dev_y = deviation;
                    best_y = corrected_y;
                    std::cout << "   new best_y=" << best_y << " (BOTTOM)\n";
                }
                break;

            default:
                break;
        }

    }

    Point corrected_pose = estimate;

    if(best_dev_x < std::numeric_limits<float>::infinity()) {
        corrected_pose.x = best_x;
    }

    if(best_dev_y < std::numeric_limits<float>::infinity()) {
        corrected_pose.y = best_y;
    }

    set_position(Point(
        clamp(corrected_pose.x, -HALF_FIELD, HALF_FIELD),
        clamp(corrected_pose.y, -HALF_FIELD, HALF_FIELD)
    ));

    if(particle_spread > 0.0) {
        set_particles_uniform(Point(corrected_pose.x, corrected_pose.y), particle_spread);
    } else {
        set_particles_point(Point(corrected_pose.x, corrected_pose.y));
    }

}

void distance_reset(float particle_spread) {
    distance_reset(get_position(), particle_spread);
}

// compute weighted average position of particles
Point get_current_belief() {
    float x = 0.0;
    float y = 0.0;
    float total_weight = 0.0;

    for(const auto& particle : particles) {
        x += particle.position.x * particle.weight;
        y += particle.position.y * particle.weight;
        total_weight += particle.weight;
    }

    if(total_weight > 0) {
        x /= total_weight;
        y /= total_weight;
    }

    return Point(x, y);
}

Point prev_raw_motor_pos = {0, 0};
compass_degrees prev_theta_raw = 0;

float imu_cw_drift = 356.0f;
float imu_ccw_drift = 356.0f;

Pose compute_odometry_delta() {

    Point raw_motor_pos = get_drive_position();
    // read imu and convert to standard radians immediately
    compass_degrees theta_raw = get_imu_raw_heading();

    // compute signed shortest difference in radians
    float imu_theta_delta = -1.0f * ((theta_raw - prev_theta_raw).radians());
    imu_theta_delta *= 360.0f / (imu_theta_delta > 0 ? imu_cw_drift : imu_ccw_drift);

    // update heading variables
    standard_radians heading_new = get_pose().theta + imu_theta_delta;
    standard_radians avg_heading = get_pose().theta + imu_theta_delta / 2.0f;

    float left_torque = get_drive_torque().x;
    float right_torque = get_drive_torque().y;

    float left_delta = left_torque < 0.3 ? raw_motor_pos.x - prev_raw_motor_pos.x : 0;
    float right_delta = right_torque < 0.3 ? raw_motor_pos.y - prev_raw_motor_pos.y : 0;

    float left_delta_in = left_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;
    float right_delta_in = right_delta * WHEEL_DIAMETER * GEAR_RATIO * M_PI / 360.0f;

    float mid_delta_in = (left_delta_in + right_delta_in) / 2.0f;

    float local_y = 0;
    float local_x = 0;

    if (std::fabs(float(imu_theta_delta)) < 1e-6f) local_y = mid_delta_in;
    else local_y = 2.0f * sin(float(imu_theta_delta) / 2.0f) * (mid_delta_in / float(imu_theta_delta));

    Pose pose_delta;
    pose_delta.x = local_y * cos(float(avg_heading));
    pose_delta.y = local_y * sin(float(avg_heading));
    pose_delta.theta = imu_theta_delta;

    prev_raw_motor_pos = raw_motor_pos;
    prev_theta_raw = theta_raw;

    return pose_delta;
}

#define ODOM_STDEV 1.0

void update_previous_belief(Pose robot_speed) {

    Point prev_belief = get_position();
    set_position(Point(
        clamp(prev_belief.x + robot_speed.x, -HALF_FIELD, HALF_FIELD),
        clamp(prev_belief.y + robot_speed.y, -HALF_FIELD, HALF_FIELD)
    ));
    
    float motion = robot_speed.magnitude() * ODOM_STDEV;
    float odom_stdev = std::max(motion, min_odom_noise);

    for(auto &particle : particles) {
        particle.position.x = clamp(particle.position.x + robot_speed.x + normal_clt(0.0f, odom_stdev), -HALF_FIELD, HALF_FIELD);
        particle.position.y = clamp(particle.position.y + robot_speed.y + normal_clt(0.0f, odom_stdev), -HALF_FIELD, HALF_FIELD);
    }

}

Template particle_filter_template(0, {
    {{DataType::UINT32}, 1}, // timestamp
    {{DataType::FLOAT, DataType::FLOAT, DataType::FLOAT}, 1}, // x, y, heading
    {{DataType::UINT8, DataType::UINT8, DataType::FLOAT, DataType::FLOAT}, (int) distance_sensors.size()}, // distance id, valid, distance, size
    {{DataType::FLOAT, DataType::FLOAT, DataType::FLOAT}, NUM_PARTICLES / 10} // particle x, y, weight (every 10th particle)
});

void update_particle_weights(Pose pose_delta) {

    std::vector<std::vector<double>> log_values;
    log_values.push_back({(double) pros::millis()});
    log_values.push_back({get_position().x, get_position().y, get_heading()});

    standard_radians robot_theta = get_heading();
    float sin_theta = sin(robot_theta);
    float cos_theta = cos(robot_theta);

    std::vector<bool> valid_sensors(distance_sensors.size(), true);
    std::vector<WallEstimate> current_wall_estimates(distance_sensors.size());

    float current_max_error = max_distance_error;
    if(fabs(pose_delta.theta.degrees()) > 1.0deg && pose_delta.magnitude() < 0.5) {
        current_max_error = 1.0;
    }

    for(auto &sensor : distance_sensors) {
        sensor->update_reading();
    }

    // max error check
    for(size_t i = 0; i < distance_sensors.size(); ++i) {
        current_wall_estimates[i] = get_expected_reading(
            get_position(),
            distance_sensors[i],
            cos_theta, 
            sin_theta);
        float expected = current_wall_estimates[i].distance;
        if(fabs(expected - distance_sensors[i]->get_reading()) > current_max_error) valid_sensors[i] = false;
    }

    // check for particles with invalid readings
    for(size_t i = 0; i < particles.size(); ++i) {
        for(size_t j = 0; j < distance_sensors.size(); ++j) {
            particles[i].sensor_readings[j] = get_expected_reading(
                particles[i].position, 
                distance_sensors[j],
                cos_theta,
                sin_theta);
            if(particles[i].sensor_readings[j].wall_id == BAD_INTERSECT) valid_sensors[j] = false;
        }
    }

    // If multiple sensors are seeing the same wall from the current robot pose,
    // keep only the one whose beam is most perpendicular to that wall.
    int best_sensor_per_wall[4] = {-1, -1, -1, -1};
    float best_score_per_wall[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
    int wall_counts[4] = {0, 0, 0, 0};

    for(size_t i = 0; i < distance_sensors.size(); ++i) {
        if(!valid_sensors[i]) continue;
        WallID wall = current_wall_estimates[i].wall_id;
        if(wall < LEFT_WALL || wall > BOTTOM_WALL) continue;

        float ca = distance_sensors[i]->cos_orient;
        float sa = distance_sensors[i]->sin_orient;
        float dx = cos_theta * ca + sin_theta * sa;
        float dy = sin_theta * ca - cos_theta * sa;
        float score = (wall == LEFT_WALL || wall == RIGHT_WALL) ? fabs(dx) : fabs(dy);

        int wall_index = static_cast<int>(wall);
        ++wall_counts[wall_index];
        if(score > best_score_per_wall[wall_index]) {
            best_score_per_wall[wall_index] = score;
            best_sensor_per_wall[wall_index] = static_cast<int>(i);
        }
    }

    for(int wall_index = 0; wall_index < 4; ++wall_index) {
        if(wall_counts[wall_index] <= 1) continue;
        for(size_t i = 0; i < distance_sensors.size(); ++i) {
            if(!valid_sensors[i]) continue;
            if(static_cast<int>(current_wall_estimates[i].wall_id) != wall_index) continue;
            if(static_cast<int>(i) == best_sensor_per_wall[wall_index]) continue;
            valid_sensors[i] = false;
        }
    }

    for(size_t i = 0; i < distance_sensors.size(); ++i) {
        log_values.push_back({
            (double) distance_sensors[i]->id,
            (double) valid_sensors[i],
            (double) distance_sensors[i]->get_reading(),
            (double) distance_sensors[i]->get_object_size()
        });
    }

    float total_weight = 0.0;

    for(int i = 0; i < NUM_PARTICLES; ++i) {

        float weight = 1.0;

        for(int j = 0; j < distance_sensors.size(); ++j) {
            if(!valid_sensors[j] || !distance_sensors[j]->get_enabled() || !distance_sensors[j]->get_valid()) continue;
            WallEstimate expected = particles[i].sensor_readings[j];
            if(expected.wall_id == NOT_IN_FIELD) {
                weight *= 0.0;
                continue;
            }
            float reading = distance_sensors[j]->get_reading();
            float dev = reading - expected.distance;
            if(fabs(dev) > current_max_error) {
                weight *= 0.0;
                continue;
            }
            float sensor_stdev;
            if(reading < 8) sensor_stdev = 0.1;
            else sensor_stdev = reading * 0.03;
            weight *= std::exp(-(dev * dev) / (2 * sensor_stdev * sensor_stdev));
        }

        particles[i].weight = weight;
        total_weight += weight;

    }

    if(total_weight == 0.0) {
        float weight = 1.0 / NUM_PARTICLES;
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight = weight;
        }
    } else {
        for(int i = 0; i < NUM_PARTICLES; ++i) {
            particles[i].weight /= total_weight;
        }
    }

    for(int i = 0; i < NUM_PARTICLES; i += 10) {
        const auto& particle = particles[i];
        log_values.push_back({particle.position.x, particle.position.y, particle.weight});
    }

    write_data(particle_filter_template, {log_values});

}

void resample_particles() {

    std::vector<Particle> newParticles(NUM_PARTICLES);

    float r = random_float(0, 1.0 / NUM_PARTICLES);
    float c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; m++) {
        float U = r + (float)m / NUM_PARTICLES;
        while (U > c && i < NUM_PARTICLES-1) {
            i++;
            c += particles[i].weight;
        }
        newParticles[m] = particles[i];  // copy particle
    }

    for (auto &p : newParticles) {
        p.weight = 1.0 / NUM_PARTICLES;
    }
    particles = newParticles;

}

void update_position() {
    
    Pose odom_pose_delta = compute_odometry_delta();
    set_heading(get_heading() + odom_pose_delta.theta);

    #if USE_MCL
    update_previous_belief(odom_pose_delta);
    update_particle_weights(odom_pose_delta);

    Point belief = get_current_belief();
    pose_delta = Pose(belief.x - get_position().x, belief.y - get_position().y, odom_pose_delta.theta);
    set_position(belief);

    resample_particles();

    #else
    set_position(Point(
        get_x() + odom_pose_delta.x,
        get_y() + odom_pose_delta.y
    ));
    
    #endif

}