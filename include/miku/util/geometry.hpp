#pragma once

#include <cmath>
#include <vector>
#include <string>
#define FMT_HEADER_ONLY
#include "miku/libs/fmt/core.h"

enum AngleType {
    COMPASS,
    STANDARD
};

enum AngleUnit {
    DEGREES,
    RADIANS
};

template<AngleUnit U = RADIANS, AngleType T = STANDARD>
struct AngleTemplate {
    float value = 0.0;

    constexpr AngleTemplate(float value = 0.0) : value(value) {}

    template<AngleUnit OtherU, AngleType OtherT>
    constexpr AngleTemplate(const AngleTemplate<OtherU, OtherT>& other) {
        float tmp = other.value;

        if constexpr (OtherU == DEGREES) tmp = tmp * M_PI / 180.0;
        if constexpr (OtherT == COMPASS) tmp = M_PI / 2.0 - tmp;  // compass → standard

        if constexpr (T == COMPASS) tmp = M_PI / 2.0 - tmp;        // standard → compass

        if constexpr (U == DEGREES) tmp = tmp * 180.0 / M_PI;

        value = tmp;
    }
    operator float() const { return value; }
    AngleTemplate operator+(const AngleTemplate& other) const {
        return AngleTemplate(value + other.value);
    }
    AngleTemplate operator+(const float& other) const {
        return AngleTemplate(value + other);
    }
    AngleTemplate& operator+=(const AngleTemplate& other) {
        value += other.value;
        return *this;
    }
    AngleTemplate& operator+=(const float& other) {
        value += other;
        return *this;
    }
    AngleTemplate operator-(const AngleTemplate& other) const {
        return AngleTemplate(value - other.value);
    }
    AngleTemplate operator-(const float& other) const {
        return AngleTemplate(value - other);
    }
    AngleTemplate& operator-=(const AngleTemplate& other) {
        value -= other.value;
        return *this;
    }
    AngleTemplate& operator-=(const float& other) {
        value -= other;
        return *this;
    }
    AngleTemplate operator*(float scalar) const {
        return AngleTemplate(value * scalar);
    }
    AngleTemplate& operator*=(float scalar) {
        value *= scalar;
        return *this;
    }
    AngleTemplate operator/(float scalar) const {
        return AngleTemplate(value / scalar);
    }
    AngleTemplate& operator/=(float scalar) {
        value /= scalar;
        return *this;
    }
    ~AngleTemplate() = default;
    template<AngleType K = T>
    inline constexpr std::enable_if_t<K == COMPASS, AngleTemplate<U, STANDARD>> standard() const {
        if constexpr (U == DEGREES) return AngleTemplate<DEGREES, STANDARD>(90.0 - value);
        return AngleTemplate<RADIANS, STANDARD>(M_PI / 2.0 - value);
    };
    template<AngleType K = T>
    inline constexpr std::enable_if_t<K == STANDARD, AngleTemplate<U, COMPASS>> compass() const {
        if constexpr (U == DEGREES) return AngleTemplate<DEGREES, COMPASS>(90.0 - value);
        return AngleTemplate<RADIANS, COMPASS>(M_PI / 2.0 - value);
    };
    template<AngleUnit V = U>
    inline constexpr std::enable_if_t<V == DEGREES, AngleTemplate<RADIANS, T>> radians() const {
        return AngleTemplate<RADIANS, T>(value * M_PI / 180.0);
    };
    template<AngleUnit V = U>
    inline constexpr std::enable_if_t<V == RADIANS, AngleTemplate<DEGREES, T>> degrees() const {
        return AngleTemplate<DEGREES, T>(value * 180.0 / M_PI);
    };
    // wrap to [-180, 180) for degrees or [-pi, pi) for radians
    inline float wrap() const {
        if constexpr (U == DEGREES) {
            float mod = std::fmod(value + 180.0, 360.0);
            if(mod < 0) mod += 360.0;
            return mod - 180.0;
        } else {
            float mod = std::fmod(value + M_PI, 2.0 * M_PI);
            if(mod < 0) mod += 2.0 * M_PI;
            return mod - M_PI;
        }
    };
    // wrap to [0, 360) for degrees or [0, 2pi) for radians
    inline float norm() const {
        if constexpr (U == DEGREES) {
            float mod = std::fmod(value, 360.0);
            if(mod < 0) mod += 360.0;
            return mod;
        } else {
            float mod = std::fmod(value, 2.0 * M_PI);
            if(mod < 0) mod += 2.0 * M_PI;
            return mod;
        }
    };
};

using standard_radians = AngleTemplate<RADIANS, STANDARD>;
using standard_degrees = AngleTemplate<DEGREES, STANDARD>;
using compass_radians = AngleTemplate<RADIANS, COMPASS>;
using compass_degrees = AngleTemplate<DEGREES, COMPASS>;

inline constexpr standard_degrees operator"" deg(long double value) {
    return standard_degrees(static_cast<float>(value));
}
inline constexpr standard_degrees operator"" deg(unsigned long long value) {
    return standard_degrees(static_cast<float>(value));
}
inline constexpr standard_radians operator"" rad(long double value) {
    return standard_radians(static_cast<float>(value));
}
inline constexpr standard_radians operator"" rad(unsigned long long value) {
    return standard_radians(static_cast<float>(value));
}

struct Point {
    float x;
    float y;

    Point(float x = 0.0, float y = 0.0) 
        : x(x), y(y) {}

    float magnitude() const {
        return std::hypot(x, y);
    }
    standard_radians angle_to(const Point& other) const {
        return standard_radians(atan2(other.y - y, other.x - x));
    }
    float distance_to(const Point& other) const {
        return std::hypot(other.x - x, other.y - y);
    }
    std::string to_string() const {
        return fmt::format("({:.1f}, {:.1f})", x, y);
    }
};

struct Pose {
    float x; 
    float y;
    standard_radians theta;

    Pose(float x = 0.0, float y = 0.0, float theta = 0.0)
        : x(x), y(y), theta(theta) {}

    Pose(Point position, standard_radians heading)
        : x(position.x), y(position.y), theta(heading) {}

    float magnitude() const {
        return std::hypot(x, y);
    }
    // standard_radians angle_to(const Point& other) const { // bugged
    //     return standard_radians(atan2(other.y - y, other.x - x));
    // }
    float distance_to(const Point& other) const {
        return std::hypot(other.x - x, other.y - y);
    }
    std::string to_string() const {
        return fmt::format("{:.1f} {:.1f} @{:.1f}", x, y, float(compass_degrees(theta).wrap()));
    }
};

struct Shape {
    virtual bool ray_intersect(Point ray, float dx, float dy) const = 0;
    virtual ~Shape() = default;
};

struct Polygon : Shape {
    std::vector<Point> ccw_vertices;

    Polygon(std::vector<Point> vertices = {}) : ccw_vertices(std::move(vertices)) {}
    Polygon(std::initializer_list<Point> vertices) : ccw_vertices(vertices) {}

    bool ray_intersect(Point ray, float dx, float dy) const override {
        const size_t n = ccw_vertices.size();
        if (n < 2) return false;

        for (size_t i = 0; i < n; ++i) {
            const Point& p1 = ccw_vertices[i];
            const Point& p2 = ccw_vertices[(i + 1) % n];

            float ex = p2.x - p1.x;
            float ey = p2.y - p1.y;
            float denom = dx * ey - dy * ex;
            if (std::fabs(denom) < 1e-6f) continue; // parallel or nearly parallel

            float rx = p1.x - ray.x;
            float ry = p1.y - ray.y;
            float t = (rx * ey - ry * ex) / denom;
            if (t < 0.0f) continue; // intersection is behind the ray origin

            float u = (rx * dy - ry * dx) / denom;
            if (u >= 0.0f && u <= 1.0f) return true;
        }

        return false;
    }
};

struct Circle : Shape {
    Point center;
    float radius;
    Circle(Point center, float radius) : center(center), radius(radius) {}

    bool ray_intersect(Point ray, float dx, float dy) const override {
        float fx = ray.x - center.x;
        float fy = ray.y - center.y;

        float a = dx * dx + dy * dy;
        float b = 2 * (fx * dx + fy * dy);
        float c = fx * fx + fy * fy - radius * radius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return false; // no intersection
        } else {
            float t1 = (-b - sqrt(discriminant)) / (2 * a);
            float t2 = (-b + sqrt(discriminant)) / (2 * a);
            return (t1 >= 0 || t2 >= 0); // at least one intersection in the ray direction
        }

    }
};