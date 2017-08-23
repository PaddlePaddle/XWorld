// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <cmath>
#include "simulator.h"

namespace simulator {

class Vec3 {
  public:
    double x;
    double y;
    double z;
    static constexpr double EPS = 1e-3;

    // Note: min() is the minimum positive number
    Vec3() : x(-std::numeric_limits<double>::max()) {
        y = x;
        z = x;
    }

    Vec3(double x_, double y_, double z_)
            : x(x_), y(y_), z(z_) {}

    Vec3 operator-(const Vec3& l) const {
        return Vec3(x - l.x, y - l.y, z - l.z);
    }

    Vec3 operator+(const Vec3& l) const {
        return Vec3(x + l.x, y + l.y, z + l.z);
    }

    bool operator==(const Vec3& l) const {
        return square_distance(l) < EPS;
    }

    double L2_norm() const {
        return x * x + y * y + z * z;
    }

    double square_distance(const Vec3& l) const {
        Vec3 diff = (*this) - l;
        return diff.L2_norm();
    }

    // currently get_direction ignores the z axis
    // get the direction of l wrt *this
    // It returns one of eight directions, every two directions
    // separated by 45 degrees: "north", "northeast", "east" ...
    std::string get_direction(const Vec3& l) const {
        if (l == (*this)) {
            return "";
        }
        double delta = atan(1.0) / 2;
        Vec3 vec = l - (*this);
        double angle = atan2(vec.y, vec.x) + delta * 8; // [0, 2pi]
        std::vector<std::string> dir = {
            "southeast", "south", "southwest", "west",
            "northwest", "north", "northeast", "east"};
        std::vector<int> start = {9, 11, 13, 15, 1, 3, 5, 7};
        for (size_t i = 0; i < dir.size(); i ++) {
            if (angle >= start[i] * delta && angle < (start[i] + 2) * delta) {
                return dir[i];
            }
        }
        return "west"; // [2pi-delta, 2pi], [0, delta]
    }
};

/*
  A world consists of many entities.
  An entity might be an agent, an object, a landmark, etc.
  This is the general data structure for representing a thing in different games
 */
class Entity {
  public:
    Entity() : id(""), type(""), location(Vec3()) {}
    std::string property(std::string key) const {
        CHECK(properties_.find(key) != properties_.end())
                << "unrecognized key!";
        return properties_.at(key);
    }
    void set_property(std::string key, std::string value) {
        properties_[key] = value;
    }
    // get the keys of all properties
    std::vector<std::string> all_properties() {
        std::vector<std::string> keys;
        for (const auto& p : properties_) {
            keys.push_back(p.first);
        }
        return keys;
    }
    std::string id;      // name + id
    std::string type;    // "agent", "goal", "block", "dummy", "world"
    Vec3 location;

  protected:
    // e.g., "color" -> "red"
    //       "name" -> "apple"
    //       "shape" -> "rectangle"
    std::unordered_map<std::string, std::string> properties_;
};

typedef std::shared_ptr<Entity> EntityPtr;

} // namespace simulator
