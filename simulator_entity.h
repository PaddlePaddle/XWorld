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
#include <boost/python.hpp>
#include <boost/tuple/tuple.hpp>

#include "simulator_util.h"

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

    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator-(const Vec3& l) const {
        return Vec3(x - l.x, y - l.y, z - l.z);
    }

    Vec3 operator+(const Vec3& l) const {
        return Vec3(x + l.x, y + l.y, z + l.z);
    }

    bool operator==(const Vec3& l) const { return square_distance(l) < EPS; }

    double L2_norm() const { return x * x + y * y + z * z; }

    double square_distance(const Vec3& l) const {
        Vec3 diff = (*this) - l;
        return diff.L2_norm();
    }

    bool defined() const {
        return (x > std::numeric_limits<int>::min() +  EPS) &&
               (y > std::numeric_limits<int>::min() +  EPS) &&
               (z > std::numeric_limits<int>::min() +  EPS);
    }

    void random_loc(int w, int h) {
        x = simulator::util::get_rand_ind(w);
        y = simulator::util::get_rand_ind(h);
        z = 0;
    }

    bool in_boundary(int w, int h) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    void scale(double s) {
        x *= s;
        y *= s;
        z *= s;
    }


    // currently get_direction ignores the z axis
    // get the direction of l wrt *this
    // It returns one of eight directions, every two directions
    // separated by 45 degrees: "north", "northeast", "east" ...
//    std::string get_direction(const Vec3& l) const {
//        if (l == (*this)) {
//            return "";
//        }
//        double delta = atan(1.0) / 2;
//        Vec3 vec = l - (*this);
//        double angle = atan2(vec.y, vec.x) + delta * 8;  // [0, 2pi]
//        std::vector<std::string> dir = {"southeast",
//                                        "south",
//                                        "southwest",
//                                        "west",
//                                        "northwest",
//                                        "north",
//                                        "northeast",
//                                        "east"};
//        std::vector<int> start = {9, 11, 13, 15, 1, 3, 5, 7};
//        for (size_t i = 0; i < dir.size(); i++) {
//            if (angle >= start[i] * delta && angle < (start[i] + 2) * delta) {
//                return dir[i];
//            }
//        }
//        return "west";  // [2pi-delta, 2pi], [0, delta]
//    }
};

/*
  A world consists of many entities.
  An entity might be an agent, an object, a landmark, etc.
  This is the general data structure for representing a thing in different games
 */
struct Entity {
    Entity() {}
    Entity(boost::python::dict e) {
        const boost::python::tuple& l = boost::python::extract<boost::python::tuple>(e["loc"]);
        type = boost::python::extract<std::string>(e["type"]);
        id = boost::python::extract<std::string>(e["id"]);
        loc = Vec3(boost::python::extract<double>(l[0]),
                   boost::python::extract<double>(l[1]),
                   boost::python::extract<double>(l[2]));
        yaw = boost::python::extract<double>(e["yaw"]);
        name = boost::python::extract<std::string>(e["name"]);
        asset_path = boost::python::extract<std::string>(e["asset_path"]);
        color = boost::python::extract<std::string>(e["color"]);
    }
    boost::python::dict to_py_dict() const {
        boost::python::dict d;
        d["type"] = type;
        d["id"] = id;
        d["loc"] = boost::python::make_tuple(loc.x, loc.y, loc.z);
        d["yaw"] = yaw;
        d["name"] = name;
        d["asset_path"] = asset_path;
        d["color"] = color;
        return d;
    }
    std::string type;
    std::string id;
    Vec3 loc;
    double yaw;
    std::string name;
    std::string asset_path;
    std::string color;
};

}  // namespace simulator
