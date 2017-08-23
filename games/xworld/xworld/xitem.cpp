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

#include "dirent.h"
#include "xitem.h"
#include <fstream>
#include <sstream>
#include <gflags/gflags.h>
#include <boost/algorithm/string.hpp>

DEFINE_bool(perturb_goal, false, "whether the goal image is perturbed with randomness");

namespace simulator { namespace xwd {

std::string get_item_repo_path() {
    std::string path = __FILE__;
    int idx = path.find_last_of("/");
    return path.substr(0, idx) + "/../images";
}

// path to the item images
const std::string ItemRepository::img_path_ = get_item_repo_path();

ItemRepository XItem::item_rep_;

std::shared_ptr<ItemNameTree>& ItemNameTree::insert_node(std::string node_name) {
    CHECK_EQ(nodes_.count(node_name), 0);
    nodes_[node_name] = std::make_shared<ItemNameTree>();
    return nodes_[node_name];
}

// item_name is the absolute path
void ItemNameTree::insert_item(std::string item_name) {
    // item_name: <path>/apple_1.jpg
    size_t idx = item_name.find_last_of("_");
    CHECK(idx != std::string::npos);
    CHECK_GT(idx, ItemRepository::img_path_.length());
    size_t start = ItemRepository::img_path_.length() + 1;
    std::string class_name = item_name.substr(start, idx - start);
    leaves_[class_name].push_back(item_name);
}

// node: "a/b/c/"
void ItemNameTree::retrieve_item_classes(std::string node, std::vector<std::string>& classes) {
    if (node == "") {
        classes.clear();
        // the images in current directory
        for (const auto& leaf : leaves_) {
            classes.push_back(leaf.first);
        }
        // the images in subdirectory
        for (const auto& node : nodes_) {
            std::vector<std::string> tmp;
            node.second->retrieve_item_classes("", tmp);
            classes.insert(classes.end(), tmp.begin(), tmp.end());
        }
    } else {
        size_t idx = node.find("/");
        CHECK(idx != std::string::npos) << node;
        CHECK_GT(nodes_.count(node.substr(0, idx)), 0) << "incorrect item path";
        auto child = nodes_[node.substr(0, idx)];
        child->retrieve_item_classes(node.substr(idx + 1), classes);
    }
}

std::vector<std::string> ItemNameTree::retrieve_item_paths(const std::string& class_name) {
    size_t idx = 0;
    std::string cur_name = class_name;
    ItemNameTree* ptr = this;
    while ((idx = cur_name.find("/")) != std::string::npos) {
        std::string subtree = cur_name.substr(0, idx);
        CHECK_GT(ptr->nodes_.count(subtree), 0);
        ptr = ptr->nodes_[subtree].get();
        cur_name = cur_name.substr(idx + 1);
    }
    CHECK_GT(ptr->leaves_.count(class_name), 0);
    auto& names = ptr->leaves_[class_name];
    return names;
}

std::string ItemNameTree::retrieve_item_path(const std::string& class_name) {
    const auto& names = retrieve_item_paths(class_name);
    return names[util::get_rand_ind(names.size())];
}

ItemRepository::ItemRepository() {
    item_tree_ = std::make_shared<ItemNameTree>();
    get_all_item_names(item_tree_, img_path_);
    preload_all_item_images(item_tree_);
    preload_all_item_properties();
}

void ItemRepository::get_all_item_names(std::shared_ptr<ItemNameTree>& root,
                                        std::string dir) {
    struct dirent *ent = nullptr;
    DIR *dir_ptr = opendir(dir.c_str());
    CHECK(dir_ptr);

    /* get all the item files and directories within directory */
    while ((ent = readdir(dir_ptr))) {
        std::string name = ent->d_name;
        if (ent->d_type == DT_DIR) { // sub-directory
            // skip the current and the upper directory
            if (name == "." || name == "..") {
                continue;
            }
            std::string subdir = dir + "/" + name;
            auto& subroot = root->insert_node(name);
            get_all_item_names(subroot, subdir);
        } else { // file
            // name format: instance_id.jpg
            if (std::count(name.begin(), name.end(), '_') == 0
                || name.find(".jpg") == std::string::npos)
                continue;
            root->insert_item(dir + "/" + name);
        }
    }
    closedir(dir_ptr);
}

void ItemRepository::preload_all_item_images(std::shared_ptr<ItemNameTree> root) {
    for (const auto& leaf : root->leaves_) {
        for (const auto& path : leaf.second) {
            cv::Mat img = cv::imread(path, 1);
            CHECK(!img.empty()) << "could not open or find the image: " + path;
            cv::resize(img, img,
                       cv::Size(XItem::item_size_, XItem::item_size_),
                       cv::INTER_LINEAR);
            CHECK(item_imgs_.find(path) == item_imgs_.end());
            item_imgs_[path] = img;
        }
    }

    for (const auto& node : root->nodes_) {
        preload_all_item_images(node.second);
    }
}

void ItemRepository::preload_all_item_properties() {
    std::ifstream infile(img_path_ + "/properties.txt");
    std::string line;
    std::vector<std::string> keys;
    while (std::getline(infile, line)) {
        boost::trim(line);
        if (line == "") { // skip empty line
            continue;
        }
        CHECK_GT(line.length(), 2) << "invalid line";
        if (line.find("//[") != std::string::npos) {
            // haven't added keys yet
            CHECK_EQ(keys.size(), 0);
            // parse property keys
            int begin = line.find("[") + 1;
            std::istringstream iss(line.substr(begin));
            std::string key;
            iss >> key;
            CHECK_EQ(key, "img_name");
            while (iss >> key) {
                keys.push_back(key);
            }
        } else if (line.substr(0, 2) == "//") {
            continue;
        } else { // parse property values
            CHECK_GT(keys.size(), 0);
            std::istringstream iss(line);
            std::string name;
            std::string value;
            iss >> name;
            name = img_path_ + "/" + name;
            CHECK(item_properties_.find(name) == item_properties_.end());
            ItemProperty ip;
            ip.keys = keys;
            while (iss >> value) {
                ip.values.push_back(value);
            }
            CHECK_EQ(ip.keys.size(), ip.values.size());
            item_properties_[name] = ip;
        }
    }
    infile.close();
}

std::string ItemRepository::retrieve_property(std::string name, std::string key) {
    CHECK_GT(item_properties_.count(name), 0);
    ItemProperty ip = item_properties_[name];
    for (size_t i = 0; i < ip.keys.size(); i ++) {
        if (ip.keys[i] == key) {
            return ip.values[i];
        }
    }
    LOG(FATAL) << "unrecognized key";
    return "";
}

XItem::XItem(ItemType type, std::string name, Loc loc, std::string img_path, bool active, bool perturb) {
    this->init(type, name, loc, img_path, active, perturb);
}

XItem::~XItem() {
}

int XItem::get_item_size() {
    return XItem::item_size_;
}

void XItem::init(ItemType type, std::string name, Loc loc, std::string img_name, bool active, bool perturb) {
    this->type_ = type;
    this->name_ = name;
    this->color_ = "";
    this->loc_ = loc;
    this->img_name_ = img_name;
    this->perturb_ = perturb;
    this->active_ = active;
    if (this->perturb_) {
        double angle = util::get_rand_range_val(360.0);
        double scale = util::get_rand_range_val(0.3) + 0.7;
        double shift_x = util::get_rand_range_val((1 - scale) / 2.0);
        double shift_y = util::get_rand_range_val((1 - scale) / 2.0);
        this->perturb_transform_ = {angle, scale, shift_x, shift_y};
    } else {
        this->perturb_transform_ = {0, 1, 0, 0};
    }
    this->get_item_image();
    this->get_item_color();
}

ItemType XItem::get_item_type() {
    return this->type_;
}

std::string XItem::get_item_name() {
    return this->name_;
}

std::string XItem::get_item_color() {
    if (this->img_.empty()) {
        return "";
    }
    // not computed yet
    if (this->color_ == "") {
        this->color_ = item_rep_.retrieve_property(this->img_name_, "color");
    }
    return this->color_;
}

bool XItem::item_color_defined(std::string color) {
    return (color != "" && color != "mixed-color");
}

cv::Mat XItem::get_item_image() {
    if (this->img_.empty()) { //load the image once
        if (!this->img_name_.empty()) {
            CHECK(item_rep_.item_imgs_.find(this->img_name_)
                  != item_rep_.item_imgs_.end()) << this->img_name_;
            cv::Mat img = item_rep_.item_imgs_[this->img_name_].clone();

            if (this->perturb_) {
                cv::Point2f src_center(img.cols / 2.0F, img.rows / 2.0F);
                double angle = this->perturb_transform_.angle;
                double scale = this->perturb_transform_.scale;
                cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, angle, scale);
                cv::warpAffine(img, img, rot_mat, img.size(),
                               cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
                double shift_x = this->perturb_transform_.x * img.cols;
                double shift_y = this->perturb_transform_.y * img.rows;
                cv::Mat shift_mat = (cv::Mat_<double>(2, 3) << 1, 0, shift_x, 0, 1, shift_y);
                cv::warpAffine(img, img, shift_mat, img.size(),
                               cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
            }
            this->img_ = img;
        }
    }
    return this->img_;
}

Loc XItem::get_item_location() {
    return this->loc_;
}

void XItem::set_item_location(int x, int y) {
    this->loc_.x = x;
    this->loc_.y = y;
}

void XItem::set_item_type(ItemType item_type){
    this->type_ = item_type;
}

bool XItem::is_reachable() {
    bool is_reachable_flag = this->get_item_type() != BLOCK;
    return is_reachable_flag;
}

}} //namespace simulator::xwd
