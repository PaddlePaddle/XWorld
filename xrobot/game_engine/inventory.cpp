#include "inventory.h"
#include "world.h"

namespace xrobot {

Inventory::Inventory(const int size) : inventory_(),
                                       size_(size),
                                       rest_(size) {
    if(size < 1) {
        size_ = INT_MAX;
        rest_ = INT_MAX;
    }
}

Inventory::~Inventory() {
    ClearInventory();
}

bool Inventory::PutObject(const RobotBaseWPtr& put_object) {
    if(rest_ < 1) {
        printf("Inventory Is Full!\n");
        return false;
    }
    auto object = put_object.lock();
    if (object) {
        auto bullet_world = object->bullet_world_.lock();

        if(object->body_data_.label == "" ||
           object->body_data_.label == "unlabeled") {
            printf("Please Assigned A Label For This Object!\n");
            return false;
        }

        object->hide(true); 

        inventory_.push_back(object);
        rest_--;

        return true;
    } else {
        printf("Invalid object\n");
    }

    return false;
}

std::vector<std::string> Inventory::GetObjectTagInInventory() {
    std::vector<std::string> ret;

    for (int i = 0; i < inventory_.size(); ++i) {
        if(auto object = inventory_[i].lock()) {
            ret.push_back(object->body_data_.label);
        }
    }

    return ret;
}

RobotBaseWPtr Inventory::GetObjectLast() {
    if(size_ == rest_) {
        printf("Inventory Is Empty!\n");
        return RobotBaseWPtr();
    }

    rest_++;

    RobotBaseWPtr get_object = inventory_.back();
    inventory_.pop_back();

    if(auto object = get_object.lock()) {
        object->hide(false);
        return object;
    }

    return RobotBaseWPtr();
}

bool Inventory::DiscardObject(const std::string& object_label) {
    assert(object_label.size());

    if(size_ == rest_) {
        printf("Inventory Is Empty!\n");
        return false;
    }

    for (int i = 0; i < inventory_.size(); ++i) {
        if(auto object = inventory_[i].lock()) {
            if(object->body_data_.label == object_label) {
                object.reset();
                inventory_.erase(inventory_.begin() + i);
                rest_++;
                return true;
            }
        }
    }

    return false;
}

void Inventory::ClearInventory() {

    for (int i = 0; i < inventory_.size(); ++i) {
        if(auto object = inventory_[i].lock()) {
            object->hide(false);
            object->recycle();
        }
    }

    inventory_.clear();
    rest_ = size_;
}

void Inventory::PrintInventory() {
    printf("---------------------------------------\n");
    for (int i = 0; i < inventory_.size(); ++i) {
        if(auto object = inventory_[i].lock()) {
            printf("%s\n", object->body_data_.label.c_str());
        }
    }
    printf("---------------------------------------\n");
}

} // xrobot
