#include "inventory.h"
#include "world.h"
namespace xrobot {
	Inventory::Inventory(const int size) : inventory_(),
										   non_pickable_list_(0),
										   size_(size),
										   rest_(size) {
		if(size < 1) {
			size_ = INT_MAX;
			rest_ = INT_MAX;
		}
	}

	Inventory::~Inventory() {
		ClearInventory();
		non_pickable_list_.clear();
	}


	bool Inventory::IsPickableObject(const std::string& name) {
		return std::find(non_pickable_list_.begin(),
					 	 non_pickable_list_.end(),
						 name) == non_pickable_list_.end();
	}

	void Inventory::AddNonPickableObjectTag(const std::string& name) {
		assert(!name.empty());
		non_pickable_list_.push_back(name);
	}

	void Inventory::ResetNonPickableObjectTag() {
		non_pickable_list_.clear();
	}

	bool Inventory::PutObject(RobotBase* object) {
		if(rest_ < 1) {
			printf("Inventory Is Full!\n");
			return false;
		}

		if(!object) {
			printf("Invalid Object!\n");
			return false;
		}

		if(object->robot_data_.label_ == "" ||
		   object->robot_data_.label_ == "unlabeled") {
			printf("Please Assigned A Label For This Object!\n");
			return false;
		}


		object->hide(true);
		btTransform transform = object->robot_data_.root_part_->object_position_;
	    transform.setOrigin(btVector3(0, -5, 0));

	    // Set Velocity to 0
	    object->bullet_world_->SetTransformation(object, transform);
	    object->bullet_world_->SetVelocity(object, btVector3(0,0,0));

	    // Change Root to Static
	    float mass;
	    object->robot_data_.root_part_->GetMass(mass);
	    object->robot_data_.root_part_->ChangeMass(0.0f);
	    object->robot_data_.root_part_->SetMassOriginal(mass);
	    object->robot_data_.root_part_->Sleep();

	     // Change Rest to Static
	    for (Object * part : object->robot_data_.other_parts_)
	    {
	        part->GetMass(mass);
	        part->ChangeMass(0.0f);
	        part->SetMassOriginal(mass);
	        part->Sleep();
	    }

		inventory_.push_back(object);

		rest_--;
		return true;
	}

	RobotBase* Inventory::GetObjectLast() {
		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return nullptr;
		}

		rest_++;
	
		RobotBase* object = inventory_.back();
		inventory_.pop_back();

		if(object) {
	        float mass;
	        object->robot_data_.root_part_->ChangeMass(
	                object->robot_data_.root_part_->GetMassOriginal());
	        object->robot_data_.root_part_->Wake();
	        object->hide(false);

	        for (Joint * joint : object->robot_data_.joints_list_) {
	            if(joint) {
	                joint->ResetJointState(0.0f, 0.005f);
	            }
	        }

	        for (Object * part : object->robot_data_.other_parts_) {
	            part->ChangeMass(part->GetMassOriginal());
	        }
	    }

		return object;
	}

	bool Inventory::GetObject(RobotBase* object, const std::string& label) {
		assert(label.size());

		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return false;
		}

		for (int i = 0; i < inventory_.size(); ++i)
		{
			if(inventory_[i]->robot_data_.label_ == label) {
				object = inventory_[i];
				rest_++;
				inventory_.erase(inventory_.begin() + i);
				return true;
			}
		}

		return false;
	}

	bool Inventory::DiscardObject(const std::string& object_label) {
		assert(object_label.size());

		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return false;
		}

		for (int i = 0; i < inventory_.size(); ++i)
		{
			if(inventory_[i]->robot_data_.label_ == object_label) {
				rest_++;
				inventory_.erase(inventory_.begin() + i);
				return true;
			}
		}

		return false;
	}

	void Inventory::ClearInventory() {
		inventory_.clear();
		rest_ = size_;
	}

	void Inventory::PrintInventory() {
		printf("---------------------------------------\n");
		for (int i = 0; i < inventory_.size(); ++i)
		{
			printf("%s\n", inventory_[i]->robot_data_.label_.c_str());
		}
		printf("---------------------------------------\n");
	}
}