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

	bool Inventory::PutObject(std::weak_ptr<RobotBase> put_object) {
		if(rest_ < 1) {
			printf("Inventory Is Full!\n");
			return false;
		}

		if(auto object = put_object.lock())
		{
			auto bullet_world = object->bullet_world_.lock();

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
		    bullet_world->SetTransformation(object, transform);
		    bullet_world->SetVelocity(object, btVector3(0,0,0));

		    // Change Root to Static
		    float mass;
		    object->robot_data_.root_part_->GetMass(mass);
		    object->robot_data_.root_part_->ChangeMass(0.0f);
		    object->robot_data_.root_part_->SetMassOriginal(mass);
		    object->robot_data_.root_part_->Sleep();

		     // Change Rest to Static
		    for (auto part : object->robot_data_.other_parts_)
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

		return false;
	}

	std::vector<std::string> Inventory::GetObjectTagInInventory() {
		std::vector<std::string> ret;

		for (int i = 0; i < inventory_.size(); ++i)
		{
			if(auto object = inventory_[i].lock()) 
			{
				ret.push_back(object->robot_data_.label_);
			}
		}

		return ret;
	}

	std::weak_ptr<RobotBase> Inventory::GetObjectLast() {
		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return std::weak_ptr<RobotBase>();
		}

		rest_++;
	
		std::weak_ptr<RobotBase> get_object = inventory_.back();
		inventory_.pop_back();

		if(auto object = get_object.lock()) {
	        float mass;
	        object->robot_data_.root_part_->ChangeMass(
	                object->robot_data_.root_part_->GetMassOriginal());
	        object->robot_data_.root_part_->Wake();
	        object->hide(false);

	        for (auto joint : object->robot_data_.joints_list_) {
	            if(joint) {
	                joint->ResetJointState(0.0f, 0.005f);
	            }
	        }

	        for (auto part : object->robot_data_.other_parts_) {
	            part->ChangeMass(part->GetMassOriginal());
	        }

	        return object;
	    }

	    return std::weak_ptr<RobotBase>();
	}

	bool Inventory::GetObject(std::weak_ptr<RobotBase> get_object, const std::string& label) {
		assert(label.size());

		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return false;
		}

		for (int i = 0; i < inventory_.size(); ++i)
		{

			if(auto object = inventory_[i].lock()) 
			{
				if(object->robot_data_.label_ == label) {
					get_object = object;
					rest_++;
					inventory_.erase(inventory_.begin() + i);
					return true;
				}
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

			if(auto object = inventory_[i].lock()) 
			{
				if(object->robot_data_.label_ == object_label) {
					rest_++;
					object = nullptr;
					inventory_.erase(inventory_.begin() + i);
					return true;
				}
			}
		}

		return false;
	}

	void Inventory::ClearInventory() {

		for (int i = 0; i < inventory_.size(); ++i)
		{
			if(auto object = inventory_[i].lock()) 
			{
				object->hide(false);
				object->RemoveRobotTemp();
			}
		}

		inventory_.clear();
		rest_ = size_;
	}

	void Inventory::PrintInventory() {
		printf("---------------------------------------\n");
		for (int i = 0; i < inventory_.size(); ++i)
		{
			if(auto object = inventory_[i].lock()) 
			{
				printf("%s\n", object->robot_data_.label_.c_str());
			}
		}
		printf("---------------------------------------\n");
	}
}