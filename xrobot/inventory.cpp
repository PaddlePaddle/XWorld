#include "inventory.h"

namespace xrobot {
	Inventory::Inventory(World* world, const int size) : world_(world),
														 inventory_(),
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

	void Inventory::PutObject(Robot* r) {
		assert(r);

		if(rest_ < 1) {
			printf("Inventory Is Full!\n");
			return;
		}

		if(r->label_ == "" || r->label_ == "unlabeled") {
			printf("Please Assigned A Label For This Object!\n");
			return;
		}

		if (inventory_.find(r->label_) != inventory_.end() && 
            !inventory_[r->label_].empty()) {
			inventory_[r->label_].push_back(r->path_);
		} else {
			std::vector<std::string> object_list_tmp;
			object_list_tmp.push_back(r->path_);
			inventory_[r->label_] = object_list_tmp;
		}

		rest_--;
	}

	std::string Inventory::GetObjectRandomly(std::string& object_label) {
		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return "";
		}

		for (auto it = inventory_.begin(); it != inventory_.end(); it++) {
			if(it->second.size() > 0) {
				std::string object_tmp = it->second.back();
				it->second.pop_back();
				rest_++;
				object_label = it->first;
				return object_tmp;
			}
		}
	}

	std::string Inventory::GetObject(const std::string& object_label) {
		assert(object_label.size());

		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return "";
		}

		if(inventory_.find(object_label) != inventory_.end() && 
            !inventory_[object_label].empty()) {
			std::string object_tmp = inventory_[object_label].back();
			inventory_[object_label].pop_back();
			rest_++;
			return object_tmp;
		} else {
			printf("Cannot Find In Inventory!\n");
			return "";
		}
	}

	void Inventory::DiscardObject(const std::string& object_label,
								  const int num) {
		assert(object_label.size());
		assert(num > 0);

		if(size_ == rest_) {
			printf("Inventory Is Empty!\n");
			return;
		}

		if(inventory_.find(object_label) != inventory_.end() && 
            !inventory_[object_label].empty()) {

			int size = inventory_[object_label].size();
			if(num >= size) {
				inventory_[object_label].clear();
				rest_ += size;
			} else {
				inventory_[object_label].erase(
					inventory_[object_label].begin() + num);
				rest_ += num;
			}

		} else {
			printf("Cannot Find In Inventory!\n");
			return;
		}
	}

	void Inventory::ClearInventory() {
		inventory_.clear();
		rest_ = size_;
	}

	void Inventory::PrintInventory() {
		printf("---------------------------------------\n");
		for (auto it = inventory_.begin(); it != inventory_.end(); it++) {
			printf("%s : %d\n", it->first.c_str(), (int) it->second.size());
		}
		printf("---------------------------------------\n");
	}
}