#ifndef INVENTORY_H_
#define INVENTORY_H_

#include <unordered_map>
#include <vector>
#include <algorithm>
#include <memory>

#include "glm/glm.hpp"
#include "bullet/LinearMath/btTransform.h"

namespace xrobot {

class RobotBase;

class Inventory {
public:
	Inventory(const int size);
	~Inventory();

	bool PutObject(std::weak_ptr<RobotBase> put_object);
	std::weak_ptr<RobotBase> GetObjectLast();
	bool GetObject(std::weak_ptr<RobotBase> get_object, const std::string& label);
	bool DiscardObject(const std::string& object_label);
	void ClearInventory();
	void PrintInventory();

	int GetNumUsedSpace() { return size_ - rest_; }
	int GetNumFreeSpace() { return rest_; }

	bool IsPickableObject(const std::string& name);
	void AddNonPickableObjectTag(const std::string& name);
	void ResetNonPickableObjectTag();

private:
	std::vector<std::weak_ptr<RobotBase>> inventory_;
	std::vector<std::string> non_pickable_list_;
	int size_;
	int rest_;
};
}

#endif // INVENTORY_H_


