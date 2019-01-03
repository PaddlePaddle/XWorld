#ifndef INVENTORY_H_
#define INVENTORY_H_

#include <unordered_map>
#include <vector>
#include <algorithm>
#include <memory>

namespace xrobot {

class RobotBase;

class Inventory {
    using RobotBaseWPtr = std::weak_ptr<RobotBase>;
public:
	Inventory(const int size);

	~Inventory();

	bool PutObject(const RobotBaseWPtr& put_object);

	RobotBaseWPtr GetObjectLast();

	std::vector<std::string> GetObjectTagInInventory();

	bool DiscardObject(const std::string& object_label);

	void ClearInventory();
	void PrintInventory();

	int GetNumUsedSpace() { return size_ - rest_; }
	int GetNumFreeSpace() { return rest_; }

private:
	std::vector<RobotBaseWPtr> inventory_;
	int size_;
	int rest_;
};
}

#endif // INVENTORY_H_


