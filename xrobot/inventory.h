#ifndef INVENTORY_H_
#define INVENTORY_H_

#include <unordered_map>
#include <vector>

#include "glm/glm.hpp"

#include "world.h"

namespace xrobot {
class Inventory {
public:
	Inventory(World* world, const int size);
	~Inventory();

	void PutObject(Robot* r);
	std::string GetObjectRandomly(std::string& object_label);
	std::string GetObject(const std::string& object_label);
	void DiscardObject(const std::string& object_label, const int num = 1);
	void ClearInventory();
	void PrintInventory();

	int GetNumUsedSpace() { return size_ - rest_; }
	int GetNumFreeSpace() { return rest_; }

private:
	World * world_;
	std::unordered_map<std::string, std::vector<std::string>> inventory_;
	int size_;
	int rest_;
};
}

#endif // INVENTORY_H_


