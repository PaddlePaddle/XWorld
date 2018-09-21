#ifndef INVENTORY_H_
#define INVENTORY_H_

#include <unordered_map>
#include <vector>
#include <algorithm>

#include "glm/glm.hpp"


namespace xrobot {
class Inventory {
public:
	Inventory(const int size);
	~Inventory();

	void PutObject(std::string& object_label, std::string& object_path);
	std::string GetObjectRandomly(std::string& object_label);
	std::string GetObject(const std::string& object_label);
	void DiscardObject(const std::string& object_label, const int num = 1);
	void ClearInventory();
	void PrintInventory();

	int GetNumUsedSpace() { return size_ - rest_; }
	int GetNumFreeSpace() { return rest_; }

	bool IsPickableObject(const std::string& name);
	void AddNonPickableObjectTag(const std::string& name);
	void ResetNonPickableObjectTag();

private:
	std::unordered_map<std::string, std::vector<std::string>> inventory_;
	std::vector<std::string> non_pickable_list_;
	int size_;
	int rest_;
};
}

#endif // INVENTORY_H_


