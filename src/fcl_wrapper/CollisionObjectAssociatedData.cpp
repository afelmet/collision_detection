#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"

namespace collision_detection
{

CollisionObjectAssociatedData::CollisionObjectAssociatedData() {}

[[nodiscard]] std::string &CollisionObjectAssociatedData::getID() { return this->id; }

void CollisionObjectAssociatedData::setID(const std::string &id_param) { this->id = id_param; }

} // namespace collision_detection
