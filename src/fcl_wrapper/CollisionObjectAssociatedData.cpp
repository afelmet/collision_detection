#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"

namespace collision_detection
{

CollisionObjectAssociatedData::CollisionObjectAssociatedData()
{
}


std::string& CollisionObjectAssociatedData::getID()
{
    return this->id;
}

void CollisionObjectAssociatedData::setID(const std::string &id_param)
{
    this->id=id_param;
}

}// end namespace trajectory_optimization
