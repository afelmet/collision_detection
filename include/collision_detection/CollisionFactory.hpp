#ifndef COLLISIONFACTORY_HPP_
#define COLLISIONFACTORY_HPP_

#include <string>
#include <variant>
#include <vector>

#include "collision_detection/BaseCollisionDetection.hpp"
#include "collision_detection/CollisionConfig.hpp"
#include "fcl_wrapper/FCLCollisionDetection.hpp"

namespace collision_detection
{
typedef std::variant<std::monostate, std::shared_ptr<FCLCollisionDetection>> CollisionPtr;

class CollisionFactory
{
  public:
    CollisionFactory();
    ~CollisionFactory();

    CollisionPtr getCollisionDetector(CollisionDetectionConfig collision_detection_config);
};
}; // namespace collision_detection

#endif
