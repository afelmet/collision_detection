#include "collision_detection/CollisionFactory.hpp"

namespace collision_detection
{
CollisionFactory::CollisionFactory() {}

CollisionFactory::~CollisionFactory() {}

[[nodiscard]] CollisionPtr
CollisionFactory::getCollisionDetector(CollisionDetectionConfig collision_detection_config)
{
    switch (collision_detection_config.collision_library)
    {
    case collision_detection::FCL:
    {
        return std::make_shared<FCLCollisionDetection>(collision_detection_config);
    }
    default:
    {
        std::cout << "Unknown collision library ";
        std::cout << collision_detection_config.collision_library;
        std::cout << " selected." << std::endl;
        return CollisionPtr{};
    }
    }
}
} // namespace collision_detection
