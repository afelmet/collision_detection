#include <iostream>

#include <base-types/samples/RigidBodyState.hpp>
#include <collision_detection/CollisionFactory.hpp>

int main()
{
    collision_detection::CollisionFactory collision_factory;

    // create collision detection config
    collision_detection::CollisionDetectionConfig coll_config;
    coll_config.collision_library = collision_detection::FCL;
    coll_config.collision_info_type = collision_detection::DISTANCE;
    coll_config.calculate_distance_information = true;
    coll_config.stop_after_first_collision = true;
    coll_config.max_num_collision_contacts = 1;

    // create collision detectors
    collision_detection::CollisionPtr world_collision_detector =
        collision_factory.getCollisionDetector(coll_config);
    collision_detection::CollisionPtr robot_collision_detector =
        collision_factory.getCollisionDetector(coll_config);

    // link collision detectors
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->assignWorldDetector(std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(
            world_collision_detector));

    /*
     *                        CREATE OBJECTS FOR TESTING COLLISION DETECTION
     */
    // BEAM A                                                dx,  dy,  dz,                  name,
    // pose,   link_padding
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->registerBoxToCollisionManager(0.2, 0.2, 0.2, std::string("beamA"), base::Pose(), 1.0);

    // BEAM B                                                dx,  dy,  dz,                  name, x
    // y     z                             pose,   link_padding
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->registerBoxToCollisionManager(
            0.2, 0.2, 0.2, std::string("beamB"),
            base::Pose(base::Position(10.1, 10.1, 0.1), base::Orientation::Identity()), 1.0);

    /*
     *                           TEST 1: NO COLLISION BETWEEN BEAMS A AND B
     */
    std::visit(
        [](auto detector)
        {
            using T = std::decay_t<decltype(detector)>;
            if constexpr (std::is_same_v<
                              T, std::shared_ptr<collision_detection::FCLCollisionDetection>>)
            {
                double collision_cost = 0.0;
                if (detector->isCollisionsOccured(collision_cost))
                {
                    auto collision_objects_names = detector->getCollidedObjectsNames();
                    for (const std::pair<std::string, std::string> &name : collision_objects_names)
                        std::cout << "[\033[0;31m FAILURE\033[0m ] Unexpected collision detected "
                                     "between objects "
                                  << name.first << " and " << name.second << "." << std::endl;
                }
                else
                {
                    std::cout << "[\033[0;32m SUCCESS\033[0m ] No unexpected collisions detected."
                              << std::endl;
                }
            }
        },
        robot_collision_detector);

    /*
     *                           TEST 2: COLLISION BETWEEN BEAMS A AND C
     */
    // BEAM C                                                dx,  dy,  dz,                 name, x
    // y     z                             pose,   link_padding
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->registerBoxToCollisionManager(
            0.2, 0.2, 0.2, std::string("beamC"),
            base::Pose(base::Position(0.1, 0.1, 0.1), base::Orientation::Identity()), 1.0);

    std::visit(
        [](auto detector)
        {
            using T = std::decay_t<decltype(detector)>;
            if constexpr (std::is_same_v<
                              T, std::shared_ptr<collision_detection::FCLCollisionDetection>>)
            {
                double collision_cost = 0.0;
                if (detector->isCollisionsOccured(collision_cost))
                {
                    auto collision_objects_name = detector->getCollidedObjectsNames();
                    for (const std::pair<std::string, std::string> &name : collision_objects_name)
                        std::cout << "[\033[0;32m SUCCESS\033[0m ] Expected collision between "
                                  << name.first << " and " << name.second << " detected."
                                  << std::endl;

                    std::cout << "[\033[0;32m SUCCESS\033[0m ] " << collision_objects_name.size()
                              << std::endl;
                }
                else
                {
                    std::cout << "[\033[0;31m FAILURE\033[0m ] Collision between beams A and C not "
                                 "detected."
                              << std::endl;
                }
            }
        },
        robot_collision_detector);

    /*
     *                           TEST 3: COLLISION BETWEEN BEAMS A AND SPHERE A
     */
    // SPHERE C
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->removeSelfCollisionObject("beamC");

    base::Pose pose;
    std::get<std::shared_ptr<collision_detection::FCLCollisionDetection>>(robot_collision_detector)
        ->registerSphereToCollisionManager(0.2, std::string("sphereA"), pose, 1.0);

    std::visit(
        [](auto detector)
        {
            using T = std::decay_t<decltype(detector)>;
            if constexpr (std::is_same_v<
                              T, std::shared_ptr<collision_detection::FCLCollisionDetection>>)
            {
                double collision_cost = 0.0;
                if (detector->isCollisionsOccured(collision_cost))
                {
                    auto collision_objects_name = detector->getCollidedObjectsNames();
                    for (const std::pair<std::string, std::string> &name : collision_objects_name)
                        std::cout << "[\033[0;32m SUCCESS\033[0m ] Expected collision between "
                                  << name.first << " and " << name.second << " detected."
                                  << std::endl;
                }
                else
                {
                    std::cout << "[\033[0;31m FAILURE\033[0m ] Collision between beam A and sphere "
                                 "A not detected."
                              << std::endl;
                }
            }
        },
        robot_collision_detector);

    return 0;
}
