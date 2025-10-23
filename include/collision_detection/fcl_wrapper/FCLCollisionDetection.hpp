#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/config.h>
#include <fcl/fcl.h>
#include <sys/stat.h>

#include <assimp/Importer.hpp>
#include <functional>
#include <memory>
#include <ranges>
#include <string>
#include <vector>

#include "collision_detection/BaseCollisionDetection.hpp"
#include "collision_detection/CollisionConfig.hpp"
// #include "collision_detection/abstract/AbstractCollisionDetection.hpp"
#include "collision_detection/fcl_wrapper/CollisionData.hpp"
#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"

namespace collision_detection
{
typedef std::pair<std::string, std::shared_ptr<fcl::CollisionObject<double>>> CollisionObjectPair;

typedef std::multimap<std::string, std::shared_ptr<fcl::CollisionObject<double>>>
    CollisionObjectsMap;

class FCLCollisionDetection : public BaseCollisionDetection<FCLCollisionDetection>
{
  private:
    Eigen::Vector3d scale_mesh_;
    CollisionObjectsMap collision_objects_container_;
    CollisionDetectionConfig collision_detection_config_;

    std::vector<std::pair<std::string, std::string>> collision_object_names_;
    std::vector<CollisionObjectAssociatedData *> collision_data_;
    std::vector<DistanceInformation> env_collision_distance_information_;
    std::vector<DistanceInformation> full_collision_distance_information_;
    std::vector<fcl::CollisionObject<double> *> fcl_octomap_boxes_;

    std::shared_ptr<octomap::OcTree> octomap_ptr_;
    std::shared_ptr<BaseCollisionDetection<FCLCollisionDetection>> world_collision_detector_;
    std::shared_ptr<fcl::CollisionObject<double>> fcl_tree_collision_object_ptr_;

    DistanceData self_distance_data;
    DistanceData env_distance_data;

  public:
    int num_octree_;
    std::shared_ptr<fcl::BroadPhaseCollisionManager<double>> broad_phase_collision_manager;

  private:
    void registerCollisionObjectToCollisionManager(
        const std::string &link_name,
        std::shared_ptr<fcl::CollisionObject<double>> &collision_object);

    double getCollisionCost(CollisionData &collision_data,
                            std::vector<DistanceInformation> &contacts);

    DistanceData getDistanceData();

    CollisionData getCollisionData();

    void calculateCompleteDistanceInfo();

    void calculateOnlyEnvironmentDistanceInfo();

  public:
    FCLCollisionDetection(CollisionDetectionConfig collision_detection_config);

    ~FCLCollisionDetection();

    std::size_t numberOfObjectsInCollisionManager();

    void getCollisionManager(
        std::shared_ptr<fcl::BroadPhaseCollisionManager<double>> &collision_manager);

    bool extractTrianglesAndVerticesFromMesh(const std::string &abs_path_to_mesh_file,
                                             std::vector<fcl::Triangle> &triangles,
                                             std::vector<fcl::Vector3d> &vertices,
                                             double scale_for_mesha_files_x,
                                             double scale_for_mesha_files_y,
                                             double scale_for_mesha_files_z);

    void registerOctreeToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap,
                                          const base::Pose &collision_object_pose,
                                          std::string link_name);

    // The parameter 'collision_object_pose' is never used in the function body.
    void registerOctreeAsBoxesToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap,
                                                 // const base::Pose &collision_object_pose,
                                                 std::string link_name);

    void registerBoxToCollisionManager(const double &box_x, const double &box_y,
                                       const double &box_z, const std::string &link_name,
                                       const base::Pose &collision_object_pose,
                                       const double &link_padding = 1.00);

    bool registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file,
                                        const Eigen::Vector3d &mesh_scale,
                                        const std::string &link_name,
                                        const base::Pose &collision_object_pose,
                                        const double &link_padding = 1.00);

    void registerMeshToCollisionManager(const std::string &link_name,
                                        const base::Pose &collision_object_pose,
                                        const std::vector<fcl::Triangle> &triangles,
                                        const std::vector<fcl::Vector3d> &vertices);

    void registerCylinderToCollisionManager(const double &radius, const double &length,
                                            const std::string &link_name,
                                            const base::Pose &collision_object_pose,
                                            const double &link_padding = 1.00);

    void registerSphereToCollisionManager(const double &radius, const std::string &link_name,
                                          const base::Pose &collision_object_pose,
                                          const double &link_padding = 1.00);

    void updateCollisionObjectTransform(std::string link_name,
                                        const base::Pose collision_object_pose);

    void updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap,
                           const std::string &env_object_name);

    void updateOctomapBoxesEnvironment(const std::shared_ptr<octomap::OcTree> &octomap,
                                       const std::string &env_object_name);

    bool isCollisionsOccured(double &total_cost);

    // NOTE: Not implemented anywhere.
    //
    // bool checkEnvironmentCollision(
    //     const shared_ptr<fcl::BroadPhaseCollisionManager<double>>
    //     &external_broad_phase_collision_manager, int num_max_contacts=1
    // );

    bool assignWorldDetector(
        std::shared_ptr<BaseCollisionDetection<FCLCollisionDetection>> collision_detector);

    bool removeSelfCollisionObject(const std::string &collision_object_name);

    bool removeWorldCollisionObject(const std::string &collision_object_name);

    void removeObject4mCollisionContainer(const std::string &collision_object_name);

    bool removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size);

    bool removeOctomapBoxes(const std::string &collision_object_name);

    std::shared_ptr<fcl::BroadPhaseCollisionManager<double>> &getCollisionManager();

    bool distanceOfClosestObstacleToRobot(std::shared_ptr<fcl::BroadPhaseCollisionManager<double>>
                                              &external_broad_phase_collision_manager,
                                          DistanceData &distance_data);

    void printCollisionObject();

    void saveOctree();

    std::ranges::subrange<std::vector<std::pair<std::string, std::string>>::iterator>
    getCollidedObjectsNames();

    auto getRobotCollisionObjectsNames();

    auto getWorldCollisionObjectsNames();

    auto getCollisionDistanceInformation();

    auto getCompleteDistanceInformation();

    auto getOnlyEnvironmentDistanceInformation();
};
} // namespace collision_detection

#endif
