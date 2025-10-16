#ifndef BASECOLLISIONDETECTION_HPP_
#define BASECOLLISIONDETECTION_HPP_

#include <octomap/octomap.h>
#include <srdfdom/model.h>

#include <base-types/samples/RigidBodyState.hpp>
#include <memory>
#include <string>
#include <vector>

// Fixes deprecated usage of global boost placeholders.
#include <base-logging/logging/Logging.hpp>
#include <boost/bind/bind.hpp>

/*
 *  CRTP Base class providing a common interface for all collision detection
 *  implementations.
 */
namespace collision_detection
{

struct DistanceInformation
{
    DistanceInformation() { nearest_points.resize(2); }

    std::string object1;
    std::string object2;
    double min_distance;
    std::vector<Eigen::Vector3d> nearest_points;
    Eigen::Vector3d contact_normal;
};

template <typename Derived> class BaseCollisionDetection
{
  public:
    std::string remove_link_;
    static std::vector<srdf::Model::CollisionPair> disabled_collisions_;

  private:
    friend Derived;
    BaseCollisionDetection() = default;

  public:
    void registerOctreeToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap,
                                          const base::Pose &collision_object_pose,
                                          std::string link_name)
    {
        static_cast<Derived *>(this)->registerOctreeToCollisionManager(
            octomap, collision_object_pose, link_name);
    }

    // Parameter 'collision_object_pose' is never used in the function body.
    void registerOctreeAsBoxesToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap,
                                                 // const base::Pose &collision_object_pose,
                                                 std::string link_name)
    {
        static_cast<Derived *>(this)->registerOctreeAsBoxesToCollisionManager(
            octomap,
            // collision_object_pose,
            link_name);
    }

    bool registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file,
                                        const Eigen::Vector3d &mesh_scale,
                                        const std::string &link_name,
                                        const base::Pose &collision_object_pose,
                                        const double &link_padding)
    {
        return static_cast<Derived *>(this)->registerMeshToCollisionManager(
            abs_path_to_mesh_file, mesh_scale, link_name, collision_object_pose, link_padding);
    }

    void registerBoxToCollisionManager(const double &box_x, const double &box_y,
                                       const double &box_z, const std::string &link_name,
                                       const base::Pose &collision_object_pose,
                                       const double &link_padding)
    {
        static_cast<Derived *>(this)->registerBoxToCollisionManager(
            box_x, box_y, box_z, link_name, collision_object_pose, link_padding);
    }

    void registerCylinderToCollisionManager(const double &radius, const double &length,
                                            const std::string &link_name,
                                            const base::Pose &collision_object_pose,
                                            const double &link_padding)
    {
        static_cast<Derived *>(this)->registerCylinderToCollisionManager(
            radius, length, link_name, collision_object_pose, link_padding);
    }

    void registerSphereToCollisionManager(const double &radius, const std::string &link_name,
                                          const base::Pose &collision_object_pose,
                                          const double &link_padding)
    {
        static_cast<Derived *>(this)->registerSphereToCollisionManager(
            radius, link_name, collision_object_pose, link_padding);
    }

    void updateCollisionObjectTransform(std::string link_name,
                                        const base::Pose collision_object_pose)
    {
        static_cast<Derived *>(this)->updateCollisionObjectTransform(link_name,
                                                                     collision_object_pose);
    }

    void updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap,
                           const std::string &env_object_name)
    {
        static_cast<Derived *>(this)->updateEnvironment(octomap, env_object_name);
    }

    void updateOctomapBoxesEnvironment(const std::shared_ptr<octomap::OcTree> &octomap,
                                       const std::string &env_object_name)
    {
        static_cast<Derived *>(this)->updateOctomapBoxesEnvironment(octomap, env_object_name);
    }

    bool removeSelfCollisionObject(const std::string &collision_object_name)
    {
        return static_cast<Derived *>(this)->removeSelfCollisionObject(collision_object_name);
    }

    bool removeWorldCollisionObject(const std::string &collision_object_name)
    {
        return static_cast<Derived *>(this)->removeWorldCollisionObject(collision_object_name);
    }

    bool removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size)
    {
        return static_cast<Derived *>(this)->removeObjectFromOctree(object_pose, object_size);
    }

    std::size_t numberOfObjectsInCollisionManager()
    {
        return static_cast<Derived *>(this)->numberOfObjectsInCollisionManager();
    }

    bool isCollisionsOccured(double &collision_cost)
    {
        return static_cast<Derived *>(this)->isCollisionOccured(collision_cost);
    }

    bool assignWorldDetector(std::shared_ptr<BaseCollisionDetection<Derived>> collision_detector)
    {
        return static_cast<Derived *>(this)->assignWorldDetector(collision_detector);
    }

    void printCollisionObject() { static_cast<Derived *>(this)->printCollisionObject(); }

    void saveOctree() { static_cast<Derived *>(this)->saveOctree(); }

    std::vector<std::pair<std::string, std::string>> getCollidedObjectsNames()
    {
        return static_cast<Derived *>(this)->getCollidedObjectsNames();
    }

    static bool linksToBeChecked(const std::string &first_link_name,
                                 const std::string &second_link_name)
    {
        if (first_link_name == second_link_name)
            return false;

        for (const srdf::Model::CollisionPair &dc : BaseCollisionDetection::disabled_collisions_)
        {
            if ((first_link_name == dc.link1_ && second_link_name == dc.link2_) ||
                (second_link_name == dc.link1_ && first_link_name == dc.link2_))
                return false;
        }
        return true;
    }

    void setDisabledCollisionPairs(std::vector<srdf::Model::CollisionPair> &disabled_collisions)
    {
        static_cast<Derived *>(this)->setDisabledCollisionPairs(disabled_collisions);
    }

    void addDisabledCollisionPairs(srdf::Model::CollisionPair &disabled_collision)
    {
        static_cast<Derived *>(this)->addDisabledCollisionPairs(disabled_collision);
    }

    void removeDisabledCollisionLink(const std::string &link)
    {
        static_cast<Derived *>(this)->removeDisabledCollisionLink(link);
    }

    bool isLinkListed(srdf::Model::CollisionPair const &remove_link)
    {
        return static_cast<Derived *>(this)->isLinkedList(remove_link);
    }

    auto getCollisionDistanceInformation()
    {
        return static_cast<Derived *>(this)->getCollisionDistanceInformation();
    }

    auto getCompleteDistanceInformation()
    {
        return static_cast<Derived *>(this)->getCompleteDistanceInformation();
    }

    auto getOnlyEnvironmentDistanceInformation()
    {
        return static_cast<Derived *>(this)->getOnlyEnvironmentDistanceInformation();
    }

    auto getRobotCollisionObjectsNames()
    {
        return static_cast<Derived *>(this)->getRobotCollisionObjectsNames();
    }

    auto getWorldCollisionObjectsNames()
    {
        return static_cast<Derived *>(this)->getWorldCollisionObjectsNames();
    }

    /*
     *  NOTE: Allows to access derived class attributes and methods.
     */
    Derived *cast() { return static_cast<Derived *>(this); }
};

template <class Derived>
std::vector<srdf::Model::CollisionPair> BaseCollisionDetection<Derived>::disabled_collisions_ = {};
} // namespace collision_detection

#endif
