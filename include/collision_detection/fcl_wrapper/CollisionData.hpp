#ifndef COLLISIONDATA_HPP
#define COLLISIONDATA_HPP

#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"
#include <fcl/fcl.h>

#include <unordered_map>

namespace collision_detection
{


// struct CollisionInformation
// {
//     std::pair<std::string,std::string>  collision_pair;
//     std::vector <std::pair<std::string,std::string> > collision_pair_names;
// };

/*
 *      HELPER STRUCTS FOR USING STD::PAIR IN THE UNORDERED MAP
 */
struct LinkPairHash {
    std::size_t operator()(const std::pair<std::string, std::string>& pair) const {
        if (pair.first.compare(pair.second) < 0)
        {
            return std::hash<std::string>{}(pair.first + pair.second);
        }
        else
        {
            return std::hash<std::string>{}(pair.second + pair.first);
        }
    }
};

struct LinkPairEqual {
    bool operator()(const std::pair<std::string, std::string>& a, const std::pair<std::string, std::string>& b) const {
        return (a.first == b.first || a.first == b.second) && (b.first == a.first || b.first == a.second);
    }
};


typedef std::pair<fcl::CollisionObject<double> *,fcl::CollisionObject<double> *> CollisionPair;

struct CollisionInformation
{
    CollisionInformation(): number_of_collisions(0){}
    /// @brief Number of collisions found.
    size_t number_of_collisions;
    /// @brief vector holding collision object names
    std::vector< std::pair<std::string, std::string> >  collision_object_names;
    /// @brief collision cost
    double collision_cost;

    /// @brief stores whether objects collided or not (use std::find_if, to get the first collision, if the stopping criterion is turned on)
    std::unordered_map<std::pair<std::string, std::string>, bool, LinkPairHash, LinkPairEqual> collisions;
    
    bool stop_after_first_collision;
//     /// @brief vector holding collision object names
//     std::vector< CollisionPair > collision_objects_pair;
};

struct CollisionData
{
    CollisionData(): done(false){}

    /// @brief Collision request
    fcl::CollisionRequest<double> request;
    /// @brief Collision result
    fcl::CollisionResult<double> result;
    /// @brief Information regarding the collision
    CollisionInformation collision_info;
    /// @brief Whether the collision iteration can stop
    bool done;
};

struct DistanceData
{
    DistanceData(): list_of_distance_information(0), done(false){}

    /// @brief Distance request
    fcl::DistanceRequest<double> request;
    /// @brief Distance result    
    fcl::DistanceResult<double> result;
    /// @brief Information regarding the collision
    CollisionInformation collision_info;
    /// @brief Information regarding the collision distance
    std::vector< DistanceInformation> list_of_distance_information;
    /// @brief Whether the distance iteration can stop
    bool done;

    /*
     *  NOTE: Store distances in a different format.
     */
    /// @brief Information regarding the names of stored collision objects
    std::vector<std::string> list_of_collision_objects;
    /// @brief Information regarding the collision distance (pre-allocated during initialization)
    std::unordered_map<std::pair<std::string, std::string>, DistanceInformation, LinkPairHash, LinkPairEqual> matrix_of_distance_information;

};

}// end namespace collision_detection
#endif // COLLISIONDATA_HPP
