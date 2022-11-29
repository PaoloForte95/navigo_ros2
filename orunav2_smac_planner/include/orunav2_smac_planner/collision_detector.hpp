/**
 * @file collision_detector.hpp
 * @author Paolo Forte
 
 */

#ifndef COLLISIONDETECTOR_HPP_
#define COLLISIONDETECTOR_HPP_

#include "collision_detector_interface.hpp"
#include "world_occupancy_map.hpp"
#include "configuration/configuration.hpp"


/**
 * @class CollisionDetector
 * This class determines if a given Configuration and its trajectory
 * are inside the World and if they do not collide with any object.
 * It implements the virtual class CollisionDetectorInterface
 */
namespace orunav2_smac_planner
{

class CollisionDetector : public CollisionDetectorInterface {

	/** A pointer to the WorldOccupancyMap containing the obstacles */
	WorldOccupancyMap* map_;

public:

	/**
	 * Instantiate a new CollisionDetector
	 * @param occupancyMap A pointer to the WorldOccupancyMap containing the obstacles
	 */
	CollisionDetector(WorldOccupancyMap* occupancyMap);


	virtual ~CollisionDetector();

	/**
	 * Check if the Configuration (and the trajectory that lead to it)
	 * is collision free (and inside the world's boundaries)
	 * @param conf pointer to the Configuration to check
	 * @returns true if the Configuration is collision free
	 */
	bool isCollisionFree(Configuration* conf);

	/**
	 * Check if a single cell is blocked in the world
	 * @param cellXcoord, cellYcoord The coordinates of the cell to check
	 * @return true if the cell is blocked
	 */
	bool isBlocked(short int cellXcoord, short int cellYcoord);

};

} // namespace orunav2_smac_planner

#endif /* COLLISIONDETECTOR_HPP_ */
