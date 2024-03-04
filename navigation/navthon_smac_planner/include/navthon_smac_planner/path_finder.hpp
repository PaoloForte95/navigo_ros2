// Copyright (c) 2022, Paolo Forte
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef PATHFINDER_HPP_
#define PATHFINDER_HPP_

#include <math.h>
#include "boost/date_time/posix_time/posix_time.hpp"


#include "world.hpp"
#include "world_occupancy_map.hpp"
#include "ara_star_path_planner.hpp"
#include "a_star_path_planner.hpp"
#include "path_node.hpp"



namespace navthon_smac_planner
{

/**
 */
class PathFinder {

private:
	

	World* myWorld_;


	VehicleMission* mission_;

	/** If true (false, by default) enables the PathPlanner to extract data for the
	 * current vehicle Heuristic Table during the search. Only possible using an
	 * optimal A* search and a single vehicle*/
	bool dataGatheringForVehicleHT_;

	/** If different than 0, it specifies the time bound (in seconds) allowed to the
	 * PathPlanner to provide a feasible, albeit not optimal solution. If the timeBound
	 * variable is set, then ARA* is employed */
	int timeBound_;

	/** Pointer to the initial world occupancy map -- if provided */
	WorldOccupancyMap* worldMap_;

	/** Size of the World, x axis */
	double xSize_;
	/** Size of the World, y axis */
	double ySize_;

public:

	/**
	 * Create a new PathFinder object
	 * @param x,y Dimensions of the World
	 */
	PathFinder(double x, double y);

	/**
	 * Create a new PathFinder object from map
	 * @param filename The file name of the map
	 */
	PathFinder(std::string filename);

	/**
	 * Create a new PathFinder directly using a map object
	 * @param map The map (will be copied).
	 */
	PathFinder(const WorldOccupancyMap& map);

	virtual ~PathFinder();

	/**
	 * Set the occupancy map (if it exists) as a portion of the current one
	 * @param xfrom,yfrom Coordinates (in meters) of the point of the current
	 * occupancy map that will become the left lower corner of the submap
	 * @param xto, yto Coordinates (in meters) of the point of the current
	 * occupancy map that will become the top right corner of the submap
	 */
	void selectSubMap(double xfrom, double yfrom, double xto, double yto);

	/**
	 * Add a mission for a single vehicle to PathFinder
	 * @param m A pointer to the VehicleMission to be solved
	 */
	void addMission(VehicleMission* m);

	/**
	 * Solve the problem
	 * @param visualization Enable/disable visualization
	 * @returns A vector of vectors of pointers to Configurations solving the problem (1 vector per vehicle)
	 */
	std::vector<Configuration*>  solve(bool visualization);

	/**
	 * Extract the waypoints from a solution vector (of Configuration)
	 * @param path A vector of Configuration from which the solution is extracted
	 * @returns A vector of waypoints
	 */
	std::vector<waypoint*> extractWaypoints(std::vector<Configuration*> path);

	/**
	 * Enables the data gathering to populate the heuristic table of the
	 * vehicle. Only possible with an optimal A* search and a single vehicle.
	 */
	void enableDataGatheringForVehicleHT();

	/**
	 * Set the time bound for the PathPlanner to return a feasible, albeit not optimal
	 * solution. When timeBound is greater than 0, an ARA* algorithm is employed.
	 * @param seconds The time bound (in seconds) for the PathPlanner to provide a solution
	 */
	void setTimeBound(int seconds);

	/**
	 * Prints the final paths on the log file and on the visualization (it active)
	 * @param paths The vector of Configuration vectors to print
	 */
	void printPaths(std::vector<std::vector<Configuration*> > paths);

};


}  // namespace navthon_smac_planner

#endif /* PATHFINDER_HPP_ */
