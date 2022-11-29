/**
 * @file path_node.hpp
 * @brief Contains the class PathNode
 * @author Paolo Forte
 *
 */

#ifndef PATHNODE_HPP_
#define PATHNODE_HPP_

#include "node.hpp"
#include "configuration/configuration.hpp"
#include "world.hpp"


/**
 * @class PathNode
 * Class that represents the basic exploration unit of a motion planning algorithm.
 */
class PathNode: public Node {

	/** Vector of pointers to vehicle Configurations representing the state of the system.
	 * The Configurations are ordered in ascending VehicleID */
	std::vector<Configuration*> currentConfigurations_;

	World* myWorld_;

	/**
	 * Ordering function to ensure that in every new PathNode the Configuration vector
	 * is ordered in the same way. The ordering is based on the vehicleID to which
	 * the Configuration belongs.
	 * @param c1 Pointer to the first Configuration
	 * @param c2 Pointer to the second Configuration
	 * @return true if the ID of the vehicle of the first Configuration is < the one of the second
	 */
	static bool configurationOrderingFunction (Configuration* c1, Configuration* c2) {
		return c1->getMission()->getVehicleID() < c2->getMission()->getVehicleID();
	}


public:

	/**

	 */
	PathNode(std::vector<Configuration*> configurations, World* w, PathNode* parent);

	/**
	 */
	PathNode(std::vector<Configuration*> newConfs, std::vector<Configuration*> unchangedConfs, World* w, PathNode* parent);

	~PathNode();

	/**
	 * Generate the children reachable from the current PathNode
	 * @returns a vector of pointers to new Nodes
	 * @todo Still to include the obstacle handling
	 */
	std::vector<Node*> generateChildren();

	/**
	 * Get a pointer to the Configurations associated to this PathNode
	 * @returns Vector to Configuration pointers
	 */
	std::vector<Configuration*> getConfigurations();

	/**
	 * Check if this node and the one passed as parameter have equivalent Configurations
	 * @param pn Pointer to the Node to compare
	 */
	bool equalContent(Node* pn);

	/**
	 * Clone the Node and return a pointer to a new Node object
	 * The clone function does not preserve the pointer to the parent
	 * Overload of base class method
	 * @returns A pointer to a new Node
	 */
	Node* clone();

	/**
	 * Print information about the node and the Configuration
	 * it points to (on log)
	 */
	void print();
};

#endif /* PATHNODE_HPP_ */
