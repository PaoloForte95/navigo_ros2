/**
 * @file lhd_model.hpp
 * @author Paolo Forte
 *

 */

#ifndef LHDMODEL_HPP_
#define LHDMODEL_HPP_

#include "vehicle_model.hpp"
#include "navthon_smac_planner/world_parameters.hpp"


namespace navthon_smac_planner
{

/**
 * @class LHDModel
 * Models a waist actuated vehicle properties
 * Note: The current position of the vehicle is represented by the middle point of its front axle
 */
class LHDModel: public VehicleModel {

	/** The length of the vehicle, from the back axle to the middle joint, in meters */
	double lhdBackLength_;
	/** The length of the vehicle, from the middle joint to the front axle, in meters */
	double lhdFrontLength_;
	/** The length of the vehicle, from the back axle to the back, in meters */
	double lhdBackFromAxle_;
	/** The length of the vehicle, from the front axle to the front, in meters */
	double lhdFrontFromAxle_;
	/** The vehicle max angle joint angle, in radians */
	double lhdBoundPhi_;


protected:
	/**
	 * Implementation of the virtual function. Generates the file that contains the
	 * additional data of the motion primitives (length, occupancy)
	 */
	void generatePrimitiveAdditionalData();

	/**
	 * Implementation of the virtual function. Load motion primitives
	 */
	void loadPrimitiveLookupTable();

public:

	/**
	 * Constructor of a LHDModel
	 * The modelPrimitive source file must have normalized angles (-pi / pi).
	 * All information about the model (width, distances) must be contained into the primitives file
	 * @param modelPrimitivesFilename The name of the file of the motion primitives, without path!
	 */
	LHDModel(std::string modelPrimitivesFilename);

	virtual ~LHDModel();

	/**
	 * Returns a vectors of pointers to the cells occupied by a waist actuated vehicle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p);

	/**
	 * Get the length of the vehicle, from the back axle to the middle joint, in meters
	 * @returns the length of the vehicle, from the back axle to the middle joint, in meters
	 */
	double getLHDBackLength();

	/**
	 * Get the length of the vehicle, from the middle joint to the front axle, in meters
	 * @returns the length of the vehicle, from the middle joint to the front axle, in meters
	 */
	double getLHDFrontLength();

	/**
	 * Get the length of the vehicle, from the back axle to the back, in meters
	 * @returns length of the vehicle, from the back axle to the back, in meters
	 */
	double getLHDBackFromAxle();

	/**
	 * Get the length of the vehicle, from the front axle to the front, in meters
	 * @returns the length of the vehicle, from the front axle to the front, in meters
	 */
	double getLHDFrontFromAxle();

	/**
	 * Get the vehicle's max angle joint angle, in radians
	 * @returns the vehicle's max angle joint angle, in radians
	 */
	double getLHDMaxSteeringAngle();

};


}  // namespace navthon_smac_planner

#endif /* LHDMODEL_HPP_ */
