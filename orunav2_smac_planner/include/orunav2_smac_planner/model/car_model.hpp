/**
 * @file car_model.hpp
 * @author Paolo Forte
 * @brief 
 * @version 0.1
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CARMODEL_HPP_
#define CARMODEL_HPP_

#include "vehicle_model.hpp"
#include "orunav2_smac_planner/world_parameters.hpp"

/**
 * @class CarModel
 * Models a car basic properties
 * Note: The current position of a car is represented by the middle point
 * of the rear axel
 */
class CarModel: public VehicleModel {

	/** The length of the car, from the back axle to the front, in meters */
	double carFrontLength_;
	/** The length of the car, from the back axle to the back, in meters */
	double carBackLength_;
	/** The car max steering angle, in radians */
	double carMaxSteeringAngle_;

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
	 * Constructor of a CarModel
	 * The modelPrimitive source file must have normalized angles (-pi / pi).
	 * The physical width and length (front and back from the back axle)
	 * of the Car are contained in the model primitives file
	 * @param modelPrimitivesFilename The name of the file of the motion primitives, without path!
	 */
	CarModel(std::string modelPrimitivesFilename);

	virtual ~CarModel();

	/**
	 * Returns a vectors of pointers to the cells occupied by a car like vehicle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p);


	/**
	 * Get the length of the car, from the back axle to the front, in meters
	 * @returns the length of the car, from the back axle to the front, in meters
	 */
	double getCarFrontLength();

	/**
	 * Get the length of the car, from the back axle to the back, in meters
	 * @returns the length of the car, from the back axle to the back, in meters
	 */
	double getCarBackLength();

	/**
	 * Get the car max steering angle, in radians
	 * @returns the car max steering angle, in radians
	 */
	double getCarMaxSteeringAngle();

};

#endif /* CARMODEL_HPP_ */
