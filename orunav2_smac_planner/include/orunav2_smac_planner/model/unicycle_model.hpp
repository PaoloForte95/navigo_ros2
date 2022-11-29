/**
 * @file unicycle_model.hpp
 * @author Paolo Forte
 *
 */

#ifndef UNICYCLEMODEL_HPP_
#define UNICYCLEMODEL_HPP_

#include "vehicle_model.hpp"
#include "orunav2_smac_planner/world_parameters.hpp"

/**
 * @class UnicycleModel
 * Models the Unicycle basic properties
 * Note: The Unicycle has its center in its current position
 */
class UnicycleModel: public VehicleModel {

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
	 * Constructor of a UnicycleModel
	 * The modelPrimitive source file must have normalized angles (-pi / pi).
	 * The physical width and length of the Unicycle are contained in the model primitives file
	 * @param modelPrimitivesFilename The name of the file of the motion primitives, without path!
	 */
	UnicycleModel(std::string modelPrimitivesFilename);

	virtual ~UnicycleModel();

	/**
	 * Returns a vectors of pointers to the cells occupied by a unicycle in
	 * a specific point.
	 * @param p A pointer to the point where the vehicle is positioned
	 * @return The vector of pointers of cells occupied
	 */
	virtual std::vector<cellPosition*> getCellsOccupiedInPosition(vehicleSimplePoint* p);

};

#endif /* UNICYCLEMODEL_HPP_ */
