/**
 * @file lhd_configuration.cpp
 * @author Paolo Forte
 * @brief 
 * @version 0.1
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "orunav2_smac_planner/configuration/lhd_configuration.hpp"

LHDConfiguration::LHDConfiguration(unsigned short int xCell, unsigned short int yCell,
		uint8_t orientID, uint8_t steerID, VehicleMission* vm) : Configuration(xCell, yCell, orientID, steerID, vm) {
	// check that the model is the correct one
	if (!dynamic_cast<orunav2_smac_planner::LHDModel*>(vm->getVehicleModel())) {
		exit(0);
	}
}

LHDConfiguration::~LHDConfiguration() {
}

LHDConfiguration::LHDConfiguration(const LHDConfiguration& origin) :
						Configuration(origin) {
}

std::vector<Configuration*> LHDConfiguration::generateNewConfigurations() {

	std::vector<Configuration*> result;
	std::vector<MotionPrimitiveData*> primitives;
	primitives = dynamic_cast<orunav2_smac_planner::LHDModel*>
	(this->getMission()->getVehicleModel())->getApplicablePrimitives(this->getOrientationID(), this->getSteeringID());

	for (std::vector<MotionPrimitiveData*>::iterator primit = primitives.begin(); primit != primitives.end(); primit++) {
		// create a new Configuration for each primitive
		int xc = (*primit)->getXOffset() + this->getXCell();
		int yc = (*primit)->getYOffset() + this->getYCell();
		// configurations must be in non-negative cells
		if (xc >= 0 && yc >= 0) {
			LHDConfiguration* newConf = new LHDConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}


std::vector<Configuration*> LHDConfiguration::generateNewConfigurations(World* w) {

	std::vector<Configuration*> result;
	std::vector<MotionPrimitiveData*> primitives;
	primitives = this->getMission()->getVehicleModel()->selectApplicablePrimitives(w, this->getXCell(),
			this->getYCell(), this->getOrientationID(),this->getSteeringID());

	for (std::vector<MotionPrimitiveData*>::iterator primit = primitives.begin(); primit != primitives.end(); primit++) {
		// create a new Configuration for each primitive
		int xc = (*primit)->getXOffset() + this->getXCell();
		int yc = (*primit)->getYOffset() + this->getYCell();
		// configurations must be in non-negative cells
		if (xc >= 0 && yc >= 0) {
			LHDConfiguration* newConf = new LHDConfiguration(xc, yc, (*primit)->getFinalOrientationID(),
					(*primit)->getFinalSteeringID(), this->getMission());
			newConf->setConfigurationPrimitive((*primit));
			result.push_back(newConf);
		}
	}
	return result;
}


Configuration* LHDConfiguration::clone() {
	return new LHDConfiguration(*this);
}

