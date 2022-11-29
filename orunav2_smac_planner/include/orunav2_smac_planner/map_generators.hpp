/**
 * @file map_generators.hpp
 * @author Paolo Forte
 * @brief 
 * @version 0.1
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MAPGENERATORS_HPP_
#define MAPGENERATORS_HPP_

#include <vector>
#include "math.h"
#include <typeinfo>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include "world_parameters.hpp"

/**
 * Generate a random map of xSize,ySize with a specific space granularity and
 * a number of obstacles (obstacles) with radius obstRad. The obstacles are randomly
 * distributed between [xObstMin, xObstMax] and [yObstMin, yObstMax]
 * The map is saved into a file specified by filename
 * @param xSize, ySize Size of the map, in meters
 * @param gran Map granularity, meters/cell
 * @param obstacles Number of obstacles to generate
 * @param obstRad Radius of the obstacles, in meters
 * @param xObstMin,xObstMax,yObstMin,yObstMax Coordinates of the area where the obstacles
 * should be, in meters
 * @param filename The name of the file (NO full path nor ".map")
 */
void generateRandomMap(double xSize, double ySize, double gran, unsigned int obstacles, double obstRad,
		double xObstMin, double xObstMax, double yObstMin, double yObstMax, std::string filename);

/**
 * Generate a narrow passage map of xSize,ySize with a specific space granularity
 * The map is saved into a file specified by filename
 * @param xSize, ySize Size of the map, in meters
 * @param gran Map granularity, meters/cell
 * @param xLowLeft,xTopRight,yLowLeft,yTopRight position of the boudaries (meters)
 * @param passageSize size of the passage (meters)
 * @param filename The name of the file (NO full path nor ".map")
 */
void generateNarrowPassageMap(double xSize, double ySize, double gran, double xLowLeft, double xTopRight,
		double yLowLeft, double yTopRight, double passageSize, std::string filename);

#endif /* MAPGENERATORS_HPP_ */
