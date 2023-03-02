#pragma once

#include <orunav2_smac_planner/path_finder.hpp>
#include "nav2_costmap_2d/costmap_2d.hpp"


void convertNavMsgsOccupancyGridToWorldOccupancyMapRef(nav2_costmap_2d::Costmap2D * costmap, WorldOccupancyMap &map)
{
  double granularity = costmap->getResolution();

  int xcells = costmap->getSizeInCellsX();
  int ycells = costmap->getSizeInCellsY();
  
  std::vector<std::vector<double> > occupancyMap;
  occupancyMap.resize(ycells);

  unsigned int k = 0;
  for (unsigned int i = 0; i < ycells; i++)
    {
      occupancyMap[i].resize(xcells);
      for (unsigned int j = 0; j < xcells; j++)
      {
        occupancyMap[i][j] = costmap->getCost(k)*0.01;
        k++;
      }
    }
  
  map.initialize(xcells, ycells, granularity, occupancyMap);
}
