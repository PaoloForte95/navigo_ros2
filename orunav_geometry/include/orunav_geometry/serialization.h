#pragma once

#include <orunav_geometry/polygon.h>
#include <orunav2_generic/serialization.hpp>

namespace boost {
  namespace serialization {
    class access;
    // --------------------------------- Polygon ----------------------------
    template<typename Archive>
      void serialize(Archive& ar, orunav_geometry::Polygon& p, const unsigned version) {
      ar & p.points;
    }
    /* template<typename Archive> */
    /*   void save(Archive& ar, const orunav_geometry::Polygon& obj, const unsigned version) { */
    /*   ar << obj.points; */
    /* } */
    
    /* template<typename Archive> */
    /*   void load(Archive& ar, orunav_geometry::Polygon& obj, const unsigned version) { */
    /*   ar >> obj.points; */
    /* } */
  }
}


