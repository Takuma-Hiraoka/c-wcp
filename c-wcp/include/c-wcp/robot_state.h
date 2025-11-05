#pragma once

#include <cnoid/Body>

namespace cwcp {
  struct ContactInfo{
    cnoid::LinkPtr link = nullptr; // nullptrならworld
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();
  };
  struct Contact{
    ContactInfo c1;
    ContactInfo c2;
  };

}
