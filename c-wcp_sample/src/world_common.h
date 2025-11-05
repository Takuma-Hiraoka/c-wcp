#pragma once
#include <cnoid/Body>
#include <c-wcp/c-wcp.h>

namespace cwcp_sample{
  void generateWallWorld(cnoid::BodyPtr& obstacle, // for visual
                         const std::shared_ptr<cwcp::CWCPParam>& param);

};
