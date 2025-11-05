#pragma once
#include <c-wcp/c-wcp.h>
#include <cnoid/MeshExtractor>
#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>

namespace cwcp {
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > generateKeepScfrConstraints(const std::shared_ptr<CWCPParam>& param);
  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> >& constraints,
                             const cnoid::LinkPtr link,
                             int level=1
                             ); // collisionConstraintについて、linkのlevel等親のリンクの干渉回避である場合、linkのBoundingBoxを追加する.
  void calcLevelLinks(const cnoid::LinkPtr inputLink,
                      int level, // input
                      std::vector<cnoid::LinkPtr>& targetLinks // output
                      ); // inputLinkのlevel等親のリンクをtargetLinksとして返す.
  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape);
  void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor);
}
