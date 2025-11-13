#pragma once

#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_scfr/ik_constraint2_scfr.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <trajectory_optimizer/trajectory_optimizer.h>
#include <c-wcp/robot_state.h>

namespace cwcp {
  class CWCPParam {
  public:
    std::vector<cnoid::BodyPtr> bodies;
    std::vector<cnoid::LinkPtr> variables;
    cnoid::LinkPtr projectLink = nullptr;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > searchRegionConstraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    std::vector<std::shared_ptr<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint> > reachabilityConstraints; // リンクごとにまとめてpush_backすること
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals;
    std::vector<std::shared_ptr<Contact> > currentContactPoints;
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field = nullptr;

    int debugLevel = 0;
    double initialBreakHeight = 0.05;
    int minimumContactCount = 5;
    global_inverse_kinematics_solver::GIKParam gikParam;
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    trajectory_optimizer::TOParam toParam;
    bool OptimizeTrajectory = false;

    CWCPParam() {
      gikParam.range = 0.5;
      gikParam.delta = 0.2;
      gikParam.timeout = 20;
      gikParam.maxTranslation = 3;
      gikParam.threads = 12;
      gikParam.goalBias = 0.2;
      gikParam.projectCellSize = 0.2;
      gikParam.pikParam.we = 3e1;
      gikParam.pikParam.wmax = 3e0;
      gikParam.pikParam.maxIteration = 100;
      gikParam.pikParam.minIteration = 20;
      gikParam.pikParam.checkFinalState = true;
      gikParam.pikParam.calcVelocity = false;
      gikParam.pikParam.convergeThre = 5e-2;
      gikParam.pikParam.pathOutputLoop = 2;
      pikParam = gikParam.pikParam;
      toParam.shortcutThre=4e-2;
    };
  };
  bool solveCWCP(const std::shared_ptr<CWCPParam>& param,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath);
  bool generateKeyPose(const std::shared_ptr<CWCPParam>& param,
                       const std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& guidePath,
                       std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& keyPosePath);
}
