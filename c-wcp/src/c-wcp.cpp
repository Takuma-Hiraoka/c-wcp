#include <c-wcp/c-wcp.h>
#include <c-wcp/util.h>

namespace cwcp {
  bool solveCWCP(const std::shared_ptr<CWCPParam>& param,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath) {
    if (param->bodies.size() == 0) { std::cerr << "[solveCWCP] error!! no bodies!!" << std::endl; return false;}
    if (param->variables.size() == 0) { std::cerr << "[solveCWCP] error!! no variables!!" << std::endl; return false;}
    if (!param->projectLink) { std::cerr << "[solveCWCP] error!! no projectLink!!" << std::endl; return false;}
    if (param->reachabilityConstraints.size() == 0) { std::cerr << "[solveCWCP] error!! no reachabilityConstraints!!" << std::endl; return false;}
    if (param->goals.size() == 0) { std::cerr << "[solveCWCP] error!! no goals!!" << std::endl; return false;}
    if (param->currentContactPoints.size() == 0) { std::cerr << "[solveCWCP] error!! no currentContactPoints!!" << std::endl; return false;}

    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;

    if (param->searchRegionConstraints.size() == 0) { // 適当
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->C().resize(1,6);
      constraint->C().insert(0,0) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 1e10;
      param->searchRegionConstraints.push_back(constraint);
    }
    constraints.push_back(param->searchRegionConstraints);

    constraints.push_back(param->constraints);
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint>> keepScfrConstraints = generateKeepScfrConstraints(param);
    constraints.back().insert(constraints.back().end(), keepScfrConstraints.begin(), keepScfrConstraints.end());

    for (int i=0; i<constraints.size(); i++) {
      for (int j=0; j<constraints[i].size(); j++) {
        constraints[i][j]->debugLevel() = 0;
        constraints[i][j]->updateBounds();
      }
    }

    // まず今触れている接触を僅かに離す. 初期状態をsatisfiedにするため.
    {
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > breakConstraints;
      for (int i=0; i<param->currentContactPoints.size(); i++) {
        // link1のlocalPoseはZ正方向が接触から離れる方向である前提
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = param->currentContactPoints[i]->c1.link;
        constraint->A_localpos() = param->currentContactPoints[i]->c1.localPose;
        constraint->B_link() = param->currentContactPoints[i]->c2.link;
        constraint->B_localpos() = param->currentContactPoints[i]->c2.localPose;
        constraint->B_localpos().translation() += param->currentContactPoints[i]->c2.localPose.linear() * cnoid::Vector3(0,0,param->initialBreakHeight);
        constraint->eval_link() = nullptr;
        breakConstraints.push_back(constraint);
      }
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > preConstraints = constraints;
      preConstraints.push_back(breakConstraints);
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
      prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                          preConstraints,
                                                          prevTasks,
                                                          param->pikParam
                                                          );
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections{keepScfrConstraints};
    param->gikParam.projectLink.push_back(param->projectLink);
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    if(!global_inverse_kinematics_solver::solveGIK(param->variables,
                                                   constraints,
                                                   param->goals,
                                                   param->nominals,
                                                   rejections,
                                                   param->gikParam,
                                                   path
                                                   )){
      std::cerr << "[solveCWCP] gik failed" << std::endl;
      return false;
    }

    // 関節角軌道が暴れているので軌道最適化
    if (param->OptimizeTrajectory) {
      param->toParam.pikParam.convergeThre=param->gikParam.pikParam.convergeThre * path->size();
      trajectory_optimizer::solveTO(param->variables,
                                    constraints,
                                    param->toParam,
                                    path);
    }

    outputPath.resize(path->size());
    for (int i=0; i<path->size(); i++) {
      outputPath[i].first = path->at(i);
      global_inverse_kinematics_solver::frame2Link(path->at(i),param->variables);
      for(std::set<cnoid::BodyPtr>::iterator it=param->bodies.begin(); it != param->bodies.end(); it++) {
        (*it)->calcForwardKinematics(false);
        (*it)->calcCenterOfMass();
      }
      std::vector<std::shared_ptr<Contact> >contacts;
      for (int j=0; j<param->reachabilityConstraints.size(); j++) {
        param->reachabilityConstraints[j]->updateBounds();
        if (param->reachabilityConstraints[j]->isSatisfied()) {
          std::shared_ptr<Contact> contact = std::make_shared<Contact>();
          contact->c1.link = param->reachabilityConstraints[j]->A_link();
          contact->c1.localPose.translation() =  param->reachabilityConstraints[j]->A_currentLocalp();
          contact->c2.link = nullptr;
          contact->c2.localPose.translation() =  param->reachabilityConstraints[j]->B_currentLocalp();
          cnoid::Vector3d z_axis = param->reachabilityConstraints[j]->currentDirection();
          cnoid::Vector3d x_axis = (z_axis==cnoid::Vector3d::UnitY() || z_axis==-cnoid::Vector3d::UnitY()) ? cnoid::Vector3d::UnitZ() : cnoid::Vector3d::UnitY().cross(z_axis);
          cnoid::Vector3d y_axis = z_axis.cross(x_axis);
          contact->c2.localPose.linear().col(0) = x_axis.normalized(); contact->c2.localPose.linear().col(1) = y_axis.normalized(); contact->c2.localPose.linear().col(2) = z_axis.normalized();
          contacts.push_back(contact);
        }
      }
      outputPath[i].second = contacts;
    }

    global_inverse_kinematics_solver::frame2Link(initialPose,param->variables);
    for(std::set<cnoid::BodyPtr>::iterator it=param->bodies.begin(); it != param->bodies.end(); it++) {
      (*it)->calcForwardKinematics(false);
      (*it)->calcCenterOfMass();
    }
    return true;

  }
}
