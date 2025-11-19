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

    param->gikParam.projectLink.push_back(param->projectLink);
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    if(!global_inverse_kinematics_solver::solveGIK(param->variables,
                                                   constraints,
                                                   param->goals,
                                                   param->nominals,
                                                   constraints.back(),
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
      for (int b=0; b<param->bodies.size(); b++) {
        param->bodies[b]->calcForwardKinematics(false);
        param->bodies[b]->calcCenterOfMass();
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
    for (int b=0; b<param->bodies.size(); b++) {
      param->bodies[b]->calcForwardKinematics(false);
      param->bodies[b]->calcCenterOfMass();
    }
    return true;

  }

  bool generateKeyPose(const std::shared_ptr<CWCPParam>& param,
                       const std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& guidePath,
                       std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& keyPosePath) {
    keyPosePath.clear();
    std::vector<std::vector<double> > tolerancess;
    std::vector<std::vector<double> > precisionss;
    int convergedLevel = param->pikParam.satisfiedConvergeLevel;
    for(int r=0;r<param->reachabilityConstraints.size();r++){
      std::vector<double> tolerances;
      std::vector<double> precisions;
      for (int c=0; c<param->reachabilityConstraints[r]->collisionConstraints().size(); c++) {
        tolerances.push_back(param->reachabilityConstraints[r]->collisionConstraints()[c]->tolerance());
        precisions.push_back(param->reachabilityConstraints[r]->collisionConstraints()[c]->precision());
        param->reachabilityConstraints[r]->collisionConstraints()[c]->tolerance() = 0.05;
        param->reachabilityConstraints[r]->collisionConstraints()[c]->precision() = 1e10; // always satisfied
      }
      tolerancess.push_back(tolerances);
      precisionss.push_back(precisions);
    }

    for (int i=0; i<guidePath.size(); i++) {
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      for(int r=0;r<param->reachabilityConstraints.size();r++) {
        for (int c=0; c<guidePath[i].second.size(); c++) {
          if (guidePath[i].second[c]->c1.link == param->reachabilityConstraints[r]->A_link()) nominals.push_back(param->reachabilityConstraints[r]);
        }
      }
      global_inverse_kinematics_solver::frame2Link(guidePath[i].first, param->variables);
      for (int b=0; b<param->bodies.size(); b++) {
        param->bodies[b]->calcForwardKinematics(false);
        param->bodies[b]->calcCenterOfMass();
      }
      if(guidePath[i].second.size() == 0) {
        std::cerr << "[generateKeyPose] no contacts." << std::endl;
        return false;
      }
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
      for (int c=0; c<param->constraints.size(); c++) constraints0.push_back(param->constraints[c]);
      std::set<cnoid::BodyPtr> active_bodies;
      for(int c=0;c<param->reachabilityConstraints.size();c++){
        if(param->reachabilityConstraints[c]->A_link()->body()) active_bodies.insert(param->reachabilityConstraints[c]->A_link()->body());
      }
      std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
      scfrConstraint->A_robot() = (*active_bodies.begin());
      std::vector<cnoid::Isometry3> poses;
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
      std::vector<cnoid::VectorX> bs;
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorX> dls;
      std::vector<cnoid::VectorX> dus;
      for (int c=0; c<guidePath[i].second.size(); c++) {
        poses.push_back(guidePath[i].second[c]->c2.localPose);
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
        C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
        C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
        C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        Cs.push_back(C);
        cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
        dls.push_back(dl);
        cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 2000.0;
        dus.push_back(du);
      }
      scfrConstraint->poses() = poses;
      scfrConstraint->As() = As;
      scfrConstraint->bs() = bs;
      scfrConstraint->Cs() = Cs;
      scfrConstraint->dls() = dls;
      scfrConstraint->dus() = dus;
      scfrConstraint->SCFRParam().eps = 0.2;
      scfrConstraint->SCFRParam().lpTolerance = 1e-7;
      scfrConstraint->maxCError() = 0.1;
      constraints0.push_back(scfrConstraint);
      bool solved = false;
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, nominals};
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
      solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                                     constraints,
                                                                     prevTasks,
                                                                     param->pikParam
                                                                     );
      if (!solved) { // 幾何形状の問題から単に位距離を近づけるだけでは重心制約等が解けなくなる可能性がある. このため解けなかった場合は諦めてガイドパスをそのまま使う.
        keyPosePath.push_back(guidePath[i]);
      } else {
        std::vector<double> frame;
        global_inverse_kinematics_solver::link2Frame(param->variables, frame);
        keyPosePath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, guidePath[i].second));
      }

    }

    if (param->debugLevel >= 1) std::cerr << "[generateKeyPose] succeeded." << std::endl;
    global_inverse_kinematics_solver::frame2Link(guidePath[0].first,param->variables);
    for (int b=0; b<param->bodies.size(); b++) {
      param->bodies[b]->calcForwardKinematics(false);
      param->bodies[b]->calcCenterOfMass();
    }

    for(int r=0;r<param->reachabilityConstraints.size();r++){
      for (int c=0; c<param->reachabilityConstraints[r]->collisionConstraints().size(); c++) {
        param->reachabilityConstraints[r]->collisionConstraints()[c]->tolerance() = tolerancess[r][c];
        param->reachabilityConstraints[r]->collisionConstraints()[c]->precision() = precisionss[r][c];
      }
    }
    param->pikParam.satisfiedConvergeLevel = convergedLevel;

    return true;
  }

}
