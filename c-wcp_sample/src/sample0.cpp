#include <choreonoid_viewer/choreonoid_viewer.h>

#include "jaxon_common.h"
#include "world_common.h"

namespace cwcp_sample{
  void sample0(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<cwcp::CWCPParam> param = std::make_shared<cwcp::CWCPParam>();
    generateWallWorld(obstacle, param);

    cnoid::BodyPtr robot;
    generateJAXON(robot, param);

    std::vector<double> initialPose;
    global_inverse_kinematics_solver::link2Frame(param->variables, initialPose);
    // searchRegionConstraints
    {
      // pitch < 90
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = robot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      //constraint->debugLevel() = 2;
      param->searchRegionConstraints.push_back(constraint);
    }

    // goal
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->rootLink();
      constraint->B_localpos() = robot->rootLink()->T();
      constraint->B_localpos().translation() += cnoid::Vector3(0.5,0,0.5);
      constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      constraint->precision() = 1e-3;
      param->goals.push_back(constraint);
    }

    param->gikParam.goalBias = 0.4;
    param->OptimizeTrajectory = true; // 軌道最適化を行うと外れ値的な接触や接触の数自体を減らせるが、特にはじめに一気に飛んでしまう
    param->toParam.initialShortcut = true;
    param->toParam.shortcut = true;
    param->toParam.shortcutThre = param->gikParam.delta/10; // 小さくしすぎるとIKが解けない確率が増える

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    for(std::set<cnoid::BodyPtr>::iterator it=param->bodies.begin(); it != param->bodies.end(); it++) viewer->objects((*it));
    viewer->drawObjects();

    std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<cwcp::Contact> > > > path;
    if(cwcp::solveCWCP(param, path)) {
      std::cerr << "solved!" << std::endl;
      while (true) {
        for(int i=0;i<path.size();i++){
          global_inverse_kinematics_solver::frame2Link(path.at(i).first,param->variables);
          for(std::set<cnoid::BodyPtr>::iterator it=param->bodies.begin(); it != param->bodies.end(); it++) {
            (*it)->calcForwardKinematics(false);
            (*it)->calcCenterOfMass();
          }
          std::vector<cnoid::SgNodePtr> markers;
          for (int j=0;j<path.at(i).second.size();j++) {
            cnoid::SgLineSetPtr lines_ = new cnoid::SgLineSet;
            lines_->setLineWidth(8.0);
            lines_->getOrCreateColors()->resize(1);
            lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.9,0.9,0.0);
            lines_->getOrCreateVertices()->resize(2);
            lines_->colorIndices().resize(0);
            lines_->addLine(0,1); lines_->colorIndices().push_back(0); lines_->colorIndices().push_back(0);
            const std::vector<cnoid::SgNodePtr>& marker = std::vector<cnoid::SgNodePtr>{lines_};
            lines_->getOrCreateVertices()->at(0) = (path.at(i).second[j]->c1.link->T() * path.at(i).second[j]->c1.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
            lines_->getOrCreateVertices()->at(1) = (path.at(i).second[j]->c2.localPose.translation()).cast<cnoid::Vector3f::Scalar>();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
          viewer->drawOn(markers);
          viewer->drawObjects();
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
      }
    }
  }
}
