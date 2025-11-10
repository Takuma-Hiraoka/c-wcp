#include "jaxon_common.h"

#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <iostream>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>

namespace cwcp_sample{
  void generateJAXON(cnoid::BodyPtr& robot, std::shared_ptr<cwcp::CWCPParam>& param) {
    cnoid::BodyLoader bodyLoader;
    robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    robot->setName("JAXON");
    if(!robot) std::cerr << "!robot" << std::endl;
    param->bodies.push_back(robot);
    std::vector<std::string> contactableLinkNames{
    "LARM_JOINT7",
    "RARM_JOINT7",
    "LLEG_JOINT5",
    "RLEG_JOINT5",
      };
    // reset manip pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
                                         0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
                                         0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
                                         0.0, 0.0, 0.0, // torso
                                         0.0, 0.0, // head
                                         0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
                                         0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
                                         -1.3, 1.3, // lfinger
                                         -1.3, 1.3, // rfinger
    };
 
    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    // variables
    {
      param->variables.push_back(robot->rootLink());
      for(int i=0;i<robot->numJoints();i++){
        if ((robot->joint(i)->name() == "motor_joint") ||
            (robot->joint(i)->name() == "LARM_F_JOINT0") ||
            (robot->joint(i)->name() == "LARM_F_JOINT1") ||
            (robot->joint(i)->name() == "RARM_F_JOINT0") ||
            (robot->joint(i)->name() == "RARM_F_JOINT1")) continue;
        param->variables.push_back(robot->joint(i));
      }
    }

    param->projectLink = robot->rootLink();

    // task: nominal constairnt
    {
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        param->nominals.push_back(constraint);
      }
    }

    param->currentContactPoints.clear();
    {
      {
        std::shared_ptr<cwcp::Contact> lleg = std::make_shared<cwcp::Contact>();
        lleg->c1.link = robot->link("LLEG_JOINT5");
        lleg->c1.localPose.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        lleg->c2.localPose = lleg->c1.link->T() * lleg->c1.localPose;
        param->currentContactPoints.push_back(lleg);
      }
      {
        std::shared_ptr<cwcp::Contact> rleg = std::make_shared<cwcp::Contact>();
        rleg->c1.link = robot->link("RLEG_JOINT5");
        rleg->c1.localPose.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        rleg->c2.localPose = rleg->c1.link->T() * rleg->c1.localPose;
        param->currentContactPoints.push_back(rleg);
      }
    }

    param->constraints.clear();
    // joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      param->constraints.push_back(constraint);
    }
    // joint displacement
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointDisplacementConstraint> constraint = std::make_shared<ik_constraint2::JointDisplacementConstraint>();
      constraint->joint() = robot->joint(i);
      param->constraints.push_back(constraint);
    }

    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> > collisionModels;
    for(int i=0;i<robot->numLinks();i++){
      collisionModels[robot->link(i)] = choreonoid_bullet::convertToBulletModel(robot->link(i)->collisionShape());
    }
    for(int b=0; b<param->bodies.size(); b++) {
      if (param->bodies[b] == robot) continue;
      for(int i=0;i<param->bodies[b]->numLinks();i++){
        if (param->bodies[b]->link(i)->name() == "floor") continue; // floorはdistance fieldを、rWall, lWallはbulletを使う
        collisionModels[param->bodies[b]->link(i)] = choreonoid_bullet::convertToBulletModel(param->bodies[b]->link(i)->collisionShape());
      }
    }

    // environmental collision
    for (int i=0; i<robot->numLinks(); i++) {
      {
        if ((robot->link(i)->name() == "LLEG_JOINT4") || // JOINT4は足首内側に入り込んだリンクなので考慮しなくて良い
            (robot->link(i)->name() == "RLEG_JOINT4") ||
            (robot->link(i)->name() == "LARM_JOINT5") || // JOINT4は肘下で前腕が触れるなら本来は触れている
            (robot->link(i)->name() == "RARM_JOINT5") ||
            (robot->link(i)->name() == "LARM_JOINT6") || // JOINT6は手首内側に入り込んだリンクなので考慮しなくて良い
            (robot->link(i)->name() == "RARM_JOINT6") ||
            (robot->link(i)->name() == "LARM_F_JOINT0") ||
            (robot->link(i)->name() == "LARM_F_JOINT1") ||
            (robot->link(i)->name() == "RARM_F_JOINT0") ||
            (robot->link(i)->name() == "RARM_F_JOINT1")) continue;
      }
      {
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = robot->link(i);
        constraint->field() = param->field;
        constraint->tolerance() = 0.02;
        constraint->precision() = 0.01;
        constraint->ignoreDistance() = 0.5;
        constraint->updateBounds();
        param->constraints.push_back(constraint);
      }
      for(int b=0; b<param->bodies.size(); b++) {
        if (param->bodies[b] == robot) continue;
        for(int j=0;j<param->bodies[b]->numLinks();j++){
          if (param->bodies[b]->link(j)->name() == "floor") continue; // floorはdistance fieldを、rWall, lWallはbulletを使う
          std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
          constraint->A_link() = robot->link(i);
          constraint->B_link() = param->bodies[b]->link(j);
          constraint->A_link_bulletModel() = constraint->A_link();
          constraint->A_bulletModel().push_back(collisionModels[constraint->A_link()]);
          constraint->B_link_bulletModel() = constraint->B_link();
          constraint->B_bulletModel().push_back(collisionModels[constraint->B_link()]);
          constraint->tolerance() = 0.02;
          constraint->precision() = 0.01;
          constraint->ignoreDistance() = 0.5;
          constraint->updateBounds();
          param->constraints.push_back(constraint);
        }
      }
    }

    // task: self collision
    {
      std::vector<std::vector<std::string> > pairs {
        std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","WAIST"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT3","WAIST"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT4","WAIST"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT5","WAIST"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT6","WAIST"}, std::vector<std::string>{"LARM_JOINT2","WAIST"}, std::vector<std::string>{"LARM_JOINT3","WAIST"}, std::vector<std::string>{"LARM_JOINT4","WAIST"}, std::vector<std::string>{"LARM_JOINT5","WAIST"}, std::vector<std::string>{"LARM_JOINT6","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT7"}, std::vector<std::string>{"LARM_JOINT7","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT7","WAIST"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT7"}/*, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT2}*/ // JAXON_JVRCは肩周りで干渉する
      };

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->A_link_bulletModel() = constraint->A_link();
        constraint->A_bulletModel().push_back(collisionModels[constraint->A_link()]);
        constraint->B_link_bulletModel() = constraint->B_link();
        constraint->B_bulletModel().push_back(collisionModels[constraint->B_link()]);
        constraint->tolerance() = 0.002;
        constraint->updateBounds();
        param->constraints.push_back(constraint);
      }
    }

    // reachability
    for (int i=0; i<contactableLinkNames.size(); i++) {
      if (!robot->link(contactableLinkNames[i])) continue;
      std::shared_ptr<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint>();
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> sdfConstraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      sdfConstraint->A_link() = robot->link(contactableLinkNames[i]);
      sdfConstraint->field() = param->field;
      sdfConstraint->tolerance() = 0.3;
      //          constraint->precision() = 0.01;
      sdfConstraint->ignoreDistance() = 0.5;
      sdfConstraint->invert() = true;
      constraint->A_link() = sdfConstraint->A_link();
      constraint->collisionConstraints().push_back(sdfConstraint);
      for(int b=0; b<param->bodies.size(); b++) {
        if (param->bodies[b] == robot) continue;
        for(int j=0;j<param->bodies[b]->numLinks();j++){
          if (param->bodies[b]->link(j)->name() == "floor") continue; // floorはdistance fieldを、rWall, lWallはbulletを使う
          std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> bulletConstraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
          bulletConstraint->A_link() = robot->link(contactableLinkNames[i]);
          bulletConstraint->B_link() = param->bodies[b]->link(j);
          bulletConstraint->A_link_bulletModel() = bulletConstraint->A_link();
          bulletConstraint->A_bulletModel().push_back(collisionModels[bulletConstraint->A_link()]);
          bulletConstraint->B_link_bulletModel() = bulletConstraint->B_link();
          bulletConstraint->B_bulletModel().push_back(collisionModels[bulletConstraint->B_link()]);
          bulletConstraint->tolerance() = 0.3;
          bulletConstraint->ignoreDistance() = 0.5;
          bulletConstraint->invert() = true;
          constraint->collisionConstraints().push_back(bulletConstraint);
        }
      }
      constraint->updateBounds();
      param->reachabilityConstraints.push_back(constraint);
    }

  }
}
