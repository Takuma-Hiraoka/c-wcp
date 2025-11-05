#include "world_common.h"

#include <cnoid/MeshGenerator>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace cwcp_sample{
  void generateWallWorld(cnoid::BodyPtr& obstacle, // for visual
                         const std::shared_ptr<cwcp::CWCPParam>& param
                         ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        rootLink->setName("floor");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2,2,0.1)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,-0.05);
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      cnoid::LinkPtr lWall = new cnoid::Link();
      {
        lWall->setName("lWall");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,0.1,1.75)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,0.6,0.9);
        posTransform->addChild(shape);
        lWall->addShapeNode(posTransform);
        rootLink->appendChild(lWall);
      }
      cnoid::LinkPtr rWall = new cnoid::Link();
      {
        rWall->setName("rWall");
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,0.1,1.75)));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.5,-0.6,0.9);
        posTransform->addChild(shape);
        rWall->addShapeNode(posTransform);
        rootLink->appendChild(rWall);
      }
      obstacle->setRootLink(rootLink);
    }
    {
      // collision world
      param->field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                         5,//size_y
                                                                                                         5,//size_z
                                                                                                         0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                                         -2.5,//origin_x
                                                                                                         -2.5,//origin_y
                                                                                                         -0.5,//origin_z
                                                                                                         0.5, // max_distance
                                                                                                         true// propagate_negative_distances
                                                                                                         );
      EigenSTL::vector_Vector3d vertices;
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->rootLink(), 0.01);
      for(int j=0;j<vertices_.size();j++){
        vertices.push_back(obstacle->rootLink()->T() * vertices_[j].cast<double>());
      }
      param->field->addPointsToField(vertices);
    }
    param->bodies.insert(obstacle);
  }
}
