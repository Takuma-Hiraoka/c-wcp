#include <c-wcp/util.h>
#include <cnoid/MeshExtractor>

namespace cwcp {
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > generateKeepScfrConstraints(const std::shared_ptr<CWCPParam>& param) {
    // reachabilityConstraintのうち、隣接リンクの凸法に含まれる点は接触不可能とする. 足裏が地面に触れるときに脛も触れるとされては困るため. こうすると逆にリンク間の接合点(例:肘)で接触ができなくなる
    for(int i=0; i<param->bodies.size(); i++) {
      for (int l=0; l<param->bodies[i]->numLinks(); l++) {
        calcIgnoreBoundingBox(param->reachabilityConstraints, param->bodies[i]->link(l), 3);
      }
    }

    // ScfrConstraintはロボットのみを対象とし、reachabilityConstraintsの該当リンクもロボットのリンクであることが前提. 必ずA_linkにすること
    // 操作物体はforward kinematics等のためparam->bodiesには含めるが、ここでのScfrConstraintには含めない. 接触リンクの質量を変更する等で対応すること
    std::set<cnoid::BodyPtr> active_bodies;
    for(int i=0;i<param->reachabilityConstraints.size();i++){
      if(param->reachabilityConstraints[i]->A_link()->body()) active_bodies.insert(param->reachabilityConstraints[i]->A_link()->body());
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > keepScfrConstraints;
    for(std::set<cnoid::BodyPtr>::iterator it=active_bodies.begin(); it != active_bodies.end(); it++) {
      std::shared_ptr<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint> keepScfrConstraint = std::make_shared<ik_constraint2_keep_collision_scfr::KeepCollisionScfrConstraint>();
      std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
      scfrConstraint->A_robot() = (*it);
      scfrConstraint->SCFRParam().eps = 0.2;
      scfrConstraint->SCFRParam().lpTolerance = 1e-7;
      scfrConstraint->maxCError() = 0.1;
      keepScfrConstraint->scfrConstraint() = scfrConstraint;
      keepScfrConstraint->minimumContactCount() = param->minimumContactCount;
      keepScfrConstraint->breakableSCFRParam().lpTolerance = 1e-7;
      for(int i=0;i<param->reachabilityConstraints.size();i++){
        if (param->reachabilityConstraints[i]->A_link()->body() == (*it)) keepScfrConstraint->keepCollisionConstraints().push_back(param->reachabilityConstraints[i]);
      }
      keepScfrConstraint->updateBounds();
      keepScfrConstraints.push_back(keepScfrConstraint);
    }
    return keepScfrConstraints;
  }
  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2_or_keep_collision::ORKeepCollisionConstraint> >& constraints,
                             const cnoid::LinkPtr link,
                             int level
                             ) {
    std::vector<cnoid::LinkPtr> targetLinks;
    calcLevelLinks(link, level, targetLinks);

    cnoid::BoundingBox bbx;
    bool found = false;
    for (int i=0; i<constraints.size(); i++) {
      for (int j=0; j<constraints[i]->collisionConstraints().size(); j++) {
        if (typeid(*(constraints[i]->collisionConstraints()[j]))!=typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) continue;
        if (constraints[i]->collisionConstraints()[j]->A_link() == link) continue;
        if (std::find(targetLinks.begin(), targetLinks.end(), constraints[i]->collisionConstraints()[j]->A_link()) != targetLinks.end()) {
          if (!found) {
            found = true;
            cnoid::SgMeshPtr mesh = convertToSgMesh(link->collisionShape());
            if(mesh && (mesh->numTriangles() != 0)) {
              mesh->updateBoundingBox();
              bbx = mesh->boundingBox();
            }
          }
          ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox;
          ignoreBoundingBox.parentLink = link;
          ignoreBoundingBox.localPose.translation() = bbx.center();
          ignoreBoundingBox.dimensions = bbx.max() - bbx.min();
          std::static_pointer_cast<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>(constraints[i]->collisionConstraints()[j])->ignoreBoundingBox().push_back(ignoreBoundingBox);
        }
      }
    }

  }
  void calcLevelLinks(const cnoid::LinkPtr inputLink,
                      int level, // input
                      std::vector<cnoid::LinkPtr>& targetLinks // output
                      ) {
    targetLinks.clear();
    targetLinks.push_back(inputLink);
    for (int iter=0; iter<level; iter++) {
      int prevLevelSize = targetLinks.size();
      for (int i=0; i<prevLevelSize; i++) {
        if ((targetLinks[i]->parent() != nullptr) && (std::find(targetLinks.begin(), targetLinks.end(), targetLinks[i]->parent()) == targetLinks.end())) targetLinks.push_back(targetLinks[i]->parent());
        cnoid::LinkPtr child = targetLinks[i]->child();
        while (child != nullptr) {
          if (std::find(targetLinks.begin(), targetLinks.end(), child) == targetLinks.end()) targetLinks.push_back(child);
          child = child->sibling();
        }
      }
    }
  }

  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh; model->getOrCreateVertices();
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    }else{
      //      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }
    model->setName(collisionshape->name());

    return model;
  }

  void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->vertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }
}
