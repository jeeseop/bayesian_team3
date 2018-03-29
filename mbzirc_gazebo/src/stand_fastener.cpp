#include <mbzirc_gazebo/stand_fastener.h>

using namespace gazebo;

StandFastener::StandFastener() {
  this->mIsFastened = false;
}

void StandFastener::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->mModel = _model;
  this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&StandFastener::OnUpdate, this, _1));
}

void StandFastener::OnUpdate(const common::UpdateInfo& _info) {
  if (!this->mIsFastened) {
    std::string targetName = "disc" + this->mModel->GetName().substr(5);
    physics::WorldPtr world = this->mModel->GetWorld();
    physics::ModelPtr targetModel = world->GetModel(targetName);
    if (targetModel != NULL) {
      physics::LinkPtr standLink = this->mModel->GetLink("link_1");
      this->mModel->CreateJoint("fastener_joint", "fixed", standLink, targetModel->GetLink("link_0"));
      this->mIsFastened = true;
    }
  }
}
