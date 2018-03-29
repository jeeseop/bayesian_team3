#ifndef STAND_FASTENER_H
#define STAND_FASTENER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
  class StandFastener : public ModelPlugin {
  public:
    StandFastener();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo& _info);

  private:
    physics::ModelPtr mModel;
    event::ConnectionPtr mUpdateConnection;
    bool mIsFastened;
  };

  GZ_REGISTER_MODEL_PLUGIN(StandFastener)
}

#endif
