#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>

/**
 * @brief Write per-joint armature (reflected rotor inertia) into a Pinocchio model.
 *
 * A URDF cannot express armature, so a model built from it has none.
 *
 * @param[in,out] model     The model to modify, untouched on failure
 * @param[in]     joints    Joint names, in the order of @p armature
 * @param[in]     armature  Armature per joint [kg m^2]; empty leaves the model untouched
 * @param[out]    error     Reason for the failure, untouched on success
 * @return true on success
 */
inline bool set_joint_armature(
  pinocchio::Model & model,
  const std::vector<std::string> & joints,
  const std::vector<double> & armature,
  std::string & error) {
  if (armature.empty()) {
    return true;
  }
  if (armature.size() != joints.size()) {
    error = "armature has " + std::to_string(armature.size()) + " entries but there are " +
      std::to_string(joints.size()) + " joints";
    return false;
  }

  Eigen::VectorXd model_armature = model.armature;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (!model.existJointName(joints[i])) {
      error = "joint '" + joints[i] + "' is not in the model";
      return false;
    }
    if (!std::isfinite(armature[i]) || armature[i] < 0.0) {
      error = "armature of joint '" + joints[i] + "' is negative or not finite";
      return false;
    }
    const auto & joint = model.joints[model.getJointId(joints[i])];
    model_armature.segment(joint.idx_v(), joint.nv()).setConstant(armature[i]);
  }
  model.armature = model_armature;
  return true;
}
