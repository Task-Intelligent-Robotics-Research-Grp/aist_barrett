#ifndef WAM_NODE_SRC_CUSTOM_SYSTEMS_H_
#define WAM_NODE_SRC_CUSTOM_SYSTEMS_H_
#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include "tf2/LinearMath/Quaternion.h"
using namespace barrett;
template <typename T1, typename T2, typename OutputType>
class Multiplier : public systems::System,
                   public systems::SingleOutput<OutputType> {
 public:
  Input<T1> input1;

 public:
  Input<T2> input2;

 public:
  Multiplier(std::string sysName = "Multiplier")
      : systems::System(sysName),
        systems::SingleOutput<OutputType>(this),
        input1(this),
        input2(this) {}
  virtual ~Multiplier() { mandatoryCleanUp(); }

 protected:
  OutputType data;
  virtual void operate() {
    data = input1.getValue() * input2.getValue();
    this->outputValue->setData(&data);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Multiplier);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ToQuaternion
    : public systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond> {
 public:
  Eigen::Quaterniond outputQuat;

 public:
  ToQuaternion(std::string sysName = "ToQuaternion")
      : systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName) {}
  virtual ~ToQuaternion() { mandatoryCleanUp(); }

 protected:
  tf2::Quaternion q;
  virtual void operate() {
    const math::Vector<3>::type &inputRPY = input.getValue();
    q.setRPY(inputRPY[0], inputRPY[1], inputRPY[2]);
    outputQuat.x() = q.getX();
    outputQuat.y() = q.getY();
    outputQuat.z() = q.getZ();
    outputQuat.w() = q.getW();
    this->outputValue->setData(&outputQuat);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // WAM_NODE_SRC_CUSTOM_SYSTEMS_H_