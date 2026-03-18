#pragma once

#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <tf2/LinearMath/Quaternion.h>

namespace wam_node
{
template <typename T1, typename T2, typename OutputType>
class Multiplier : public barrett::systems::System,
                   public barrett::systems::SingleOutput<OutputType>
{
  public:
    Multiplier(std::string sysName = "Multiplier")
        :barrett::systems::System(sysName),
         barrett::systems::SingleOutput<OutputType>(this),
         input1(this),
         input2(this)                                   {}
    virtual ~Multiplier()                               { mandatoryCleanUp(); }

  private:
    virtual void        operate()
                        {
                            data = input1.getValue() * input2.getValue();
                            this->outputValue->setData(&data);
                        }

  public:
    Input<T1> input1;
    Input<T2> input2;

  private:
    OutputType data;

  private:
    DISALLOW_COPY_AND_ASSIGN(Multiplier);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ToQuaternion
    : public barrett::systems::SingleIO<barrett::math::Vector<3>::type,
                                        Eigen::Quaterniond>
{
  public:
    ToQuaternion(std::string sysName = "ToQuaternion")
        :barrett::systems::SingleIO<barrett::math::Vector<3>::type,
                                    Eigen::Quaterniond>(sysName)
    {
    }
    virtual ~ToQuaternion()                             { mandatoryCleanUp(); }

  private:
    virtual void        operate()
                        {
                            const auto&     inputRPY = input.getValue();
                            tf2::Quaternion q;
                            q.setRPY(inputRPY[0], inputRPY[1], inputRPY[2]);
                            outputQuat.x() = q.getX();
                            outputQuat.y() = q.getY();
                            outputQuat.z() = q.getZ();
                            outputQuat.w() = q.getW();
                            this->outputValue->setData(&outputQuat);
                        }

  private:
    Eigen::Quaterniond  outputQuat;

  private:
    DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
