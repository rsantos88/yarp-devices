// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_RAW_CONTROLBOARD_HPP__
#define __DEXTRA_RAW_CONTROLBOARD_HPP__

#include <mutex>

#include <yarp/dev/ControlBoardInterfaces.h>

#include "Synapse.hpp"

#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup DextraRawControlboard
 * @brief Contains roboticslab::DextraRawControlboard.
 */

/**
 * @ingroup DextraRawControlboard
 * @brief Base implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraRawControlboard : public yarp::dev::IAxisInfoRaw,
                              public yarp::dev::IControlLimitsRaw,
                              public yarp::dev::IControlModeRaw,
                              public yarp::dev::IEncodersTimedRaw,
                              public yarp::dev::IPositionControlRaw,
                              public yarp::dev::IPositionDirectRaw,
                              public yarp::dev::IVelocityControlRaw
{
public:

    DextraRawControlboard();
    virtual ~DextraRawControlboard() = default;

    void acquireSynapseHandle(Synapse * synapse);
    void destroySynapse();

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    virtual bool getAxisNameRaw(int axis, std::string & name) override;
    virtual bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

    virtual bool setLimitsRaw(int axis, double min, double max) override;
    virtual bool getLimitsRaw(int axis, double * min, double * max) override;
    virtual bool setVelLimitsRaw(int axis, double min, double max) override;
    virtual bool getVelLimitsRaw(int axis, double * min, double * max) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    virtual bool getControlModeRaw(int j, int * mode) override;
    virtual bool getControlModesRaw(int * modes) override;
    virtual bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    virtual bool setControlModeRaw(int j, int mode) override;
    virtual bool setControlModesRaw(int * modes) override;
    virtual bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getAxes(int * ax) override;
    virtual bool resetEncoderRaw(int j) override;
    virtual bool resetEncodersRaw() override;
    virtual bool setEncoderRaw(int j, double val) override;
    virtual bool setEncodersRaw(const double * vals) override;
    virtual bool getEncoderRaw(int j, double * v) override;
    virtual bool getEncodersRaw(double * encs) override;
    virtual bool getEncoderSpeedRaw(int j, double * sp) override;
    virtual bool getEncoderSpeedsRaw(double * spds) override;
    virtual bool getEncoderAccelerationRaw(int j, double * spds) override;
    virtual bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    virtual bool getEncoderTimedRaw(int j, double * encs, double * time) override;
    virtual bool getEncodersTimedRaw(double * encs, double * time) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //virtual bool getAxes(int * ax) override;
    virtual bool positionMoveRaw(int j, double ref) override;
    virtual bool positionMoveRaw(const double * refs) override;
    virtual bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    virtual bool relativeMoveRaw(int j, double delta) override;
    virtual bool relativeMoveRaw(const double * deltas) override;
    virtual bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    virtual bool checkMotionDoneRaw(int j, bool * flag) override;
    virtual bool checkMotionDoneRaw(bool * flag) override;
    virtual bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    virtual bool setRefSpeedRaw(int j, double sp) override;
    virtual bool setRefSpeedsRaw(const double * spds) override;
    virtual bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    virtual bool setRefAccelerationRaw(int j, double acc) override;
    virtual bool setRefAccelerationsRaw(const double * accs) override;
    virtual bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    virtual bool getRefSpeedRaw(int j, double * ref) override;
    virtual bool getRefSpeedsRaw(double * spds) override;
    virtual bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    virtual bool getRefAccelerationRaw(int j, double * acc) override;
    virtual bool getRefAccelerationsRaw(double * accs) override;
    virtual bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    virtual bool stopRaw(int j) override;
    virtual bool stopRaw() override;
    virtual bool stopRaw(int n_joint, const int * joints) override;
    virtual bool getTargetPositionRaw(int joint, double * ref) override;
    virtual bool getTargetPositionsRaw(double * refs) override;
    virtual bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    virtual bool setPositionRaw(int j, double ref) override;
    virtual bool setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual bool setPositionsRaw(const double *refs) override;

    //  --------- IVelocityControlRaw declarations and stub implementations. ---------

    // re-implemented methods have been omitted
    virtual bool velocityMoveRaw(int j, double sp) override { return false; }
    virtual bool velocityMoveRaw(const double * sp) override { return false; }
    virtual bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override { return false; }
    virtual bool getRefVelocityRaw(int joint, double * vel) override { return false; }
    virtual bool getRefVelocitiesRaw(double * vels) override { return false; }
    virtual bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override { return false; }

protected:

    Synapse * synapse;

private:

    double getSetpoint(int j);
    void getSetpoints(Synapse::Setpoints & setpoints);
    void setSetpoint(int j, Synapse::setpoint_t setpoint);
    void setSetpoints(const Synapse::Setpoints & setpoints);

    Synapse::Setpoints setpoints;
    mutable std::mutex setpointMutex;
};

} // namespace roboticslab

#endif // __DEXTRA_RAW_CONTROLBOARD_HPP__
