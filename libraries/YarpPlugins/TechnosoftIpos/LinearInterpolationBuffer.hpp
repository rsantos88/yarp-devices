// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LINEAR_INTERPOLATION_BUFFER_HPP__
#define __LINEAR_INTERPOLATION_BUFFER_HPP__

#include <cstdint>

#include <mutex>
#include <string>

#include <yarp/os/Searchable.h>

#include "StateVariables.hpp"

#define DEFAULT_PERIOD_MS 50
#define DEFAULT_BUFFER_SIZE 1
#define DEFAULT_MODE "pt"

// https://github.com/roboticslab-uc3m/yarp-devices/issues/198#issuecomment-487279910
#define PT_BUFFER_MAX_SIZE 285
#define PVT_BUFFER_MAX_SIZE 222
#define PT_PVT_BUFFER_LOW_SIGNAL 15 // max: 15

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 */
class LinearInterpolationBuffer
{
public:
    LinearInterpolationBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);
    virtual ~LinearInterpolationBuffer() {}
    void resetIntegrityCounter();
    virtual void setInitialReference(double target);
    virtual std::string getType() const = 0;
    void updateTarget(double target);
    double getLastTarget() const;
    int getPeriod() const;
    void setPeriod(int periodMs);
    int getBufferSize() const;
    void setBufferSize(int bufferSize);
    virtual int getSubMode() const = 0;
    virtual std::uint64_t makeDataRecord() = 0;
    LinearInterpolationBuffer * cloneTo(const std::string & type);
    static LinearInterpolationBuffer * createBuffer(const yarp::os::Searchable & config, const StateVariables & vars);

protected:
    int periodMs;
    int bufferSize;
    double factor;
    double maxVel;
    double lastSentTarget;
    double lastReceivedTarget;
    int integrityCounter;
    mutable std::mutex mtx;
};

/**
 * @ingroup TechnosoftIpos
 */
class PtBuffer : public LinearInterpolationBuffer
{
public:
    PtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);
    virtual std::string getType() const override;
    virtual int getSubMode() const override;
    virtual std::uint64_t makeDataRecord() override;

private:
    double maxDistance;
};

/**
 * @ingroup TechnosoftIpos
 */
class PvtBuffer : public LinearInterpolationBuffer
{
public:
    PvtBuffer(int _periodMs, int _bufferSize, double _factor, double _maxVel);
    virtual void setInitialReference(double target) override;
    virtual std::string getType() const override;
    virtual int getSubMode() const override;
    virtual std::uint64_t makeDataRecord() override;

private:
    double previousTarget;
    bool isFirstPoint;
};

} // namespace roboticslab

#endif // __LINEAR_INTERPOLATION_BUFFER_HPP__
