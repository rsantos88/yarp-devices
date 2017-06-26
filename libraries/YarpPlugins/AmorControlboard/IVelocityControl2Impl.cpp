// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IVelocityControl2 related ----------------------------------------

bool roboticslab::AmorControlboard::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    handleReady.wait();

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    for (int j = 0; j < n_joint; j++)
    {
        velocities[joints[j]] = toRad(spds[j]);
    }

    bool ret;

    handleReady.wait();
    ret = amor_set_velocities(handle, velocities) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocity(const int joint, double *vel)
{
    CD_DEBUG("(%d)\n", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    handleReady.wait();

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    *vel = toDeg(velocities[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(double *vels)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 velocities;

    handleReady.wait();

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        vels[j] = toDeg(velocities[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 velocities;

    handleReady.wait();

    if (amor_get_req_velocities(handle, &velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    for (int j = 0; j < n_joint; j++)
    {
        vels[j] = toDeg(velocities[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelPid(int j, const yarp::dev::Pid &pid)
{
    CD_ERROR("Not available (%d).\n", j);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelPids(const yarp::dev::Pid *pids)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelPid(int j, yarp::dev::Pid *pid)
{
    CD_ERROR("Not available (%d).\n", j);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelPids(yarp::dev::Pid *pids)
{
    CD_ERROR("Not available.\n");
    return false;
}

// -----------------------------------------------------------------------------
