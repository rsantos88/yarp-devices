// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IPositionControl related --------------------------------

bool roboticslab::AmorControlboard::getAxes(int *ax)
{
    CD_DEBUG("\n");
    *ax = AMOR_NUM_JOINTS;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    handleReady.wait();

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    positions[j] = toRad(ref);

    bool ret;

    handleReady.wait();
    ret = amor_set_positions(handle, positions) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const double *refs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] = toRad(refs[j]);
    }

    bool ret;

    handleReady.wait();
    ret = amor_set_positions(handle, positions) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n", j, delta);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    handleReady.wait();

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    positions[j] += toRad(delta);

    bool ret;

    handleReady.wait();
    ret = amor_set_positions(handle, positions) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const double *deltas)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] += toRad(deltas[j]);
    }

    bool ret;

    handleReady.wait();
    ret = amor_set_positions(handle, positions) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(int j, bool *flag)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(bool *flag)
{
    CD_DEBUG("\n");

    amor_movement_status status;

    handleReady.wait();

    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeed(int j, double sp)
{
    CD_DEBUG("(%d, %f).\n", j, sp);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 speeds;

    handleReady.wait();

    if (amor_get_ref_speeds(handle, &speeds) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    speeds[j] = toRad(sp);

    bool ret;

    handleReady.wait();
    ret = amor_set_ref_speeds(handle, speeds) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const double *spds)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 speeds;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        speeds[j] = toRad(spds[j]);
    }

    bool ret;

    handleReady.wait();
    ret = amor_set_ref_speeds(handle, speeds) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAcceleration(int j, double acc)
{
    CD_DEBUG("(%d, %f).\n", j, acc);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 accelerations;

    handleReady.wait();

    if (amor_get_ref_accelerations(handle, &accelerations) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    accelerations[j] = toRad(acc);

    bool ret;

    handleReady.wait();
    ret = amor_set_ref_accelerations(handle, accelerations) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const double *accs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 accelerations;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        accelerations[j] = toRad(accs[j]);
    }

    bool ret;

    handleReady.wait();
    ret = amor_set_ref_accelerations(handle, accelerations) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeed(int j, double *ref)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    real speed;

    handleReady.wait();

    if (amor_get_ref_speed(handle, j, &speed) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    *ref = toDeg(speed);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(double *spds)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 speeds;

    handleReady.wait();

    if (amor_get_ref_speeds(handle, &speeds) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        spds[j] = toDeg(speeds[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAcceleration(int j, double *acc)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    real acceleration;

    handleReady.wait();

    if (amor_get_ref_acceleration(handle, j, &acceleration) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    *acc = toDeg(acceleration);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(double *accs)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 accelerations;

    handleReady.wait();

    if (amor_get_ref_accelerations(handle, &accelerations) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    handleReady.post();

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        accs[j] = toDeg(accelerations[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(int j)
{
    CD_WARNING("Selective stop not available, stopping all joints at once (%d).\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop()
{
    CD_DEBUG("\n");

    bool ret;

    handleReady.wait();
    ret = amor_controlled_stop(handle) == AMOR_SUCCESS;
    handleReady.post();

    return ret;
}

// -----------------------------------------------------------------------------
