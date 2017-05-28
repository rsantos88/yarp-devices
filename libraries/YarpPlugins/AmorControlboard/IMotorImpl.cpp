// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- IMotor related ------------------------------------

bool roboticslab::AmorControlboard::getNumberOfMotors(int *num)
{
    CD_DEBUG("\n");
    *num = AMOR_NUM_JOINTS;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTemperature(int m, double *val)
{
    CD_DEBUG("(%d)\n", m);

    if (!indexWithinRange(m))
    {
        return false;
    }

    AMOR_VECTOR7 temperatures;

    if (amor_get_temperatures(handle, &temperatures) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve temperature info.\n");
        return false;
    }

    *val = temperatures[m];

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTemperatures(double *vals)
{
    CD_DEBUG("\n");

    AMOR_VECTOR7 temperatures;

    if (amor_get_temperatures(handle, &temperatures) != AMOR_SUCCESS)
    {
        CD_ERROR("Could not retrieve temperature info.\n");
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        vals[j] = temperatures[j];
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTemperatureLimit(int m, double *temp)
{
    CD_ERROR("Not available (%d).\n", m);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setTemperatureLimit(int m, const double temp)
{
    CD_ERROR("Not available (%d, %f).\n", m, temp);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getGearboxRatio(int m, double *val)
{
    CD_ERROR("Not available (%d).\n", m);
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setGearboxRatio(int m, const double val)
{
    CD_ERROR("Not available (%d, %f).\n", m, val);
    return false;
}

// -----------------------------------------------------------------------------
