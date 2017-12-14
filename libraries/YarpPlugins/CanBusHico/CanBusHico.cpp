// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <cstring>

#include <ColorDebug.hpp>

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::sendRaw(uint32_t id, uint16_t len, uint8_t * msgData)
{
    struct can_msg msg;
    std::memset(&msg,0,sizeof(struct can_msg));
    msg.ff = FF_NORMAL;
    msg.id = id;
    msg.dlc = len;
    std::memcpy(msg.data, msgData, len*sizeof(uint8_t));

    canBusReady.wait();
    if ( ::write(fileDescriptor,&msg,sizeof(struct can_msg)) == -1)
    {
        canBusReady.post();
        CD_ERROR("%s.\n", std::strerror(errno))
        return false;
    }
    canBusReady.post();

    return true;
}

// -----------------------------------------------------------------------------

int roboticslab::CanBusHico::read_timeout(struct can_msg *buf, unsigned int timeout)
{
    fd_set fds;
    struct timeval tv;
    int sec,ret;
    FD_ZERO(&fds);

    sec=timeout/1000;
    tv.tv_sec=sec;
    tv.tv_usec=(timeout-(sec*1000))*1000;

    FD_SET(fileDescriptor,&fds);

    //-- select() returns the number of ready descriptors, or -1 for errors.
    ret=::select(fileDescriptor+1,&fds,0,0,&tv);
    if(ret==0)
    {
        //-- Commenting select() timeout as way too verbose, happens all the time.
        //CD_DEBUG("select() timeout.\n");
        return 0;  // Return 0 on select timeout.
    }
    else if (ret<0)
    {
        //CD_ERROR("select() error: %s.\n", strerror(errno));
        return ret;   // Return <0 on select error.
    }

    assert(FD_ISSET(fileDescriptor,&fds));

    canBusReady.wait();
    //-- read() returns the number read, -1 for errors or 0 for EOF.
    ret=::read(fileDescriptor,buf,sizeof(struct can_msg));
    canBusReady.post();

    if (ret<0)
    {
        CD_ERROR("read() error: %s.\n", std::strerror(errno));
    }

    return ret;  //-- If gets to here, return whatever read() returned.
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::setFdMode(bool requestedBlocking)
{
    bool currentlyBlocking = (fcntlFlags & O_NONBLOCK) != O_NONBLOCK;

    if (currentlyBlocking != requestedBlocking)
    {
        int flag = requestedBlocking ? ~O_NONBLOCK : O_NONBLOCK;
        return ::fcntl(fileDescriptor, F_SETFL, fcntlFlags & flag) != -1;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::setDelay()
{
    fd_set fds;
    struct timeval tv;

    tv.tv_sec = DELAY * 1000;
    tv.tv_usec = 0;

    FD_ZERO(&fds);
    FD_SET(fileDescriptor, &fds);

    //-- select() returns the number of ready descriptors, or -1 for errors.
    int ret = ::select(fileDescriptor + 1, &fds, 0, 0, &tv);

    if (ret <= 0)
    {
        //-- No CD as select() timeout is way too verbose, happens all the time.
        // Return 0 on select timeout, <0 on select error.
        return false;
    }

    assert(FD_ISSET(fileDescriptor, &fds));

    return true;
}

// -----------------------------------------------------------------------------
