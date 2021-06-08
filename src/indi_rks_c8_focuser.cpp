#include <cstring>
#include <unistd.h>

#include "libindi/indicom.h"

#include "config.h"
#include "indi_rks_c8_focuser.h"

// We declare an auto pointer to RKSC8Focuser.
static std::unique_ptr<RKSC8Focuser> mydriver(new RKSC8Focuser());

RKSC8Focuser::RKSC8Focuser()
    : _reader(0),
      _writer(0),
      _comms(0),
      _microsteps(ELS::MS_X64),
      _maxPos(0),
      _position(0)
{
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);

    // Here we tell the base Focuser class what types of connections we can support
    setSupportedConnections(CONNECTION_SERIAL);

    // And here we tell the base class about our focuser's capabilities.
    SetCapability(FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_ABS_MOVE);
}

const char *RKSC8Focuser::getDefaultName()
{
    return "RKS C8 Focuser";
}

bool RKSC8Focuser::initProperties()
{
    // initialize the parent's properties first
    INDI::Focuser::initProperties();

    // TODO: Add any custom properties you need here.
    FocusAbsPosN[0].min = 0.0;
    FocusAbsPosN[0].max = 2048000.0;
    FocusAbsPosN[0].value = 1024000.0;
    FocusAbsPosN[0].step = 500.0;

    FocusMaxPosN[0].min = 0.0;
    FocusMaxPosN[0].max = 2048000.0;
    FocusMaxPosN[0].value = 2048000.0;
    FocusMaxPosN[0].step = 500.0;

    // Enable/Disable
    IUFillSwitch(&EnableS[0], "ENABLE", "Enable", ISS_OFF);
    IUFillSwitch(&EnableS[1], "DISABLE", "Disable", ISS_OFF);
    IUFillSwitchVector(&EnableSP, EnableS, 2, getDeviceName(),
                       "Motor Enable", "", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // Speed
    IUFillSwitch(&SpeedS[0], "NORMAL", "Normal", ISS_OFF);
    IUFillSwitch(&SpeedS[1], "THREEX", "X3", ISS_OFF);
    IUFillSwitchVector(&SpeedSP, SpeedS, 2, getDeviceName(),
                       "Speed", "", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // Microsteps
    IUFillSwitch(&MicrostepS[0], "MS_X8", "X8", ISS_OFF);
    IUFillSwitch(&MicrostepS[1], "MS_X16", "X16", ISS_OFF);
    IUFillSwitch(&MicrostepS[2], "MS_X32", "X32", ISS_OFF);
    IUFillSwitch(&MicrostepS[3], "MS_X64", "X64", ISS_OFF);
    IUFillSwitchVector(&MicrostepSP, MicrostepS, 4, getDeviceName(),
                       "Microsteps", "", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // Zero
    IUFillSwitch(&ZeroS[0], "ZERO", "Zero", ISS_OFF);
    IUFillSwitchVector(&ZeroSP, ZeroS, 1, getDeviceName(),
                       "Zero Position", "", OPTIONS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    addAuxControls();

    return true;
}

void RKSC8Focuser::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);

    // TODO: Call define* for any custom properties.
}

bool RKSC8Focuser::updateProperties()
{
    INDI::Focuser::updateProperties();
    LOG_INFO("update properties");

    if (isConnected())
    {
        defineProperty(&EnableSP);
        defineProperty(&SpeedSP);
        defineProperty(&MicrostepSP);
        defineProperty(&ZeroSP);
    }
    else
    {
        deleteProperty(EnableSP.name);
        deleteProperty(SpeedSP.name);
        deleteProperty(MicrostepSP.name);
        deleteProperty(ZeroSP.name);
    }

    return true;
}

bool RKSC8Focuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // TODO: Check to see if this is for any of my custom Number properties.
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool RKSC8Focuser::ISNewSwitch(const char *dev,
                               const char *name,
                               ISState *states,
                               char *names[],
                               int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Enable/Disable
        if (strcmp(EnableSP.name, name) == 0)
        {
            bool currentEnabled = IUFindOnSwitchIndex(&EnableSP) == 0;

            IUUpdateSwitch(&EnableSP, states, names, n);

            bool targetEnabled = IUFindOnSwitchIndex(&EnableSP) == 0;

            if (currentEnabled == targetEnabled)
            {
                EnableSP.s = IPS_OK;
                IDSetSwitch(&EnableSP, nullptr);
                return true;
            }

            if (_comms != 0)
            {
                _comms->enableMotor(targetEnabled);
            }
        }

        // Speed
        if (strcmp(SpeedSP.name, name) == 0)
        {
            bool currentSpeed = IUFindOnSwitchIndex(&SpeedSP) == 0;

            IUUpdateSwitch(&SpeedSP, states, names, n);

            bool targetSpeed = IUFindOnSwitchIndex(&SpeedSP) == 0;

            if (currentSpeed == targetSpeed)
            {
                SpeedSP.s = IPS_OK;
                IDSetSwitch(&SpeedSP, nullptr);
                return true;
            }

            if (_comms != 0)
            {
                _comms->setSpeed((ELS::FocusSpeed)(currentSpeed + 1));
            }
        }

        // Microsteps
        if (strcmp(MicrostepSP.name, name) == 0)
        {
            int currentMS = IUFindOnSwitchIndex(&MicrostepSP);

            IUUpdateSwitch(&MicrostepSP, states, names, n);

            int targetMS = IUFindOnSwitchIndex(&MicrostepSP);

            if (currentMS == targetMS)
            {
                MicrostepSP.s = IPS_OK;
                IDSetSwitch(&MicrostepSP, nullptr);
                return true;
            }

            if (_comms != 0)
            {
                _comms->setMicrostep((ELS::Microsteps)(targetMS + 1));
            }
        }

        // Zero
        if (strcmp(ZeroSP.name, name) == 0)
        {
            int currentZero = IUFindOnSwitchIndex(&ZeroSP);

            IUUpdateSwitch(&ZeroSP, states, names, n);

            int targetZero = IUFindOnSwitchIndex(&ZeroSP);

            if (currentZero == targetZero)
            {
                MicrostepSP.s = IPS_OK;
                IDSetSwitch(&MicrostepSP, nullptr);
                return true;
            }

            if (_comms != 0)
            {
                _comms->zero();
            }
        }
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool RKSC8Focuser::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // TODO: Check to see if this is for any of my custom Text properties.
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::Focuser::ISNewText(dev, name, texts, names, n);
}

bool RKSC8Focuser::ISSnoopDevice(XMLEle *root)
{
    // TODO: Check to see if this is for any of my custom Snoops. Fo shizzle.

    return INDI::Focuser::ISSnoopDevice(root);
}

bool RKSC8Focuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);

    // TODO: Call IUSaveConfig* for any custom properties I want to save.

    return true;
}

bool RKSC8Focuser::Connect()
{
    bool rc = INDI::Focuser::Connect();

    return rc;
}

bool RKSC8Focuser::Disconnect()
{
    _comms->enableMotor(false);

    _reader->shutdown();

    bool rc = INDI::Focuser::Disconnect();

    return rc;
}

bool RKSC8Focuser::Handshake()
{
    if (isSimulation())
    {
        LOGF_INFO("Connected successfully to simulated %s.", getDeviceName());
        return true;
    }

    _writer = new HCWriter(this);
    _comms = new ELS::HostComms(_writer, this);
    _reader = new HCReader(PortFD, _comms, this);
    _reader->start();

    _comms->getMaxPos();
    _comms->getPos();
    _comms->getMicrostep();
    _comms->getSpeed();
    _comms->getMotorEnabled();

    _comms->enableMotor(true);

    return true;
}

IPState RKSC8Focuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    // NOTE: This is needed if we don't specify FOCUSER_CAN_ABS_MOVE
    // TODO: Actual code to move the focuser. You can use IEAddTimer to do a
    // callback after "duration" to stop your focuser.
    LOGF_INFO("MoveFocuser: %d %d %d", dir, speed, duration);
    return IPS_OK;
}

IPState RKSC8Focuser::MoveAbsFocuser(uint32_t targetTicks)
{
    // NOTE: This is needed if we do specify FOCUSER_CAN_ABS_MOVE
    // TODO: Actual code to move the focuser.
    LOGF_INFO("MoveAbsFocuser: %d", targetTicks);
    if (_comms != 0)
    {
        _comms->focusAbs(targetTicks);
    }
    return IPS_OK;
}

IPState RKSC8Focuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // NOTE: This is needed if we do specify FOCUSER_CAN_REL_MOVE
    // TODO: Actual code to move the focuser.
    switch (dir)
    {
    case FOCUS_INWARD:
        LOGF_INFO("MoveRelFocuser IN %d", ticks);
        if (_comms != 0)
        {
            _comms->focusRel(ELS::FD_FOCUS_INWARD, ticks);
        }
        break;
    case FOCUS_OUTWARD:
        LOGF_INFO("MoveRelFocuser OUT %d", ticks);
        if (_comms != 0)
        {
            _comms->focusRel(ELS::FD_FOCUS_OUTWARD, ticks);
        }
        break;
    }
    return IPS_OK;
}

bool RKSC8Focuser::AbortFocuser()
{
    // NOTE: This is needed if we do specify FOCUSER_CAN_ABORT
    // TODO: Actual code to stop the focuser.
    LOG_INFO("AbortFocuser");
    if (_comms != 0)
    {
        _comms->focusAbort();
    }
    return true;
}

void RKSC8Focuser::movingRel(ELS::FocusDirection dir, uint32_t steps)
{
    LOGF_INFO("Moving relative %s %u steps",
              (dir == ELS::FD_FOCUS_INWARD) ? "IN" : "OUT", steps);
    FocusAbsPosNP.s = IPS_BUSY;
    FocusRelPosNP.s = IPS_BUSY;
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusAbsPosNP, nullptr);
}

void RKSC8Focuser::movingAbs(uint32_t fromPosition, uint32_t toPosition)
{
    LOGF_INFO("Moving absolute from %u to %u", fromPosition, toPosition);
    FocusAbsPosNP.s = IPS_BUSY;
    FocusRelPosNP.s = IPS_BUSY;
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusAbsPosNP, nullptr);
}

void RKSC8Focuser::stopped(uint32_t position)
{
    LOGF_INFO("Stopped at %u", position);
    _position = position;
    updateAbsPosition(_position);
    FocusRelPosNP.s = IPS_OK;
    IDSetNumber(&FocusRelPosNP, nullptr);
}

void RKSC8Focuser::motorEnabled(bool isEnabled)
{
    LOGF_INFO("Motor is %s", isEnabled ? "ENABLED" : "DISABLED");

    IUResetSwitch(&EnableSP);

    if (isEnabled)
    {
        EnableS[0].s = ISS_ON;
    }
    else
    {
        EnableS[1].s = ISS_ON;
    }

    EnableSP.s = IPS_OK;
    IDSetSwitch(&EnableSP, nullptr);
}

void RKSC8Focuser::zeroed()
{
    LOG_INFO("Zeroed");

    IUResetSwitch(&ZeroSP);
    ZeroS[0].s = ISS_OFF;
    ZeroSP.s = IPS_OK;
    IDSetSwitch(&ZeroSP, nullptr);
}

void RKSC8Focuser::position(uint32_t position)
{
    LOGF_INFO("Position is now %u", position);
    _position = position;
    updateAbsPosition(_position);
}

void RKSC8Focuser::microsteps(ELS::Microsteps ms)
{
    const char *s = 0;

    _microsteps = ms;

    IUResetSwitch(&MicrostepSP);

    switch (_microsteps)
    {
    case ELS::MS_X64:
        s = "X64";
        MicrostepS[3].s = ISS_ON;
        break;
    case ELS::MS_X32:
        s = "X32";
        MicrostepS[2].s = ISS_ON;
        break;
    case ELS::MS_X16:
        s = "X16";
        MicrostepS[1].s = ISS_ON;
        break;
    case ELS::MS_X8:
        s = "X8";
        MicrostepS[0].s = ISS_ON;
        break;
    }

    MicrostepSP.s = IPS_OK;
    IDSetSwitch(&MicrostepSP, nullptr);

    LOGF_INFO("Microsteps is now %s", s);
}

void RKSC8Focuser::maxPos(uint32_t position)
{
    LOGF_INFO("Max position is %u", position);
    _maxPos = position;
    FocusMaxPosN[0].value = _maxPos;
    IDSetNumber(&FocusMaxPosNP, nullptr);
    IUUpdateMinMax(&FocusAbsPosNP);
}

void RKSC8Focuser::speed(ELS::FocusSpeed speed)
{
    const char *s = 0;

    _speed = speed;

    IUResetSwitch(&SpeedSP);

    switch (speed)
    {
    case ELS::FS_NORMAL:
        s = "Normal";
        SpeedS[0].s = ISS_ON;
        break;
    case ELS::FS_X3:
        s = "X3";
        SpeedS[1].s = ISS_ON;
        break;
    }

    SpeedSP.s = IPS_OK;
    IDSetSwitch(&SpeedSP, nullptr);

    LOGF_INFO("Speed is now %s", s);
}

void RKSC8Focuser::log(const char *fmt, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    LOG_INFO(buffer);
}

void RKSC8Focuser::updateAbsPosition(uint32_t position)
{
    FocusAbsPosN[0].value = position;
    FocusAbsPosNP.s = IPS_OK;
    FocusRelPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, nullptr);
}

//
// HCWriter
//

RKSC8Focuser::HCWriter::HCWriter(RKSC8Focuser *parent)
    : _parent(parent)
{
}

bool RKSC8Focuser::HCWriter::writeLine(const char *line)
{
    int len = sprintf(_buffer, "%s\r\n", line);

    return (write(_parent->PortFD, _buffer, len) == len);
}

void RKSC8Focuser::HCWriter::close()
{
    // Do nothing because INDI is managing the
    // file handle
}

//
// HCReader
//

RKSC8Focuser::HCReader::HCReader(int fd,
                                 ELS::HostComms *comms,
                                 RKSC8Focuser *parent)
    : _fd(fd),
      _comms(comms),
      _parent(parent),
      _bufLen(0)
{
    _buffer[g_bufSize] = 0;
}

bool RKSC8Focuser::HCReader::start()
{

    if (pipe(_pipefd) != 0)
    {
        _parent->log("Failed to allocate pipe");
        return false;
    }

    return (pthread_create(&_readThreadHandle,
                           NULL,
                           readThreadRedirect,
                           this) == 0);
}

bool RKSC8Focuser::HCReader::shutdown()
{
    close(_pipefd[1]);

    if (pthread_join(_readThreadHandle, NULL) != 0)
    {
        return false;
    }

    return true;
}

void RKSC8Focuser::HCReader::readThread()
{
    fd_set fds;
    ssize_t bytesRead = 0;

    FD_ZERO(&fds);
    FD_SET(_fd, &fds);
    FD_SET(_pipefd[0], &fds);

    int max_fd = _fd;
    if (_pipefd[0] > max_fd)
    {
        max_fd = _pipefd[0];
    }
    max_fd++;

    while (select(max_fd, &fds, NULL, NULL, NULL) != -1)
    {
        if (FD_ISSET(_pipefd[0], &fds))
        {
            return;
        }

        if (FD_ISSET(_fd, &fds))
        {
            bytesRead = read(_fd,
                             _buffer + _bufLen,
                             g_bufSize - _bufLen);
            _bufLen += bytesRead;
            _buffer[_bufLen] = 0;

            char *currentSegment = _buffer;
            int bytesLeft = _bufLen;
            while (bytesLeft > 0)
            {
                // Search for LF or CR/LF pair
                int crIdx = -1;
                int lfIdx = -1;
                for (int i = 0; i < bytesLeft; i++)
                {
                    if (currentSegment[i] == '\r')
                    {
                        crIdx = i;
                    }
                    if (currentSegment[i] == '\n')
                    {
                        lfIdx = i;
                        break;
                    }
                }

                // Check if found
                if (lfIdx != -1)
                {
                    int cmdLen = lfIdx;
                    if ((crIdx != -1) && ((crIdx + 1) == lfIdx))
                    {
                        cmdLen -= 1;
                    }
                    currentSegment[cmdLen] = 0;

                    if (_comms != 0)
                    {
                        _comms->processLine(currentSegment);
                    }

                    currentSegment += lfIdx + 1;
                    bytesLeft -= lfIdx + 1;
                    _bufLen -= lfIdx + 1;
                }
                else
                {
                    // Were we searching from the beginning?
                    if (currentSegment == _buffer)
                    {
                        // Is buffer full?
                        if (_bufLen == g_bufSize)
                        {
                            _bufLen = 0;
                        }
                    }
                    else
                    {
                        char *tmp = _buffer;
                        while (true)
                        {
                            *tmp = *currentSegment;

                            if (*tmp == 0)
                            {
                                break;
                            }

                            tmp++;
                            currentSegment++;
                        }
                        _bufLen = bytesLeft;
                    }

                    bytesLeft = 0;
                }
            }
        }

        FD_SET(_fd, &fds);
        FD_SET(_pipefd[0], &fds);
    }
}

/* static */ void *RKSC8Focuser::HCReader::readThreadRedirect(void *obj)
{
    ((HCReader *)(obj))->readThread();

    return 0;
}
