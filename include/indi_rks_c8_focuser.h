#pragma once

#include <pthread.h>

#include "libindi/indifocuser.h"
#include "CommsWriter.hpp"
#include "HostComms.hpp"
#include "HostCommsListener.hpp"

class RKSC8Focuser : public INDI::Focuser, public ELS::HostCommsListener
{
public:
    RKSC8Focuser();
    virtual ~RKSC8Focuser() = default;

    virtual const char *getDefaultName() override;

    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewNumber(const char *dev,
                             const char *name,
                             double values[],
                             char *names[],
                             int n) override;
    virtual bool ISNewSwitch(const char *dev,
                             const char *name,
                             ISState *states,
                             char *names[], int n) override;
    virtual bool ISNewText(const char *dev,
                           const char *name,
                           char *texts[],
                           char *names[],
                           int n) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;

    // HostCommsListener
    virtual void movingRel(ELS::FocusDirection dir,
                           uint32_t steps) override;
    virtual void movingAbs(uint32_t fromPosition,
                           uint32_t toPosition) override;
    virtual void stopped(uint32_t position) override;
    virtual void motorEnabled(bool isEnabled) override;
    virtual void zeroed() override;
    virtual void position(uint32_t position) override;
    virtual void microsteps(ELS::Microsteps ms) override;
    virtual void maxPos(uint32_t position) override;
    virtual void speed(ELS::FocusSpeed speed) override;

protected:
    virtual bool saveConfigItems(FILE *fp) override;

    virtual bool Connect();
    virtual bool Disconnect();

    virtual bool Handshake() override;

    virtual IPState MoveFocuser(INDI::FocuserInterface::FocusDirection dir,
                                int speed,
                                uint16_t duration);
    virtual IPState MoveAbsFocuser(uint32_t targetTicks);
    virtual IPState MoveRelFocuser(INDI::FocuserInterface::FocusDirection dir,
                                   uint32_t ticks);
    virtual bool AbortFocuser();

private:
    void log(const char *fmt, ...);

    void updateAbsPosition(uint32_t position);

private:
    class HCWriter : public ELS::CommsWriter
    {
    public:
        HCWriter(RKSC8Focuser *parent);

        virtual bool writeLine(const char *line);
        virtual void close();

    private:
        RKSC8Focuser *_parent;
        char _buffer[1024];
    };
    friend class HCWriter;

private:
    class HCReader
    {
    public:
        HCReader(int fd,
                 ELS::HostComms *comms,
                 RKSC8Focuser *parent);

        bool start();
        bool shutdown();

    private:
        void readThread();

    private:
        static void *readThreadRedirect(void *obj);

    private:
        static const int g_bufSize = 1023;

    private:
        int _fd;
        ELS::HostComms *_comms;
        RKSC8Focuser *_parent;
        pthread_t _readThreadHandle;
        char _buffer[g_bufSize + 1];
        int _bufLen;
        int _pipefd[2];
    };
    friend class HCReader;

private:
    HCReader *_reader;
    HCWriter *_writer;
    ELS::HostComms *_comms;

    ELS::Microsteps _microsteps;
    uint32_t _maxPos;
    uint32_t _position;
    ELS::FocusSpeed _speed;

    // Enable/Disable
    ISwitch EnableS[2];
    ISwitchVectorProperty EnableSP;

    // Speed
    ISwitch SpeedS[2];
    ISwitchVectorProperty SpeedSP;

    // Microsteps
    ISwitch MicrostepS[4];
    ISwitchVectorProperty MicrostepSP;

    // Zero
    ISwitch ZeroS[1];
    ISwitchVectorProperty ZeroSP;
};
