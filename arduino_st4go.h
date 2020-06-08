/*******************************************************************************
  Copyright(c) 2018 Jasem Mutlaq. All rights reserved.

  Arduino ST4 Driver.

  For this project: https://github.com/kevinferrare/arduino-st4

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/

#pragma once

#include "inditelescope.h"
#include "indiguiderinterface.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

#include <sys/time.h>
#include <time.h>
namespace Connection
{
class Serial;
}
class ARDUST4Driver;
class ArduinoST4 : public INDI::Telescope, public INDI::GuiderInterface
{
  public:
    ArduinoST4();
    virtual ~ArduinoST4() = default;
    typedef enum { ARD_N, ARD_S, ARD_W, ARD_E } ARDUINO_DIRECTION;

    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual void ISGetProperties(const char *dev) override;
    virtual bool ReadScopeStatus() override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    void guideTimeout(ARDUINO_DIRECTION direction);

  protected:
    const char *getDefaultName() override;
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    virtual bool ardMoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command, double ms);
    virtual bool ardMoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command, double ms);
    virtual bool Abort() override;
    virtual bool SetTrackMode(uint8_t mode) override;
    virtual bool SetTrackEnabled(bool enabled) override;
    virtual bool SetTrackRate(double raRate, double deRate) override;

    virtual bool Disconnect() override;
    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;

    // Helper functions
    static void guideTimeoutHelperN(void *p);
    static void guideTimeoutHelperS(void *p);
    static void guideTimeoutHelperW(void *p);
    static void guideTimeoutHelperE(void *p);
    virtual bool Sync(double ra, double dec) override;
    virtual bool Goto(double, double) override;
    virtual bool Park() override;
    virtual bool UnPark() override;

//bool Connect() override;
    // Parking
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;
  private:  
bool Zaparkuj();  
bool Wyparkuj();
   bool callHandshake();
ISwitch UstawSrateS[5];
   ISwitchVectorProperty UstawSrateSP;
   enum { Sr_5, Sr_10,Sr_20,Sr_50,Sr_100 };
ISwitch UstawDecRateS[9];
   ISwitchVectorProperty UstawDecRateSP;
   enum { Dr_5, Dr_10,Dr_20,Dr_50,Dr_100,Dr_200,Dr_400 ,Dr_600,Dr_800};

ISwitch UstawDecStepS[5];
   ISwitchVectorProperty UstawDecStepSP;
   enum { Dst_1,Dst_2,Dst_4,Dst_8,Dst_16};
ISwitch UstawRaStepS[2];
   ISwitchVectorProperty UstawRaStepSP;
   enum { Rst_1,Rst_2};
ISwitch UstawTrackRateS[4];
   ISwitchVectorProperty UstawTrackRateSP;
   enum { Tr_130,Tr_144,Tr_150,Tr_156};
ISwitch UstawPowerS[3];
   ISwitchVectorProperty UstawPowerSP;
   enum { P_Low,P_Mid,P_High};

ISwitch UstawguiderS[5];
   ISwitchVectorProperty UstawguiderSP;
   enum { Gr_11, Gr_12,Gr_13,Gr_15,Gr_20 };
ISwitch TrackStateS[2];
   ISwitchVectorProperty TrackStateSP;
   enum {TRACK_ON, TRACK_OFF};


    bool sendCommand(const char *cmd);
    double currentRA { 0 };
    double currentDEC { 90 };
    double targetRA { 0 };
    double targetDEC { 0 };

    /// used by GoTo and Park
    void StartSlew(double ra, double dec, TelescopeStatus status);

    bool forceMeridianFlip { false };
    unsigned int DBG_SCOPE { 0 };

    double guiderEWTarget[2];
    double guiderNSTarget[2];

    INumber GuideRateN[2];
    INumberVectorProperty GuideRateNP;
ARDUST4Driver *driver;

    // Debug channel to write mount logs to
    // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
    // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
    // for extra debug logs. This way the user can turn it on/off as desired.


    int GuideNSTID { -1 };
    int GuideWETID { -1 };
    ARDUINO_DIRECTION guideDirection;

    int PortFD2 { -1 };

    Connection::Serial *serialConnection { nullptr };

    const uint8_t ARDUINO_TIMEOUT = 3;
};
