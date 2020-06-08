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

#include "arduino_st4go.h"

#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
#include <cmath>

#include <memory>
#include <termios.h>
#include <sys/ioctl.h>

#include <libnova/transform.h>
#include <libnova/julian_day.h>
#include <libnova/utility.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
using namespace std;

// We declare an auto pointer to ArduinoST4.
std::unique_ptr<ArduinoST4> arduinoST4(new ArduinoST4());

#define FLAT_TIMEOUT 3


#define GOTO_RATE      sltr*trrate     /* slew rate, degrees/s Track rate =0.00417*/
#define SLEW_RATE      sltr*trrate      /* slew rate, degrees/s */
#define FINE_SLEW_RATE sltr*trrate      /* slew rate, degrees/s */



#define GOTO_LIMIT      5       /* Move at GOTO_RATE until distance from target is GOTO_LIMIT degrees */
#define SLEW_LIMIT      1       /* Move at SLEW_LIMIT until distance from target is SLEW_LIMIT degrees */

#define RA_AXIS     0
#define DEC_AXIS    1
#define GUIDE_NORTH 0
#define GUIDE_SOUTH 1
#define GUIDE_WEST  0
#define GUIDE_EAST  1

#define MIN_AZ_FLIP 180
#define MAX_AZ_FLIP 200
double guiderr =2;
double trrate = 0.00417808; //86164 sekund na 360stopni

double sltr = 26.06; //ile razy szybciej slew ni¿ track 26.2
double slewrate = sltr*trrate ;
double gotorate = sltr*trrate ;
double finerate = sltr*trrate ;
double trsl =1/sltr;
double trslconst =1/sltr; 
double decsltr = 92.9;
double slewrateD = decsltr*trrate ;
double gotorateD = decsltr*trrate ;
double finerateD = decsltr*trrate ;
int strona = 0; //prawa;

 double parkAZ=0;
double parkAlt=0;

void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    arduinoST4->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    arduinoST4->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    arduinoST4->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    arduinoST4->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root)
{
    arduinoST4->ISSnoopDevice(root);
}

ArduinoST4::ArduinoST4()
{
driver = new ARDUST4Driver("/dev/ttyUSB0");
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_PIER_SIDE_SIMULATION |
                           TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK | TELESCOPE_HAS_TRACK_RATE,
                           4);

    /* initialize random seed: */
    srand(static_cast<uint32_t>(time(nullptr)));

    // assume no pier side property
    currentPierSide = lastPierSide = PIER_UNKNOWN;
}
/*******
ArduinoST4::ArduinoST4() : INDI::GuiderInterface()
{
    setVersion(2, 0);
}
//////////////////*****
bool ArduinoST4::Connect()
{
if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
    SetTimer(POLLMS);
        return true;
    }

    driver->setDebug(isDebug());
SetTimer(POLLMS);
    bool rc = driver->Connect();

    if (rc)
        IDMessage(getDeviceName(), "ARDUST4 is online.");
    else
        IDMessage(getDeviceName(), "Error: cannot find ARDUST4 device.");

    return rc;

}

**********///
bool ArduinoST4::initProperties()
{
    INDI::Telescope::initProperties();
    /************* How fast do we guide compared to sidereal rate *****************/
    IUFillNumber(&GuideRateN[RA_AXIS], "GUIDE_RATE_WE", "W/E Rate", "%g", 0.1, 1 , 0.1, 1);
    IUFillNumber(&GuideRateN[DEC_AXIS], "GUIDE_RATE_NS", "N/S Rate", "%g", 0.1, 1, 0.1, 1);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);

    IUFillSwitch(&SlewRateS[SLEW_GUIDE], "1x", "5", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_CENTERING], "2x", "10", ISS_ON);
    IUFillSwitch(&SlewRateS[SLEW_FIND], "3x", "20", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_MAX], "4x", "100", ISS_OFF);
    IUFillSwitchVector(&SlewRateSP, SlewRateS, 4, getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);


     IUFillSwitch(&TrackStateS[TRACK_ON], "TRACK_ON", "On", ISS_OFF);
     IUFillSwitch(&TrackStateS[TRACK_OFF], "TRACK_OFF", "Off", ISS_ON);
     IUFillSwitchVector(&TrackStateSP, TrackStateS, 2, getDeviceName(), "TELESCOPE_TRACK_STATE", "Tracking", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0,
                        IPS_IDLE);
IUFillSwitch(&UstawSrateS[Sr_5], "500", "50X", ISS_OFF);
IUFillSwitch(&UstawSrateS[Sr_10], "1000", "100X", ISS_OFF);
IUFillSwitch(&UstawSrateS[Sr_20], "2606", "260X", ISS_ON);
IUFillSwitch(&UstawSrateS[Sr_50], "2100", "209X", ISS_OFF);
IUFillSwitch(&UstawSrateS[Sr_100], "2718", "272X", ISS_OFF);
IUFillSwitchVector(&UstawSrateSP, UstawSrateS, 5, getDeviceName(), "SRATE_CONTROL", "Ra Slew rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

IUFillSwitch(&UstawRaStepS[Rst_1], "1", "1MR", ISS_OFF);
IUFillSwitch(&UstawRaStepS[Rst_2], "2", "2MR", ISS_ON);
IUFillSwitchVector(&UstawRaStepSP, UstawRaStepS, 2, getDeviceName(), "RSTEP_CONTROL", "Ra Step", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);


IUFillSwitch(&UstawPowerS[P_Low], "PL", "PL", ISS_OFF);
IUFillSwitch(&UstawPowerS[P_Mid], "PM", "PM", ISS_ON);
IUFillSwitch(&UstawPowerS[P_High], "PH", "PH", ISS_OFF);
IUFillSwitchVector(&UstawPowerSP, UstawPowerS, 3, getDeviceName(), "POWER_CONTROL", "Power", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

IUFillSwitch(&UstawTrackRateS[Tr_130], "130Z", "Prawa", ISS_ON);
IUFillSwitch(&UstawTrackRateS[Tr_144], "144Z", "Lewa", ISS_OFF);
IUFillSwitch(&UstawTrackRateS[Tr_150], "150Z", "--", ISS_OFF);
IUFillSwitch(&UstawTrackRateS[Tr_156], "156Z", "--", ISS_OFF);
IUFillSwitchVector(&UstawTrackRateSP, UstawTrackRateS, 4, getDeviceName(), "TRATE_CONTROL", "Strona monatzu", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);


IUFillSwitch(&UstawDecRateS[Dr_5], "5", "5X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_10], "10", "10X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_20], "20", "20X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_50], "50", "50X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_100], "58", "58X", ISS_ON);
IUFillSwitch(&UstawDecRateS[Dr_200], "116", "116X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_400], "232", "232X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_600], "600", "600X", ISS_OFF);
IUFillSwitch(&UstawDecRateS[Dr_800], "800", "800X", ISS_OFF);
IUFillSwitchVector(&UstawDecRateSP, UstawDecRateS, 9, getDeviceName(), "DRATE_CONTROL", "Dec Slew rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

IUFillSwitch(&UstawDecStepS[Dst_1], "1", "1M", ISS_OFF);
IUFillSwitch(&UstawDecStepS[Dst_2], "2", "2M", ISS_OFF);
IUFillSwitch(&UstawDecStepS[Dst_4], "4", "4M", ISS_OFF);
IUFillSwitch(&UstawDecStepS[Dst_8], "8", "8M", ISS_OFF);
IUFillSwitch(&UstawDecStepS[Dst_16], "16", "16M", ISS_ON);
IUFillSwitchVector(&UstawDecStepSP, UstawDecStepS, 5, getDeviceName(), "DSTEP_CONTROL", "Dec Step", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);


IUFillSwitch(&UstawguiderS[Gr_11], "11", "1.1X", ISS_OFF);
IUFillSwitch(&UstawguiderS[Gr_12], "12", "1.2X", ISS_OFF);
IUFillSwitch(&UstawguiderS[Gr_13], "13", "1.3X", ISS_OFF);
IUFillSwitch(&UstawguiderS[Gr_15], "15", "1.5X", ISS_OFF);
IUFillSwitch(&UstawguiderS[Gr_20], "20", "2X", ISS_ON);
IUFillSwitchVector(&UstawguiderSP, UstawguiderS, 5, getDeviceName(), "GRATE_CONTROL", "Guiding rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);


    // Add Tracking Modes
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_CUSTOM", "Custom");

    // Let's simulate it to be an F/7.5 120mm telescope
    ScopeParametersN[0].value = 120;
    ScopeParametersN[1].value = 900;
    ScopeParametersN[2].value = 120;
    ScopeParametersN[3].value = 448;

    TrackState = SCOPE_IDLE;

    SetParkDataType(PARK_RA_DEC);

    initGuiderProperties(getDeviceName(), MOTION_TAB);

    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();

    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
double longitude = 20, latitude = 50;
//    double longitude = 19.93524, latitude = 50.05230;
    // Get value from config file if it exists.
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
    currentRA  = get_local_sidereal_time(longitude);
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LAT", &latitude);
    currentDEC = latitude > 0 ? 90 : -90;
//Wyparkuj();
//setPollingPeriodRange(100, 36000000);
    setDefaultPollingPeriod(250);


    addAuxControls();
/*****

    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]() { return Handshake(); });
    serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
     Arduino default port
   serialConnection->setDefaultPort("/dev/ttyACM0");
  registerConnection(serialConnection);
*******/

/****
    SetTelescopeCapability(TELESCOPE_CAN_SYNC |TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT, 4);
*****/

    return true;
}


void ArduinoST4::ISGetProperties(const char *dev)
{
    /* First we let our parent populate */
    INDI::Telescope::ISGetProperties(dev);


}

/**************************************************************************************
** !!!!!!!!!!!!!!
***************************************************************************************/
bool ArduinoST4::updateProperties()
{
    INDI::Telescope::updateProperties();
    uint32_t cap = GetTelescopeCapability();
    if (IUFindOnSwitchIndex(&SimulatePierSideSP) == 0)
    {
        cap |= TELESCOPE_HAS_PIER_SIDE;
    }
    else
    {
        cap &= ~static_cast<uint32_t>(TELESCOPE_HAS_PIER_SIDE);
    }

    SetTelescopeCapability(cap, 4);


    if (isConnected())
    {
        defineNumber(&GuideNSNP);
        defineNumber(&GuideWENP);
        defineNumber(&GuideRateNP);
defineSwitch(&UstawSrateSP);
defineSwitch(&UstawDecRateSP);
defineSwitch(&UstawguiderSP);
defineSwitch(&TrackStateSP);
defineSwitch(&SlewRateSP);

defineSwitch(&UstawDecStepSP);
defineSwitch(&UstawRaStepSP);
defineSwitch(&UstawTrackRateSP);
defineSwitch(&UstawPowerSP);

        sendTimeFromSystem();
 

/******
        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(currentRA);
            SetAxis2ParkDefault(currentDEC);

            if (isParked())
            {
                currentRA = ParkPositionN[AXIS_RA].value;
                currentDEC = ParkPositionN[AXIS_DE].value;
            }
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(currentRA);
            SetAxis2Park(currentDEC);
            SetAxis1ParkDefault(currentRA);
            SetAxis2ParkDefault(currentDEC);
        }
   ******/     

        // initialise the pier side if it's available
        if (HasPierSide())
            currentPierSide = Telescope::expectedPierSide(currentRA);
    }
    else
    {
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
deleteProperty(UstawSrateSP.name);
deleteProperty(UstawDecRateSP.name);
deleteProperty(UstawguiderSP.name);
deleteProperty(TrackStateSP.name);
deleteProperty(SlewRateSP.name);
deleteProperty(UstawDecStepSP.name);
deleteProperty(UstawRaStepSP.name);
deleteProperty(UstawTrackRateSP.name);
deleteProperty(UstawPowerSP.name);

defineSwitch(&UstawDecStepSP);
defineSwitch(&UstawRaStepSP);
defineSwitch(&UstawTrackRateSP);
defineSwitch(&UstawPowerSP);

    }

    return true;
}

const char *ArduinoST4::getDefaultName()
{
    return static_cast<const char *>("Arduino ST4 GOTO");
}

bool ArduinoST4::callHandshake()
{
    if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
    SetTimer(POLLMS);
        return true;
    }

    PortFD2 = serialConnection->getPortFD();    
    SetTimer(POLLMS);
    return true;
}


bool ArduinoST4::Disconnect()
{
    sendCommand("DISCONNECT#");

    return INDI::Telescope::Disconnect();
}
/*************************************
bool ArduinoST4::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (!strcmp(name, GuideNSNP.name) || !strcmp(name, GuideWENP.name))
    {
        processGuiderProperties(name, values, names, n);
        return true;
    }

    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}
*********************/
bool ArduinoST4::ReadScopeStatus()
{
    static struct timeval lastTime
    {
        0, 0
    };
    struct timeval currentTime
    {
        0, 0
    };
    double deltaTimeSecs = 0, da_ra = 0, da_dec = 0, deltaRa = 0, deltaDec = 0, ra_guide_dt = 0, dec_guide_dt = 0;
    int nlocked, ns_guide_dir = -1, we_guide_dir = -1;

#ifdef USE_EQUATORIAL_PE
    static double last_dx = 0, last_dy = 0;
    char RA_DISP[64], DEC_DISP[64], RA_GUIDE[64], DEC_GUIDE[64], RA_PE[64], DEC_PE[64], RA_TARGET[64], DEC_TARGET[64];
#endif

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&currentTime, nullptr);

    if (lastTime.tv_sec == 0 && lastTime.tv_usec == 0)
        lastTime = currentTime;

    // Time diff in seconds
    deltaTimeSecs  = currentTime.tv_sec - lastTime.tv_sec + (currentTime.tv_usec - lastTime.tv_usec) / 1e6;
    lastTime = currentTime;

    if (fabs(targetRA - currentRA) * 15. >= GOTO_LIMIT)
        da_ra = gotorate * deltaTimeSecs;
    else if (fabs(targetRA - currentRA) * 15. >= SLEW_LIMIT)
        da_ra = slewrate * deltaTimeSecs;
    else
        da_ra = finerate * deltaTimeSecs;

    if (fabs(targetDEC - currentDEC) >= GOTO_LIMIT)
        da_dec = gotorateD * deltaTimeSecs;
    else if (fabs(targetDEC - currentDEC) >= SLEW_LIMIT)
        da_dec = slewrateD * deltaTimeSecs;
    else
        da_dec = finerateD * deltaTimeSecs;

    if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY)
    {
        int rate = IUFindOnSwitchIndex(&SlewRateSP);

        switch (rate)
        {
            case SLEW_GUIDE:
                da_ra  = TrackRateN[AXIS_RA].value / (3600.0 * 15) * GuideRateN[RA_AXIS].value * deltaTimeSecs;
                da_dec = TrackRateN[AXIS_RA].value / 3600.0 * GuideRateN[DEC_AXIS].value * deltaTimeSecs;;
                break;

            case SLEW_CENTERING:
                da_ra  = finerate * deltaTimeSecs * .1;
                da_dec = finerateD * deltaTimeSecs * .1;
                break;

            case SLEW_FIND:
                da_ra  = slewrate * deltaTimeSecs;
                da_dec = slewrateD * deltaTimeSecs;
                break;

            default:
                da_ra  = gotorate * deltaTimeSecs;
                da_dec = gotorateD * deltaTimeSecs;
                break;
        }

        switch (MovementNSSP.s)
        {
            case IPS_BUSY:
                if (MovementNSS[DIRECTION_NORTH].s == ISS_ON)
                    currentDEC += da_dec;
                else if (MovementNSS[DIRECTION_SOUTH].s == ISS_ON)
                    currentDEC -= da_dec;
                break;

            default:
                break;
        }

        switch (MovementWESP.s)
        {
            case IPS_BUSY:

                if (MovementWES[DIRECTION_WEST].s == ISS_ON)
                    currentRA -= da_ra / 15.;
                else if (MovementWES[DIRECTION_EAST].s == ISS_ON)
                    currentRA += da_ra / 15.;
                break;

            default:
                break;
        }

        NewRaDec(currentRA, currentDEC);
        return true;
    }

    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
    switch (TrackState)
    {
        /*case SCOPE_IDLE:
            EqNP.s = IPS_IDLE;
            break;*/
        case SCOPE_SLEWING:
        case SCOPE_PARKING:
            /* slewing - nail it when both within one pulse @ SLEWRATE */
            nlocked = 0;        // seems to be some sort of state variable

            deltaRa = targetRA - currentRA;

            // Always take the shortcut, don't go all around the globe
            // If the difference between target and current is more than 12 hours, then we need to take the shortest path
            if (deltaRa > 12)
                deltaRa -= 24;
            else if (deltaRa < -12)
                deltaRa += 24;

            // In meridian flip, move to the position by doing a full rotation
            if (forceMeridianFlip)
            {
                // set deltaRa according to the target pier side so that the
                // slew is away from the meridian until the direction to go is towards
                // the target.
                switch (currentPierSide)
                {
                case PIER_EAST:
                    // force Ra move direction to be positive, i.e. to the West,
                    // until it is large and positive
                    if (deltaRa < 9)
                        deltaRa = 1;
                    else
                        forceMeridianFlip = false;
                    break;
                case PIER_WEST:
                    // force Ra move direction to be negative, i.e. East,
                    // until it is large and negative
                    if (deltaRa > -9)
                        deltaRa = -1;
                    else
                        forceMeridianFlip = false;
                    break;
                case PIER_UNKNOWN:
                    break;
                }
            }

            if (fabs(deltaRa) * 15. <= da_ra)
            {
                currentRA = targetRA;
                nlocked++;
            }
            else if (deltaRa > 0)
                currentRA += da_ra / 15.;
            else
                currentRA -= da_ra / 15.;

            currentRA = range24(currentRA);

            deltaDec = targetDEC - currentDEC;
            if (fabs(deltaDec) <= da_dec)
            {
                currentDEC = targetDEC;
                nlocked++;
            }
            else if (deltaDec > 0)
                currentDEC += da_dec;
            else
                currentDEC -= da_dec;

            EqNP.s = IPS_BUSY;

            if (nlocked == 2)
            {
                forceMeridianFlip = false;

                if (TrackState == SCOPE_SLEWING)
                {
                    // Initially no PE in both axis.
#ifdef USE_EQUATORIAL_PE
                    EqPEN[0].value = currentRA;
                    EqPEN[1].value = currentDEC;
                    IDSetNumber(&EqPENV, nullptr);
#endif

                    TrackState = SCOPE_TRACKING;

                    if (IUFindOnSwitchIndex(&SlewRateSP) != SLEW_CENTERING)
                    {
                        IUResetSwitch(&SlewRateSP);
                        SlewRateS[SLEW_CENTERING].s = ISS_ON;
                        IDSetSwitch(&SlewRateSP, nullptr);
                    }


                    EqNP.s = IPS_OK;
                    LOG_INFO("Telescope slew is complete. Tracking...");
                }
                else
                {
                    SetParked(true);
                    EqNP.s = IPS_IDLE;
                }
            }

            break;

        case SCOPE_IDLE:
            //currentRA += (TRACKRATE_SIDEREAL/3600.0 * dt) / 15.0;
            currentRA += (TrackRateN[AXIS_RA].value / 3600.0 * deltaTimeSecs) / 15.0;
            currentRA = range24(currentRA);
            break;

        case SCOPE_TRACKING:
            // In case of custom tracking rate
            if (TrackModeS[1].s == ISS_ON)
            {
                currentRA  += ( ((TRACKRATE_SIDEREAL / 3600.0) - (TrackRateN[AXIS_RA].value / 3600.0)) * deltaTimeSecs) / 15.0;
                currentDEC += ( (TrackRateN[AXIS_DE].value / 3600.0) * deltaTimeSecs);
            }

            deltaTimeSecs *= 1000;

            if (guiderNSTarget[GUIDE_NORTH] > 0)
            {
                LOGF_DEBUG("Commanded to GUIDE NORTH for %g ms", guiderNSTarget[GUIDE_NORTH]);
                ns_guide_dir = GUIDE_NORTH;
            }
            else if (guiderNSTarget[GUIDE_SOUTH] > 0)
            {
                LOGF_DEBUG("Commanded to GUIDE SOUTH for %g ms", guiderNSTarget[GUIDE_SOUTH]);
                ns_guide_dir = GUIDE_SOUTH;
            }

            // WE Guide Selection
            if (guiderEWTarget[GUIDE_WEST] > 0)
            {
                we_guide_dir = GUIDE_WEST;
                LOGF_DEBUG("Commanded to GUIDE WEST for %g ms", guiderEWTarget[GUIDE_WEST]);
            }
            else if (guiderEWTarget[GUIDE_EAST] > 0)
            {
                we_guide_dir = GUIDE_EAST;
                LOGF_DEBUG("Commanded to GUIDE EAST for %g ms", guiderEWTarget[GUIDE_EAST]);
            }

            if ( (ns_guide_dir != -1 || we_guide_dir != -1) && IUFindOnSwitchIndex(&SlewRateSP) != SLEW_GUIDE)
            {
                IUResetSwitch(&SlewRateSP);
                SlewRateS[SLEW_GUIDE].s = ISS_ON;
                IDSetSwitch(&SlewRateSP, nullptr);
            }

            if (ns_guide_dir != -1)
            {
                dec_guide_dt = (TrackRateN[AXIS_RA].value * GuideRateN[DEC_AXIS].value * guiderNSTarget[ns_guide_dir] / 1000.0 *
                                (ns_guide_dir == GUIDE_NORTH ? 1 : -1)) / 3600.0;

                guiderNSTarget[ns_guide_dir] = 0;
                GuideNSNP.s = IPS_IDLE;
                IDSetNumber(&GuideNSNP, nullptr);

#ifdef USE_EQUATORIAL_PE
                EqPEN[DEC_AXIS].value += dec_guide_dt;
#else
                currentDEC += dec_guide_dt;
#endif
            }

            if (we_guide_dir != -1)
            {
                ra_guide_dt = (TrackRateN[AXIS_RA].value * GuideRateN[RA_AXIS].value * guiderEWTarget[we_guide_dir] / 1000.0 *
                               (we_guide_dir == GUIDE_WEST ? -1 : 1)) / (3600.0 * 15.0);

                ra_guide_dt /= (cos(currentDEC * 0.0174532925));

                guiderEWTarget[we_guide_dir] = 0;
                GuideWENP.s = IPS_IDLE;
                IDSetNumber(&GuideWENP, nullptr);

#ifdef USE_EQUATORIAL_PE
                EqPEN[RA_AXIS].value += ra_guide_dt;
#else
                currentRA += ra_guide_dt;
#endif
            }

            //Mention the followng:
            // Current RA displacemet and direction
            // Current DEC displacement and direction
            // Amount of RA GUIDING correction and direction
            // Amount of DEC GUIDING correction and direction

#ifdef USE_EQUATORIAL_PE

            dx = EqPEN[RA_AXIS].value - targetRA;
            dy = EqPEN[DEC_AXIS].value - targetDEC;
            fs_sexa(RA_DISP, fabs(dx), 2, 3600);
            fs_sexa(DEC_DISP, fabs(dy), 2, 3600);

            fs_sexa(RA_GUIDE, fabs(ra_guide_dt), 2, 3600);
            fs_sexa(DEC_GUIDE, fabs(dec_guide_dt), 2, 3600);

            fs_sexa(RA_PE, EqPEN[RA_AXIS].value, 2, 3600);
            fs_sexa(DEC_PE, EqPEN[DEC_AXIS].value, 2, 3600);

            fs_sexa(RA_TARGET, targetRA, 2, 3600);
            fs_sexa(DEC_TARGET, targetDEC, 2, 3600);

            if (dx != last_dx || dy != last_dy || ra_guide_dt != 0.0 || dec_guide_dt != 0.0)
            {
                last_dx = dx;
                last_dy = dy;
                //LOGF_DEBUG("dt is %g\n", dt);
                LOGF_DEBUG("RA Displacement (%c%s) %s -- %s of target RA %s", dx >= 0 ? '+' : '-',
                           RA_DISP, RA_PE, (EqPEN[RA_AXIS].value - targetRA) > 0 ? "East" : "West", RA_TARGET);
                LOGF_DEBUG("DEC Displacement (%c%s) %s -- %s of target RA %s", dy >= 0 ? '+' : '-',
                           DEC_DISP, DEC_PE, (EqPEN[DEC_AXIS].value - targetDEC) > 0 ? "North" : "South", DEC_TARGET);
                LOGF_DEBUG("RA Guide Correction (%g) %s -- Direction %s", ra_guide_dt, RA_GUIDE,
                           ra_guide_dt > 0 ? "East" : "West");
                LOGF_DEBUG("DEC Guide Correction (%g) %s -- Direction %s", dec_guide_dt, DEC_GUIDE,
                           dec_guide_dt > 0 ? "North" : "South");
            }

            if (ns_guide_dir != -1 || we_guide_dir != -1)
                IDSetNumber(&EqPENV, nullptr);
#endif

            break;

        default:
            break;
    }

    char RAStr[64], DecStr[64];

    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);

    return true;
}

bool ArduinoST4::Goto(double r, double d)
{
    StartSlew(r, d, SCOPE_SLEWING);
    return true;
}

bool ArduinoST4::Sync(double ra, double dec)
{
    currentRA  = ra;
    currentDEC = dec;

#ifdef USE_EQUATORIAL_PE
    EqPEN[RA_AXIS].value  = ra;
    EqPEN[DEC_AXIS].value = dec;
    IDSetNumber(&EqPENV, nullptr);
#endif

    LOG_INFO("Sync is successful.");

    EqNP.s = IPS_OK;

    NewRaDec(currentRA, currentDEC);

    return true;
}

bool ArduinoST4::Park()
{
    StartSlew(GetAxis1Park(), GetAxis2Park(), SCOPE_PARKING);
//Zaparkuj();
    return true;
}

// common code for GoTo and park
void ArduinoST4::StartSlew(double ra, double dec, TelescopeStatus status)
{
    targetRA  = ra;
    targetDEC = dec;
    char RAStr[64], DecStr[64];

    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);



    if (getSimulatePierSide())
    {
        // set the pier side
        TelescopePierSide newPierSide = expectedPierSide(targetRA);

        // check if a meridian flip is needed
        if (newPierSide != currentPierSide)
        {
            forceMeridianFlip = true;
            setPierSide(newPierSide);
        }
        currentPierSide = newPierSide;
    }

    if (IUFindOnSwitchIndex(&TrackModeSP) != SLEW_MAX)
    {
        IUResetSwitch(&TrackModeSP);
        TrackModeS[SLEW_MAX].s = ISS_ON;
        IDSetSwitch(&TrackModeSP, nullptr);
    }

    const char * statusStr;
    switch (status)
    {
    case SCOPE_PARKING:
        statusStr = "Parking";
        break;
    case SCOPE_SLEWING:
        statusStr = "Slewing";
        break;
    default:
        statusStr = "unknown";
    }
    TrackState = status;

/*******************************************
*********************************************/
/***********
double przelicznik = 1000/slewrate;
double przelicznik2 = SLEDZ_RATE*0.001; //ile stopni na milisek
***************/
double przelicznik = 1000/slewrate;
double przelicznikwe = 8960;
//double przelicznikDec = 1000/slewrateD;
double przelicznikDec = 433.333333333;
LOGF_ERROR("gslewelta: %f", slewrate);

LOGF_ERROR("gotprzelta: %f", przelicznik);
LOGF_ERROR("gotprzelta2: %f", przelicznikDec);
	double przelicznik2 =1/(przelicznik)*0.1;
 //track do slew

double ms;
unsigned long ilosckrokowdec;
const char* komenda;
unsigned long ilosckrokowwe;
const char* komendawe;


            double rightAscensionDelta = targetRA - currentRA;
            if (rightAscensionDelta < -12)
            {
                //Shortest path from 24 to 0
                rightAscensionDelta += 24;
            }
            else if (rightAscensionDelta > 12)
            {
                //Shortest path from 0 to 24
                rightAscensionDelta -= 24;
            }
            if (rightAscensionDelta < 0)
		
            {LOGF_ERROR("gotoWmsdelta: %f", rightAscensionDelta);
               rightAscensionDelta=-1*rightAscensionDelta; 
rightAscensionDelta=rightAscensionDelta*15;
		ms = przelicznik*rightAscensionDelta;
/******
poprawki
*********/ 
LOGF_ERROR("gotoWms: %f", ms);
ms = ms/(1-trsl);


LOGF_ERROR("gotoWpopoprms: %f", ms);

ilosckrokowwe= (przelicznikwe*rightAscensionDelta)/(1-trsl);

std::string komendawe1= "WEST";
int lengthwe = snprintf( NULL, 0, "%lu", ilosckrokowwe);
char* komendawe2 = (char*)malloc( lengthwe + 1 );
snprintf( komendawe2, lengthwe + 1, "%lu", ilosckrokowwe);


komendawe1 +=komendawe2;
free(komendawe2);
komendawe1 +="#";
const char* komendawe=komendawe1.c_str();
LOGF_DEBUG("gotowesteps: %s", komendawe);
LOGF_ERROR("gotonwesteps: %s", komendawe);
sendCommand(komendawe);



               // ardMoveWE(DIRECTION_WEST, MOTION_START, ms);
            }
            else if (rightAscensionDelta > 0)
             {
LOGF_DEBUG("gotoEmsdelta: %f", rightAscensionDelta);
LOGF_ERROR("gotoedeltams: %f", rightAscensionDelta);
rightAscensionDelta=rightAscensionDelta*15;
                ms = przelicznik*rightAscensionDelta;
/******
poprawki
*********/
LOGF_ERROR("gotoEms: %f", ms);
ms = ms/(1+trsl);
LOGF_ERROR("gotoEpopoprms: %f", ms);

ilosckrokowwe= (przelicznikwe*rightAscensionDelta)/(1+trsl);

std::string komendawe1= "EAST";
int lengthwe = snprintf( NULL, 0, "%lu", ilosckrokowwe);
char* komendawe2 = (char*)malloc( lengthwe + 1 );
snprintf( komendawe2, lengthwe + 1, "%lu", ilosckrokowwe);


komendawe1 +=komendawe2;
free(komendawe2);
komendawe1 +="#";
const char* komendawe=komendawe1.c_str();
LOGF_DEBUG("gotowesteps: %s", komendawe);
LOGF_ERROR("gotonwesteps: %s", komendawe);
sendCommand(komendawe);


               // ardMoveWE(DIRECTION_EAST, MOTION_START, ms);
            }





double declinationDelta = targetDEC - currentDEC;
            if (declinationDelta < 0)
            {LOGF_ERROR("gotoSmsdelta: %f", declinationDelta);
               // ms= -1*przelicznikDec*declinationDelta;
//LOGF_DEBUG("gotonSms: %f", ms);
//LOGF_ERROR("gotonsms: %f", ms);
    //            ardMoveNS(DIRECTION_SOUTH, MOTION_START, ms);

ilosckrokowdec= -1*przelicznikDec*declinationDelta;
//int x =2;
std::string komenda1;
if (strona == 0) {
komenda1= "NORTH";}
if (strona == 1) {
komenda1= "SOUTH";}
int length = snprintf( NULL, 0, "%lu", ilosckrokowdec);
char* komenda2 = (char*)malloc( length + 1 );
snprintf( komenda2, length + 1, "%lu", ilosckrokowdec);


komenda1 +=komenda2;
free(komenda2);
komenda1 +="#";
const char* komenda=komenda1.c_str();
LOGF_DEBUG("gotonSsteps: %s", komenda);
LOGF_ERROR("gotonSsteps: %s", komenda);
sendCommand(komenda);
            }
            else if (declinationDelta > 0)

            {LOGF_ERROR("gotoNmsdelta: %f", declinationDelta);
//
                ms= przelicznikDec*declinationDelta;

 //   LOGF_ERROR("gotoNsms: %f", ms);
 //               ardMoveNS(DIRECTION_, MOTION_START, ms);
ilosckrokowdec= 1*przelicznikDec*declinationDelta;
//int x =2;
std::string komenda1;
if (strona == 1) {
komenda1= "NORTH";}
if (strona == 0) {
komenda1= "SOUTH";}
int length = snprintf( NULL, 0, "%lu", ilosckrokowdec);
char* komenda2 = (char*)malloc( length + 1 );
snprintf( komenda2, length + 1, "%lu", ilosckrokowdec);


komenda1 +=komenda2;
free(komenda2);
komenda1 +="#";
const char* komenda=komenda1.c_str();
LOGF_DEBUG("gotonSsteps: %s", komenda);
LOGF_ERROR("gotonSsteps: %s", komenda);
sendCommand(komenda);
            }
/**************************
*******************************************/


    LOGF_INFO("%s to RA: %s - DEC: %s, pier side %s, %s", statusStr, RAStr, DecStr, getPierSideStr(currentPierSide), forceMeridianFlip ? "with flip" : "direct");
}

bool ArduinoST4::UnPark()
{
    SetParked(false);
Wyparkuj();
    return true;
}

bool ArduinoST4::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, "GUIDE_RATE") == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);
unsigned long gra = GuideRateN[0].value*10+0.5;

std::string komenda1= "GR";
int length = snprintf( NULL, 0, "%lu", gra);
char* komenda2 = (char*)malloc( length + 1 );
snprintf( komenda2, length + 1, "%lu", gra);


komenda1 +=komenda2;
free(komenda2);
komenda1 +="#";
const char* komenda=komenda1.c_str();
LOGF_DEBUG("gotonSsteps: %s", komenda);
LOGF_ERROR("gotonSsteps: %s", komenda);
sendCommand(komenda);
    
            return true;
        }

        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}


bool ArduinoST4::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Slew mode
        if (strcmp(name, SlewRateSP.name) == 0)
        {
            if (IUUpdateSwitch(&SlewRateSP, states, names, n) < 0)
                return false;

            SlewRateSP.s = IPS_OK;
            IDSetSwitch(&SlewRateSP, nullptr);
            return true;
        }


if (!strcmp(name, UstawDecStepSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawDecStepIndex = IUFindOnSwitchIndex(&UstawDecStepSP);
          if (!strcmp(actionName, UstawDecStepS[currentUstawDecStepIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawDecStepS is already %s", UstawDecStepS[currentUstawDecStepIndex].label);
             UstawDecStepSP.s = IPS_IDLE;
             IDSetSwitch(&UstawDecStepSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawDecStepSP, states, names, n);
          currentUstawDecStepIndex = IUFindOnSwitchIndex(&UstawDecStepSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawDecStepS is now %s", UstawDecStepS[currentUstawDecStepIndex].label);
          UstawDecStepSP.s = IPS_OK;
          IDSetSwitch(&UstawDecStepSP, NULL);
std::string cmd1= UstawDecStepS[currentUstawDecStepIndex].label;
cmd1 +="#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);


          return true;
      }

if (!strcmp(name, UstawRaStepSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawRaStepIndex = IUFindOnSwitchIndex(&UstawRaStepSP);
          if (!strcmp(actionName, UstawRaStepS[currentUstawRaStepIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawRaStepS is already %s", UstawRaStepS[currentUstawRaStepIndex].label);
             UstawRaStepSP.s = IPS_IDLE;
             IDSetSwitch(&UstawRaStepSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawRaStepSP, states, names, n);
          currentUstawRaStepIndex = IUFindOnSwitchIndex(&UstawRaStepSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawRaStepS is now %s", UstawRaStepS[currentUstawRaStepIndex].label);
          UstawRaStepSP.s = IPS_OK;
          IDSetSwitch(&UstawRaStepSP, NULL);
std::string cmd1= UstawRaStepS[currentUstawRaStepIndex].label;
cmd1 +="#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);


          return true;
      }

if (!strcmp(name, UstawTrackRateSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawTrackRateIndex = IUFindOnSwitchIndex(&UstawTrackRateSP);
          if (!strcmp(actionName, UstawTrackRateS[currentUstawTrackRateIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawTrackRateS is already %s", UstawTrackRateS[currentUstawTrackRateIndex].label);
             UstawTrackRateSP.s = IPS_IDLE;
             IDSetSwitch(&UstawTrackRateSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawTrackRateSP, states, names, n);
          currentUstawTrackRateIndex = IUFindOnSwitchIndex(&UstawTrackRateSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawTrackRateS is now %s", UstawTrackRateS[currentUstawTrackRateIndex].label);
          UstawTrackRateSP.s = IPS_OK;
          IDSetSwitch(&UstawTrackRateSP, NULL);
if (currentUstawTrackRateIndex==0) {strona=0;}
if (currentUstawTrackRateIndex==1) {strona=1;}
//std::string cmd1= UstawTrackRateS[currentUstawTrackRateIndex].label;
//cmd1 +="#";
//const char* cmd=cmd1.c_str();
//sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "strona is now %d",strona);


          return true;
      }

if (!strcmp(name, UstawPowerSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawPowerIndex = IUFindOnSwitchIndex(&UstawPowerSP);
          if (!strcmp(actionName, UstawPowerS[currentUstawPowerIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawPowerS is already %s", UstawPowerS[currentUstawPowerIndex].label);
             UstawPowerSP.s = IPS_IDLE;
             IDSetSwitch(&UstawPowerSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawPowerSP, states, names, n);
          currentUstawPowerIndex = IUFindOnSwitchIndex(&UstawPowerSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawPowerS is now %s", UstawPowerS[currentUstawPowerIndex].label);
          UstawPowerSP.s = IPS_OK;
          IDSetSwitch(&UstawPowerSP, NULL);
std::string cmd1= UstawPowerS[currentUstawPowerIndex].label;
cmd1 +="#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);


          return true;
      }


if (!strcmp(name, UstawguiderSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawguiderIndex = IUFindOnSwitchIndex(&UstawguiderSP);
          if (!strcmp(actionName, UstawguiderS[currentUstawguiderIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawguiderS is already %s", UstawguiderS[currentUstawguiderIndex].label);
             UstawguiderSP.s = IPS_IDLE;
             IDSetSwitch(&UstawguiderSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawguiderSP, states, names, n);
          currentUstawguiderIndex = IUFindOnSwitchIndex(&UstawguiderSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawguiderS is now %s", UstawguiderS[currentUstawguiderIndex].label);
          UstawguiderSP.s = IPS_OK;
          IDSetSwitch(&UstawguiderSP, NULL);
std::string cmd1= UstawguiderS[currentUstawguiderIndex].label;
cmd1 +="#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);

std::string stringi(UstawguiderS[currentUstawguiderIndex].name);
std::string::size_type sz;
int i_dec = std::stoi (stringi,&sz);
guiderr=i_dec;



DEBUGF(INDI::Logger::DBG_SESSION, "guiderr is now %f",guiderr);

          return true;
      }


if (!strcmp(name, UstawSrateSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawSrateIndex = IUFindOnSwitchIndex(&UstawSrateSP);
          if (!strcmp(actionName, UstawSrateS[currentUstawSrateIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawSrateS is already %s", UstawSrateS[currentUstawSrateIndex].label);
             UstawSrateSP.s = IPS_IDLE;
             IDSetSwitch(&UstawSrateSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawSrateSP, states, names, n);
          currentUstawSrateIndex = IUFindOnSwitchIndex(&UstawSrateSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawSrateS is now %s", UstawSrateS[currentUstawSrateIndex].label);
          UstawSrateSP.s = IPS_OK;
          IDSetSwitch(&UstawSrateSP, NULL);
std::string cmd1= UstawSrateS[currentUstawSrateIndex].label;
cmd1 +="#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);

std::string stringi(UstawSrateS[currentUstawSrateIndex].name);
std::string::size_type sz;
int i_dec = std::stoi (stringi,&sz);
sltr=i_dec/100;
//sltr=(currentUstawSrateIndex+1)*5;
slewrate = sltr*trrate ;
gotorate = sltr*trrate ;
finerate = sltr*trrate ;
trsl =1/sltr;
if (TrackState == SCOPE_SLEWING){
StartSlew(targetRA, targetDEC, SCOPE_SLEWING);}
DEBUGF(INDI::Logger::DBG_SESSION, "sltr is now %f",sltr);
DEBUGF(INDI::Logger::DBG_SESSION, "trsl is now %f",trsl);

          return true;
      }

if (!strcmp(name, UstawDecRateSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentUstawDecRateIndex = IUFindOnSwitchIndex(&UstawDecRateSP);
          if (!strcmp(actionName, UstawDecRateS[currentUstawDecRateIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "UstawDecRateS is already %s", UstawDecRateS[currentUstawDecRateIndex].label);
             UstawDecRateSP.s = IPS_IDLE;
             IDSetSwitch(&UstawDecRateSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&UstawDecRateSP, states, names, n);
          currentUstawDecRateIndex = IUFindOnSwitchIndex(&UstawDecRateSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "UstawDecRateS is now %s", UstawDecRateS[currentUstawDecRateIndex].label);
          UstawDecRateSP.s = IPS_OK;
          IDSetSwitch(&UstawDecRateSP, NULL);
std::string cmd1= UstawDecRateS[currentUstawDecRateIndex].label;
cmd1 +="D#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);

std::string stringi(UstawDecRateS[currentUstawDecRateIndex].name);
std::string::size_type sz;
int i_dec = std::stoi (stringi,&sz);
decsltr=i_dec;
slewrateD = decsltr*trrate ;
gotorateD = decsltr*trrate ;
finerateD = decsltr*trrate ;
if (TrackState == SCOPE_SLEWING){
StartSlew(targetRA, targetDEC, SCOPE_SLEWING);}

DEBUGF(INDI::Logger::DBG_SESSION, "decsltr is now %f",decsltr);

          return true;
      }
if (!strcmp(name, SlewRateSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentSlewRateIndex = IUFindOnSwitchIndex(&SlewRateSP);
          if (!strcmp(actionName, SlewRateS[currentSlewRateIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "SlewRateS is already %s", SlewRateS[currentSlewRateIndex].label);
             SlewRateSP.s = IPS_IDLE;
             IDSetSwitch(&SlewRateSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&SlewRateSP, states, names, n);
          currentSlewRateIndex = IUFindOnSwitchIndex(&SlewRateSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "SlewRateS is now %s", SlewRateS[currentSlewRateIndex].label);
          SlewRateSP.s = IPS_OK;
          IDSetSwitch(&SlewRateSP, NULL);
std::string cmd1= SlewRateS[currentSlewRateIndex].label;
cmd1 +="B#";
const char* cmd=cmd1.c_str();
sendCommand(cmd);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);

std::string stringi(SlewRateS[currentSlewRateIndex].label);
std::string::size_type sz;
int i_dec = std::stoi (stringi,&sz);
sltr=i_dec/100;
//sltr=(currentUstawSrateIndex+1)*5;
slewrate = sltr*trrate ;
gotorate = sltr*trrate ;
finerate = sltr*trrate ;
trsl =1/sltr;

if (TrackState == SCOPE_SLEWING){
StartSlew(targetRA, targetDEC, SCOPE_SLEWING);}
DEBUGF(INDI::Logger::DBG_SESSION, "sltr is now %f",sltr);
DEBUGF(INDI::Logger::DBG_SESSION, "trsl is now %f",trsl);

          return true;
      }

if (!strcmp(name, TrackStateSP.name))
      {
          // Find out which state is requested by the client
          const char *actionName = IUFindOnSwitchName(states, names, n);
          // If door is the same state as actionName, then we do nothing. i.e. if actionName is DOOR_OPEN and our door is already open, we return
          int currentTrackStateIndex = IUFindOnSwitchIndex(&TrackStateSP);
          if (!strcmp(actionName, TrackStateS[currentTrackStateIndex].name))
          {
             DEBUGF(INDI::Logger::DBG_SESSION, "TrackStateS is already %s", TrackStateS[currentTrackStateIndex].label);
             TrackStateSP.s = IPS_IDLE;
             IDSetSwitch(&TrackStateSP, NULL);
             return true;
          }
           
          // Otherwise, let us update the switch state
          IUUpdateSwitch(&TrackStateSP, states, names, n);
          currentTrackStateIndex = IUFindOnSwitchIndex(&TrackStateSP);
          DEBUGF(INDI::Logger::DBG_SESSION, "TrackStateS is now %s", TrackStateS[currentTrackStateIndex].label);
          TrackStateSP.s = IPS_OK;
          IDSetSwitch(&TrackStateSP, NULL);
const char* onstat= "TRACK_ON";
const char*  cmd;
if (!strcmp(onstat, TrackStateS[currentTrackStateIndex].name)) {cmd="RA0#";
TrackState = SCOPE_TRACKING;}
if (strcmp(onstat, TrackStateS[currentTrackStateIndex].name)) {cmd ="STOP#";
TrackState = SCOPE_IDLE;}
DEBUGF(INDI::Logger::DBG_SESSION, "trstate %s",TrackStateS[currentTrackStateIndex].name);
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);
sendCommand(cmd);


          return true;
      }




#ifdef USE_EQUATORIAL_PE
        if (strcmp(name, "PE_NS") == 0)
        {
            IUUpdateSwitch(&PEErrNSSP, states, names, n);

            PEErrNSSP.s = IPS_OK;

            if (PEErrNSS[DIRECTION_NORTH].s == ISS_ON)
            {
                EqPEN[DEC_AXIS].value += TRACKRATE_SIDEREAL / 3600.0 * GuideRateN[DEC_AXIS].value;
                LOGF_DEBUG("Simulating PE in NORTH direction for value of %g", TRACKRATE_SIDEREAL / 3600.0);
            }
            else
            {
                EqPEN[DEC_AXIS].value -= TRACKRATE_SIDEREAL / 3600.0 * GuideRateN[DEC_AXIS].value;
                LOGF_DEBUG("Simulating PE in SOUTH direction for value of %g", TRACKRATE_SIDEREAL / 3600.0);
            }

            IUResetSwitch(&PEErrNSSP);
            IDSetSwitch(&PEErrNSSP, nullptr);
            IDSetNumber(&EqPENV, nullptr);

            return true;
        }

        if (strcmp(name, "PE_WE") == 0)
        {
            IUUpdateSwitch(&PEErrWESP, states, names, n);

            PEErrWESP.s = IPS_OK;

            if (PEErrWES[DIRECTION_WEST].s == ISS_ON)
            {
                EqPEN[RA_AXIS].value -= TRACKRATE_SIDEREAL / 3600.0 / 15. * GuideRateN[RA_AXIS].value;
                LOGF_DEBUG("Simulator PE in WEST direction for value of %g", TRACKRATE_SIDEREAL / 3600.0);
            }
            else
            {
                EqPEN[RA_AXIS].value += TRACKRATE_SIDEREAL / 3600.0 / 15. * GuideRateN[RA_AXIS].value;
                LOGF_DEBUG("Simulator PE in EAST direction for value of %g", TRACKRATE_SIDEREAL / 3600.0);
            }

            IUResetSwitch(&PEErrWESP);
            IDSetSwitch(&PEErrWESP, nullptr);
            IDSetNumber(&EqPENV, nullptr);

            return true;
        }
#endif
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool ArduinoST4::Abort()
{
sendCommand("RA0#");
        LOG_ERROR("Abort move");
sendCommand("DEC0#");
    return true;
}

bool ArduinoST4::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }
if (command == MOTION_START){
if (dir ==DIRECTION_NORTH) {
	if (strona == 0) {sendCommand("PN#");}
	if (strona == 1) {sendCommand("PD#");}
}
if (dir ==DIRECTION_SOUTH) {
	if (strona == 1) {sendCommand("PN#");}
	if (strona == 0) {sendCommand("PD#");}
}}
if (command == MOTION_STOP){sendCommand("DEC0#");}
    return true;
}

bool ArduinoST4::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }
if (command == MOTION_START){
    if (dir ==DIRECTION_EAST) {sendCommand("WS#");}
    if (dir ==DIRECTION_WEST) {sendCommand("ZA#");}
}
if (command == MOTION_STOP){sendCommand("RA0#");}
    return true;
}

bool ArduinoST4::ardMoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command, double dms)
{LOGF_ERROR("gotoCMDs: %f", dms);
uint32_t   ms = dms+0.5;
LOGF_ERROR("gotoNS: %f", ms);
LOGF_ERROR("gotoNS: %d", ms);
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

if (dir ==DIRECTION_NORTH) {
	    if (GuideNSTID)
    {
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    if (sendCommand("PN#") == false)
        return IPS_ALERT;

    guideDirection = ARD_N;
    GuideNSTID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperN, this);

}
if (dir ==DIRECTION_SOUTH) {

	    if (GuideNSTID)
    {
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    if (sendCommand("PD#") == false)
        return IPS_ALERT;

    guideDirection = ARD_S;
    GuideNSTID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperS, this);}
    return true;
}

bool ArduinoST4::ardMoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command, double dms)
{LOGF_ERROR("gotoCMDs: %f", dms);
uint32_t   ms = dms+0.5;
LOGF_ERROR("gotoCMDf: %f", ms);
LOGF_ERROR("gotowsgg: %d", ms);

    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }
if (command == MOTION_START){
    if (dir ==DIRECTION_EAST) {	    

	if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    if (sendCommand("WS#") == false)
        return IPS_ALERT;

    guideDirection = ARD_E;
    GuideWETID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperE, this);
}
    if (dir ==DIRECTION_WEST) {
if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    if (sendCommand("ZA#") == false)
        return IPS_ALERT;

    guideDirection = ARD_W;
    GuideWETID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperW, this);}
}
    return true;
}


bool ArduinoST4::SetCurrentPark()
{
    SetAxis1Park(currentRA);
    SetAxis2Park(currentDEC);
Zaparkuj();
    return true;
}

bool ArduinoST4::SetDefaultPark()
{
    // By default set RA to HA
 //   SetAxis1Park(get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));

    // Set DEC to 90 or -90 depending on the hemisphere
 //   SetAxis2Park((LocationN[LOCATION_LATITUDE].value > 0) ? 90 : -90);
Wyparkuj();
    return true;
}

bool ArduinoST4::SetTrackMode(uint8_t mode)
{
    INDI_UNUSED(mode);
    return true;
}

bool ArduinoST4::SetTrackEnabled(bool enabled)
{
 
     if (enabled)
     {
sendCommand("RA0#");
     }
     else
     {
sendCommand("STOP#");
     }
     return true;
 }

bool ArduinoST4::SetTrackRate(double raRate, double deRate)
{
    INDI_UNUSED(raRate);
    INDI_UNUSED(deRate);
    return true;
}

/**************************************************************************************
** Client is asking us to guide
***************************************************************************************/

IPState ArduinoST4::GuideNorth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: N %.0f ms", ms);

    guiderNSTarget[GUIDE_NORTH] = ms;
    guiderNSTarget[GUIDE_SOUTH] = 0;
    if (GuideNSTID)
    {        LOG_ERROR("Abort moveGuideNSTID");
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    if (sendCommand("DEC+#") == false)
        return IPS_ALERT;

    guideDirection = ARD_N;
    GuideNSTID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperN, this);
    return IPS_BUSY;
}

IPState ArduinoST4::GuideSouth(uint32_t ms)
{
    LOGF_DEBUG("Guiding: S %.0f ms", ms);

    if (GuideNSTID)
    {
        IERmTimer(GuideNSTID);
        GuideNSTID = 0;
    }

    if (sendCommand("DEC-#") == false)
        return IPS_ALERT;

    guideDirection = ARD_S;
    GuideNSTID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperS, this);
    return IPS_BUSY;
}

IPState ArduinoST4::GuideEast(uint32_t ms)
{
    LOGF_DEBUG("Guiding: E %.0f ms", ms);

    if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    if (sendCommand("RA+#") == false)
        return IPS_ALERT;

    guideDirection = ARD_E;
    GuideWETID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperE, this);
    return IPS_BUSY;
}

IPState ArduinoST4::GuideWest(uint32_t ms)
{
    LOGF_DEBUG("Guiding: E %.0f ms", ms);

    if (GuideWETID)
    {
        IERmTimer(GuideWETID);
        GuideWETID = 0;
    }

    if (sendCommand("RA-#") == false)
        return IPS_ALERT;

    guideDirection = ARD_E;
    GuideWETID      = IEAddTimer(static_cast<int>(ms), guideTimeoutHelperE, this);
    return IPS_BUSY;
}

//GUIDE The timer helper functions.
void ArduinoST4::guideTimeoutHelperN(void *p)
{
    static_cast<ArduinoST4 *>(p)->guideTimeout(ARD_N);
}
void ArduinoST4::guideTimeoutHelperS(void *p)
{
    static_cast<ArduinoST4 *>(p)->guideTimeout(ARD_S);
}
void ArduinoST4::guideTimeoutHelperW(void *p)
{
    static_cast<ArduinoST4 *>(p)->guideTimeout(ARD_W);
}
void ArduinoST4::guideTimeoutHelperE(void *p)
{
    static_cast<ArduinoST4 *>(p)->guideTimeout(ARD_E);
}

void ArduinoST4::guideTimeout(ARDUINO_DIRECTION direction)
{
    if (direction == ARD_N || direction == ARD_S)
    {
        if (sendCommand("DEC0#"))
        {
            GuideNSNP.s = IPS_IDLE;
            LOG_DEBUG("Guiding: DEC axis stopped.");
        }
        else
        {
            GuideNSNP.s = IPS_ALERT;
            LOG_ERROR("Failed to stop DEC axis.");
        }

        GuideNSTID            = 0;
        GuideNSNP.np[0].value = 0;
        GuideNSNP.np[1].value = 0;
        IDSetNumber(&GuideNSNP, nullptr);
    }

    if (direction == ARD_W || direction == ARD_E)
    {
        if (sendCommand("RA0#"))
        {
            GuideWENP.s = IPS_IDLE;
            LOG_DEBUG("Guiding: RA axis stopped.");
        }
        else
        {
            LOG_ERROR("Failed to stop RA axis.");
            GuideWENP.s = IPS_ALERT;
        }

        GuideWENP.np[0].value = 0;
        GuideWENP.np[1].value = 0;
        GuideWETID            = 0;
        IDSetNumber(&GuideWENP, nullptr);
    }
}



bool ArduinoST4::sendCommand(const char *cmd)
{
    int nbytes_read=0, nbytes_written=0, tty_rc = 0;
    char res[8] = {0};
DEBUGF(INDI::Logger::DBG_SESSION, "cmd is now %s",cmd);

    if (!isSimulation())
    {
        tcflush(3, TCIOFLUSH);
        if ( (tty_rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
//LOGF_DEBUG("Serial write error: %s", errorMessage);
            return false;
        }
    }

    if (isSimulation())
    {
        strncpy(res, "OK#", 8);
        nbytes_read = 3;
    }
/*****************
    else
    {
        if ( (tty_rc = tty_read_section(PortFD, res, '#', ARDUINO_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial read error: %s", errorMessage);
            return false;
        }
    }
*****************/

    res[nbytes_read - 1] = '\0';
    DEBUGF(INDI::Logger::DBG_SESSION,"RES %s", res);

    return true;
}

/*****************
bool ArduinoST4::sendCommand(const char *cmd){
if (isSimulation())
    {

        return true;
    }

driver->sendPulse(cmd);
return true;
}
*****************/

bool ArduinoST4::Zaparkuj()
 {
     ln_hrz_posn horizontalPos;
     // Libnova south = 0, west = 90, north = 180, east = 270
 
     ln_lnlat_posn observer;
     observer.lat = LocationN[LOCATION_LATITUDE].value;
     observer.lng = LocationN[LOCATION_LONGITUDE].value;
     if (observer.lng > 180)
         observer.lng -= 360;
 
     ln_equ_posn equatorialPos;
     equatorialPos.ra  = currentRA * 15;
     equatorialPos.dec = currentDEC;
     ln_get_hrz_from_equ(&equatorialPos, &observer, ln_get_julian_from_sys(), &horizontalPos);
 
 //    parkAZ = horizontalPos.az - 180;
   //  if (parkAZ < 0)
  //       parkAZ += 360;
parkAZ = horizontalPos.az;
      parkAlt = horizontalPos.alt;
 
     char AzStr[16], AltStr[16];
     fs_sexa(AzStr, parkAZ, 2, 3600);
     fs_sexa(AltStr, parkAlt, 2, 3600);
 DEBUGF(INDI::Logger::DBG_SESSION, "Setting current parking position to coordinates Az (%s) Alt (%s)", AzStr, AltStr);
  //   LOGF_ERROR("Setting current parking position to coordinates Az (%s) Alt (%s)", AzStr, AltStr);

    ofstream outFile;
    outFile.open ("/home/astro/parkingdata.txt", ios::out);
    outFile << parkAZ << " " << parkAlt << endl;
 outFile.close();


 
     return true;
 }

bool ArduinoST4::Wyparkuj()
 {
 ln_hrz_posn horizontalPos;
    ifstream inFile;
    inFile.open("/home/astro/parkingdata.txt", ios::out);
    inFile >> horizontalPos.az >> horizontalPos.alt;
    inFile.close();
 DEBUGF(INDI::Logger::DBG_SESSION,"plik coordinates az (%f) alt (%f)", horizontalPos.az, horizontalPos.alt);
 
    
     // Libnova south = 0, west = 90, north = 180, east = 270
    // horizontalPos.az  = parkAZ;
    // horizontalPos.alt = parkAlt;
 
     ln_lnlat_posn observer;
     observer.lat = LocationN[LOCATION_LATITUDE].value;
     observer.lng = LocationN[LOCATION_LONGITUDE].value;
//LOGF_ERROR("Setting current unparking position to coordinates az (%f) alt (%f)", observer.lat, observer.lng);
     if (observer.lng > 180)
         observer.lng -= 360;
 
     ln_equ_posn equatorialPos;

     ln_get_equ_from_hrz(&horizontalPos, &observer, ln_get_julian_from_sys(), &equatorialPos);
 
     double parkRA = equatorialPos.ra;
 //    if (parkAZ < 0)
 //        parkAZ += 360;
     double parkDEC = equatorialPos.dec;
 
     char RAStr[16], DECStr[16];
     fs_sexa(RAStr, parkRA, 2, 3600);
     fs_sexa(DECStr, parkDEC, 2, 3600);
currentRA = parkRA/15;
currentDEC = parkDEC;
 
 //   LOGF_ERROR("plik to coordinates RA (%s) DEC (%s)", RAStr, DECStr);
 

 
     return true;
 }


