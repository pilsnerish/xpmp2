/// @file       XPMP2-Sample.cpp
/// @brief      Stress Test for CSL Model Creation
/// @see        https://github.com/TwinFan/XPMP2/issues/23
///
/// @author     Birger Hoppe
/// @copyright  (c) 2020 Birger Hoppe
/// @copyright  Permission is hereby granted, free of charge, to any person obtaining a
///             copy of this software and associated documentation files (the "Software"),
///             to deal in the Software without restriction, including without limitation
///             the rights to use, copy, modify, merge, publish, distribute, sublicense,
///             and/or sell copies of the Software, and to permit persons to whom the
///             Software is furnished to do so, subject to the following conditions:\n
///             The above copyright notice and this permission notice shall be included in
///             all copies or substantial portions of the Software.\n
///             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
///             IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
///             FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
///             AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
///             LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
///             OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
///             THE SOFTWARE.

// Standard C headers
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <ctime>

// Standard C++ headers
#include <random>

// X-Plane SDK
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMProcessing.h"

// Include XPMP2 headers
#include "XPCAircraft.h"
#include "XPMPAircraft.h"
#include "XPMPMultiplayer.h"

#if !XPLM300
	#error This plugin requires version 300 of the SDK
#endif

// Need random numbers
std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_int_distribution<> rndMdl;
std::uniform_int_distribution<long> rndPlanes(5,15);

XPLMFlightLoopID planeCbId = nullptr;

constexpr int TARGET_NUM_AC = 50;
constexpr float HOW_OFTEN = -5.0;       // how often to create/destroy planes? (flight loop return value)

//
// MARK: Utility Functions
//

/// Log a message to X-Plane's Log.txt with sprintf-style parameters
void LogMsg (const char* szMsg, ... )
{
    char buf[512] = "StressTest: ";
    va_list args;
    // Write all the variable parameters
    va_start (args, szMsg);
    std::vsnprintf(buf+12, sizeof(buf)-14, szMsg, args);
    va_end (args);
    std::strcat(buf, "\n");
    // write to log (flushed immediately -> expensive!)
    XPLMDebugString(buf);
}

/// This is a callback the XPMP2 calls regularly to learn about configuration settings.
/// Only 3 are left, all of them integers.
int CBIntPrefsFunc (const char *, [[maybe_unused]] const char * item, int defaultVal)
{
    // We always want to replace dataRefs and textures upon load to make the most out of the .obj files
    if (!strcmp(item, XPMP_CFG_ITM_REPLDATAREFS)) return 1;
    if (!strcmp(item, XPMP_CFG_ITM_REPLTEXTURE)) return 1;      // actually...this is ON by default anyway, just to be sure
    // StressTest: Always DEBUG-level logging, but not model matching
    if (!strcmp(item, XPMP_CFG_ITM_MODELMATCHING)) return 0;
    if (!strcmp(item, XPMP_CFG_ITM_LOGLEVEL)) return 0;       // DEBUG logging level
    // Otherwise we just accept defaults
    return defaultVal;
}

//
// MARK: Helper functions for position calculations
//

/// Distance of our simulated planes to the user's plane's position? [m]
constexpr float PLANE_DIST_M = 200.0f;
/// Radius of the circle the planes do [m]
constexpr float PLANE_RADIUS_M = 100.0f;
/// Altitude difference to stack the 3 planes one above the other [m]
constexpr float PLANE_STACK_ALT_M = 50.0f;
/// Time it shall take to fly/roll a full circle [seconds]
constexpr float PLANE_CIRCLE_TIME_S = 20.0f;
/// Time it shall take to fly/roll a full circle [minutes]
constexpr float PLANE_CIRCLE_TIME_MIN = PLANE_CIRCLE_TIME_S / 60.0f;
/// Engine / prop rotation assumptions: rotations per minute
constexpr float PLANE_PROP_RPM = 300.0f;

/// PI
constexpr double PI         = 3.1415926535897932384626433832795028841971693993751;

/// Summarizes the 3 values of a position in the local coordinate system
struct positionTy {
    double x = 0.0f;
    double y = 0.0f;
    double z = 0.0f;
};

/// Position of user's plane
static XPLMDataRef dr_x = XPLMFindDataRef("sim/flightmodel/position/local_x");      // double
static XPLMDataRef dr_y = XPLMFindDataRef("sim/flightmodel/position/local_y");      // double
static XPLMDataRef dr_z = XPLMFindDataRef("sim/flightmodel/position/local_z");      // double
static XPLMDataRef dr_heading = XPLMFindDataRef("sim/flightmodel/position/psi");    // float
static XPLMDataRef dr_time = XPLMFindDataRef("sim/time/total_running_time_sec");    // float

/// Returns a number between 0.0 and 1.0, increasing over the course of 10 seconds, then restarting
inline float GetTimeFragment ()
{
    const float t = XPLMGetDataf(dr_time);
    return std::fmod(t, PLANE_CIRCLE_TIME_S) / PLANE_CIRCLE_TIME_S;
}

/// Returns a number between 0.0 and 1.0, going up and down over the course of 10 seconds
inline float GetTimeUpDown ()
{
    return std::abs(std::fmod(XPLMGetDataf(dr_time), PLANE_CIRCLE_TIME_S) / (PLANE_CIRCLE_TIME_S/2.0f) - 1.0f);
}

/// Convert from degree to radians
inline double deg2rad (const double deg) { return (deg * PI / 180.0); }

/// Save string copy
inline char* strScpy (char* dest, const char* src, size_t size)
{
    strncpy(dest, src, size);
    dest[size-1] = 0;               // this ensures zero-termination!
    return dest;
}

/// Finds a position 200m in front of the user's plane serving as the center for further operations
positionTy FindCenterPos (float dist)
{
    // Current user's plane's position and heading (relative to Z)
    positionTy pos = {
        XPLMGetDatad(dr_x),
        XPLMGetDatad(dr_y),
        XPLMGetDatad(dr_z)
    };
    float heading = XPLMGetDataf(dr_heading);

    // Move point 200m away from aircraft, direction of its heading
    const double head_rad = deg2rad(heading);
    pos.x += sin(head_rad) * dist;              // east axis
    pos.z -= cos(head_rad) * dist;              // south axis

    return pos;
}

/// Put the position on a circle around itself
void CirclePos (positionTy& pos,
                float heading,
                float radius)
{
    const double head_rad = deg2rad(heading);
    pos.x += radius * sin(head_rad);        // east axis
    pos.z -= radius * cos(head_rad);        // south axis
}

/// Convert local position to world coordinates
void ConvLocalToWorld (const positionTy& pos,
                       double& lat, double& lon, double& alt)
{
    XPLMLocalToWorld(pos.x, pos.y, pos.z,
                     &lat, &lon, &alt);
}

//
// MARK: Using XPMP2 - New XPMP2::Aircraft class
//       This is the new and recommended way of using the library:
//       Deriving a class from XPMP2::Aircraft and providing
//       a custom implementation for UpdatePosition(),
//       which provides all current values in one go directly into
//       the member variables, which are later on used for
//       controlling the plane objects. This avoids any unnecessary copying
//       within the library

using namespace XPMP2;

/// Subclassing XPMP2::Aircraft to create our own class
class SampleAircraft : public Aircraft
{
public:
    double agl = NAN;           // my individual height above ground level
public:
    /// Constructor just passes on all parameters to library
    SampleAircraft(const std::string& _icaoType,
                   const std::string& _icaoAirline,
                   const std::string& _livery,
                   XPMPPlaneID _modeS_id = 0,
                   const std::string& _cslId = "") :
    Aircraft(_icaoType, _icaoAirline, _livery, _modeS_id, _cslId)
    {
        // in our sample implementation, label, radar and info texts
        // are not dynamic. In others, they might be, then update them
        // in UpdatePosition()

        // Label
        label = "XPMP2::Aircraft";
        colLabel[0] = 0.0f;             // green
        colLabel[1] = 1.0f;
        colLabel[2] = 0.0f;

        // Radar
        acRadar.code = 7654;
        acRadar.mode = xpmpTransponderMode_ModeC;

        // informational texts
        strScpy(acInfoTexts.icaoAcType, _icaoType.c_str(), sizeof(acInfoTexts.icaoAcType));
        strScpy(acInfoTexts.icaoAirline, _icaoAirline.c_str(), sizeof(acInfoTexts.icaoAirline));
        strScpy(acInfoTexts.tailNum, "ST-RES", sizeof(acInfoTexts.tailNum));
        
        // Individual random altitude
        static std::uniform_int_distribution<long> rndAlt((long)PLANE_STACK_ALT_M,10*(long)PLANE_STACK_ALT_M);
        agl = (double)rndAlt(gen);
    }

    /// Custom implementation for the virtual function providing updates values
    virtual void UpdatePosition (float, int)
    {
        // Calculate the plane's position
        const float angle = std::fmod(360.0f * GetTimeFragment(), 360.0f);
        positionTy pos = FindCenterPos(PLANE_DIST_M);               // relative to user's plane
        CirclePos(pos, angle, PLANE_RADIUS_M);                      // turning around a circle
        pos.y += agl;
        
        // Set the position the easy way
        drawInfo.x = (float)pos.x;
        drawInfo.y = (float)pos.y + GetVertOfs();
        drawInfo.z = (float)pos.z;

        // further attitude information
        SetPitch(0.0f);
        SetHeading(std::fmod(90.0f + angle, 360.0f));
        SetRoll(20.0f);

        // Plane configuration info
        // This fills a large array of float values:
        const float r = GetTimeUpDown();        // a value between 0 and 1
        SetGearRatio(r);
        SetNoseWheelAngle(r * 90.0f - 45.0f);  // turn nose wheel -45°..+45°
        SetFlapRatio(r);
        SetSpoilerRatio(r);
        SetSpeedbrakeRatio(r);
        SetSlatRatio(r);
        SetWingSweepRatio(0.0f);
        SetThrustRatio(0.5f);
        SetYokePitchRatio(r);
        SetYokeHeadingRatio(r);
        SetYokeRollRatio(r);

        // lights
        SetLightsTaxi(false);
        SetLightsLanding(false);
        SetLightsBeacon(true);
        SetLightsStrobe(true);
        SetLightsNav(true);

        // tires don't roll in the air
        SetTireDeflection(0.0f);
        SetTireRotAngle(0.0f);
        SetTireRotRpm(0.0f);                    // also sets the rad/s value!

        // For simplicity, we keep engine and prop rotation identical...probably unrealistic
        SetEngineRotRpm(1,PLANE_PROP_RPM);        // also sets the rad/s value!
        // 2nd engine shall turn 4 times slower...
        SetEngineRotRpm(2,PLANE_PROP_RPM/4);      // also sets the rad/s value!

        SetPropRotRpm(PLANE_PROP_RPM);          // also sets the rad/s value!

        // Current position of engine / prop: keeps turning as per engine/prop speed:
        float deg = std::fmod(PLANE_PROP_RPM * PLANE_CIRCLE_TIME_MIN * GetTimeFragment() * 360.0f,
                              360.0f);
        SetEngineRotAngle(1,deg);
        // 2nd engine shall turn 4 times slower...
        deg = std::fmod(PLANE_PROP_RPM/4 * PLANE_CIRCLE_TIME_MIN * GetTimeFragment() * 360.0f,
                        360.0f);
        SetEngineRotAngle(2,deg);

        SetPropRotAngle(deg);

        // no reversers and no moment of touch-down in flight
        SetThrustReversRatio(0.0f);
        SetReversDeployRatio(0.0f);
        SetTouchDown(false);
    }

};


// array of smart pointers to the SampleAircraft object
typedef std::unique_ptr<SampleAircraft> SampleAircraftPtr;
typedef std::vector<SampleAircraftPtr> vecAcTy;

// the global vector of all planes we created
static vecAcTy vecAc;

//
// MARK: Menu functionality
//

/// menu id of our plugin's menu
XPLMMenuID hMenu = nullptr;

/// Planes currently visible?
bool gbVisible = true;

/// Labels currently shown in map view?
bool gbMapLabels = true;

/// for cycling CSL models: what is the index used for the first plane?
int gModelIdxBase = 0;

/// Is any plane object created?
inline bool ArePlanesCreated () { return !vecAc.empty(); }

/// How many planes are actually instanciated?
inline long CountInstances ()
{
    return std::count_if(vecAc.cbegin(), vecAc.cend(),
                         [](const SampleAircraftPtr& pAc){return pAc->IsInstanciated();});
}

/// Create one more additional plane
void PlaneCreateOne ()
{
    static std::string mdlName;     // static avoids recreation all the time on the stack
    static std::string mdlIcao;
    static std::string mdlAirline;
    static std::string mdlLivery;
    
    // Which model to pick? (randomly)
    int mdlIdx = rndMdl(gen);
    XPMPGetModelInfo2(mdlIdx, mdlName, mdlIcao, mdlAirline, mdlLivery);
    
    // Create a new plane
    vecAc.emplace_back(std::make_unique<SampleAircraft>(mdlIcao, mdlAirline, mdlLivery));
}

/// Remove an instanciated plane
void PlaneRemoveOne ()
{
    // find first instanciated plane
    vecAcTy::iterator i = std::find_if(vecAc.begin(), vecAc.end(),
                                       [](const SampleAircraftPtr& pAc){return pAc->IsInstanciated();});
    // remove that plane
    if (i != vecAc.end())
        vecAc.erase(i);
}

float PlaneCreationCB (float, float, int, void*)
{
    // Number of existing instaces
    const long numInst = CountInstances();
    // Bail if there are still a lot of instances to be created
    if (numInst < (long)vecAc.size() * 8 / 10)
        return HOW_OFTEN;

    long addCnt = rndPlanes(gen);        // how many planes to add and remove?
    long delCnt = rndPlanes(gen);
    
    // If current numbers of actually created instances
    // are far away from target then push a bit into the right direction
    if (numInst < TARGET_NUM_AC * 8 / 10)        // more than 20% too low
        addCnt += delCnt;                       // ensure net add
    else if (numInst > TARGET_NUM_AC * 12 / 10)  // more than 20% too high
        delCnt += addCnt;                       // ensure net deletion
    
    // Create planes
    while (addCnt-- > 0)
        PlaneCreateOne();
    
    // Remove planes
    while (delCnt-- > 0)
        PlaneRemoveOne();
    
    return HOW_OFTEN;
}

/// Create our initial planes
void PlanesCreate ()
{
    // Create flight loop callback for actual creation of planes
    if (!planeCbId) {
        XPLMCreateFlightLoop_t flDef = {
            sizeof(flDef),
            xplm_FlightLoop_Phase_BeforeFlightModel,
            PlaneCreationCB,
            nullptr
        };
        planeCbId = XPLMCreateFlightLoop(&flDef);
    }
    XPLMScheduleFlightLoop(planeCbId, -10.0, 1);

    // Put a checkmark in front of menu item if planes are visible
    XPLMCheckMenuItem(hMenu, 0, ArePlanesCreated()  ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(hMenu, 1, gbVisible           ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

/// Remove all planes
void PlanesRemove ()
{
    // Remove planes
    vecAc.clear();
    
    // Remove flight loop
    if (planeCbId) {
        XPLMDestroyFlightLoop(planeCbId);
        planeCbId = nullptr;
    }

    // Remove the checkmark in front of menu item
    XPLMCheckMenuItem(hMenu, 0, xplm_Menu_Unchecked);
    XPLMCheckMenuItem(hMenu, 1, xplm_Menu_Unchecked);
}

/// Show/hide the planes (temporarily, without destroying the plane objects)
void PlanesShowHide ()
{
    gbVisible = !gbVisible;             // toggle setting
    for (SampleAircraftPtr& ptr: vecAc)
        ptr->SetVisible(gbVisible);

    // Put a checkmark in front of menu item if planes are visible
    XPLMCheckMenuItem(hMenu, 1, gbVisible           ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

/// Cycle the CSL models of the 3 planes
void PlanesCycleModels ()
{}

/// @brief Rematch CSL models based on existing definition
/// @details This will pick a different (partly random) CSL model
///          for those planes, for which no exact match has been found.
///          The A321 is defined without operator code, so each re-match
///          will pick any of the available A321 models.
void PlanesRematch ()
{
    for (SampleAircraftPtr& ptr: vecAc)
        ptr->ReMatchModel();
}

void MenuUpdateCheckmarks ()
{
    XPLMCheckMenuItem(hMenu, 0, ArePlanesCreated()           ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(hMenu, 1, gbVisible                    ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    XPLMCheckMenuItem(hMenu, 4, XPMPHasControlOfAIAircraft() ? xplm_Menu_Checked : xplm_Menu_Unchecked);
}

/// Callback function for the case that we might get AI access later
void CPRequestAIAgain (void*)
{
    // Well...we just try again ;-)
    XPMPMultiplayerEnable(CPRequestAIAgain);
    MenuUpdateCheckmarks();
}

/// Callback function for menu
void CBMenu (void* /*inMenuRef*/, void* inItemRef)
{
    // Toggle plane visibility?
    if (inItemRef == (void*)1)
    {
        if (ArePlanesCreated())
            PlanesRemove();
        else
            PlanesCreate();
    }
    // Show/Hide Planes?
    else if (inItemRef == (void*)2)
    {
        PlanesShowHide();
    }
    // Cycle Models?
    else if (inItemRef == (void*)3)
    {
        PlanesCycleModels();
    }
    // Rematch Models?
    else if (inItemRef == (void*)4)
    {
        PlanesRematch();
    }
    // Toggle AI control?
    else if (inItemRef == (void*)5)
    {
        if (XPMPHasControlOfAIAircraft())
            XPMPMultiplayerDisable();
        else
            // When requested by menu we actually wait via callback to get control
            XPMPMultiplayerEnable(CPRequestAIAgain);
    }
    // Update menu items' checkmarks
    MenuUpdateCheckmarks();
}

//
// MARK: Standard Plugin Callbacks
//

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
	std::strcpy(outName, "XPMP2-Stress (StressTest)");
	std::strcpy(outSig, "TwinFan.plugin.XPMP2-Sample-StressTest");
	std::strcpy(outDesc, "Putting some stress on creating/removing instances");

    // use native paths, i.e. Posix style (as opposed to HFS style)
    // https://developer.x-plane.com/2014/12/mac-plugin-developers-you-should-be-using-native-paths/
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS",1);

    // Create the menu for the plugin
    int my_slot = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "XPMP2 Stress", NULL, 0);
    hMenu = XPLMCreateMenu("XPMP2 Stress", XPLMFindPluginsMenu(), my_slot, CBMenu, NULL);
    XPLMAppendMenuItem(hMenu, "Toggle Planes",      (void*)1, 0);
    XPLMAppendMenuItem(hMenu, "Toggle Visibility",  (void*)2, 0);
    XPLMAppendMenuItem(hMenu, "(Cycle Models)",     (void*)3, 0);
    XPLMAppendMenuItem(hMenu, "Rematch Models",     (void*)4, 0);
    XPLMAppendMenuItem(hMenu, "Toggle AI control",  (void*)5, 0);
    MenuUpdateCheckmarks();
    
	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
    // The path separation character, one out of /\:
    char pathSep = XPLMGetDirectorySeparator()[0];
    // The plugin's path, results in something like ".../Resources/plugins/XPMP2-Sample/64/XPMP2-Sample.xpl"
    char szPath[256];
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, szPath, nullptr, nullptr);
    *(std::strrchr(szPath, pathSep)) = 0;   // Cut off the plugin's file name
    *(std::strrchr(szPath, pathSep)+1) = 0; // Cut off the "64" directory name, but leave the dir separation character
    // We search in a subdirectory named "Resources" for all we need
    std::string resourcePath = szPath;
    resourcePath += "Resources";            // should now be something like ".../Resources/plugins/XPMP2-Sample/Resources"

    // Try initializing XPMP2:
    const char *res = XPMPMultiplayerInit ("XPMP2-Stress",          // plugin name,
                                           resourcePath.c_str(),    // path to supplemental files
                                           CBIntPrefsFunc,          // configuration callback function
                                           "C172");                 // default ICAO type
    if (res[0]) {
        LogMsg("Initialization of XPMP2 failed: %s", res);
        return 0;
    }

    // Load our CSL models
    res = XPMPLoadCSLPackage(resourcePath.c_str());     // CSL folder root path
    if (res[0]) {
        LogMsg("Error while loading CSL packages: %s", res);
    }
    
    // Bail if no models
    if (XPMPGetNumberOfInstalledModels() <= 0) {
        LogMsg("No CSL models installed...stress test makes no sense this way!");
        return 0;
    }
    
    // Now that we know the number of models create a random distribution
    rndMdl = std::uniform_int_distribution<>(0, XPMPGetNumberOfInstalledModels()-1);

    // Now we also try to get control of AI planes. That's optional, though,
    // other plugins (like LiveTraffic, XSquawkBox, X-IvAp...)
    // could have control already
    res = XPMPMultiplayerEnable(CPRequestAIAgain);
    if (res[0]) {
        LogMsg("Could not enable AI planes: %s", res);
    }

    // *** Create the planes ***
    PlanesCreate();

    // Success
    MenuUpdateCheckmarks();
    LogMsg("Enabled");
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    // Remove the planes
    PlanesRemove();

    // Give up AI plane control
    XPMPMultiplayerDisable();

    // Properly cleanup the XPMP2 library
    XPMPMultiplayerCleanup();

    LogMsg("Disabled");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, long inMsg, void*)
{
    // Some other plugin wants TCAS/AI control, so we (as an artificial
    // traffic plugin) give up
    if (inMsg == XPLM_MSG_RELEASE_PLANES) {
        XPMPMultiplayerDisable();
        MenuUpdateCheckmarks();
    }
}
