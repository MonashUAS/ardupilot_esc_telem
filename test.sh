#!/bin/bash

set -e
set -x

rm -rf build
ccache --clear
ccache --zero-stats
time (
for x in AR-F407SmartBat ARK_CANNODE ARK_GPS ARK_RTK_GPS AeroFox-Airspeed AeroFox-Airspeed-DLVR AeroFox-GNSS_F9P AeroFox-PMU BirdCANdy C-RTK2-HP CUAV_GPS CarbonixF405 CarbonixL496 CubeBlack-periph CubeOrange-periph CubeOrange-periph-heavy CubePilot-CANMod FreeflyRTK G4-ESC Here4AP HerePro Hitec-Airspeed HitecMosaic HolybroG4_Compass HolybroG4_GPS HolybroGPS MatekH743-periph MatekL431-Airspeed MatekL431-BattMon MatekL431-DShot MatekL431-EFI MatekL431-GPS MatekL431-GPSIN MatekL431-HWTelem MatekL431-Periph MatekL431-Proximity MatekL431-RC MatekL431-Rangefinder MatekL431-bdshot Nucleo-G491 Nucleo-L476 Nucleo-L496 Pixracer-periph Sierra-F405 Sierra-F412 Sierra-F9P Sierra-L431 Sierra-PrecisionPoint Sierra-TrueNavPro Sierra-TrueNorth Sierra-TrueSpeed ZubaxGNSS f103-ADSB f103-Airspeed f103-GPS f103-HWESC f103-QiotekPeriph f103-RangeFinder f103-Trigger f303-GPS f303-HWESC f303-M10025 f303-M10070 f303-MatekGPS f303-PWM f303-TempSensor f303-Universal f405-MatekAirspeed f405-MatekGPS kha_eth mRo-M10095 mRoCANPWM-M10126 rGNSS sw-nav-f405; do
    ./waf configure --board=$x
    ./waf AP_Periph
done
)

ccache --show-stats

