TCAS and AI/multiplayer Support
==

XPMP2 provides TCAS blibs and AI/multiplayer data using
[TCAS Override](https://developer.x-plane.com/article/overriding-tcas-and-providing-traffic-information/)
introduced with X-Plane 11.50 Beta 8. Please do read
[that blog post by Laminar](https://developer.x-plane.com/article/overriding-tcas-and-providing-traffic-information/)
as it explains all what's necessary and what also XPMP2 does:

- Try acquiring TCAS control via a call to
  [`XPLMAcquirePlanes`](https://developer.x-plane.com/sdk/XPLMPlanes/#XPLMAcquirePlanes).
  All what follows only happens if XPMP2 was granted access:
- Sets `sim/operation/override/override_TCAS` to 1 to tell X-Plane that it
  will use TCAS Override
- Sets `sim/operation/override/override_multiplayer_map_layer` to 1 to tell
  X-Plane that it shall not draw icons into the map as XPMP2 provides
  a self-defined additional map layer
- All aircraft are sorted by distance (biased by `XPMP2::Aircraft::aiPrio`),
  for each aircraft a number of dataRefs are set, see below
- In a call to `XPLMSetActiveAircraftCount` X-Plane is informed about
  the exact number of provided planes. (This might not be strictly necessary,
  but probably a cleaner approach than to wait 10 cylces for not
  updated dataRefs to be taken out of X-Plane's consideration.)

TCAS Target dataRefs
--

XPMP2 writes the folloing TCAS target dataRefs when it has TCAS control:

`.../tcas/targets/` | XPMP2's value from `XPMP2::Aircraft`
------------------- | --------------------
`modeS_id`          | `modeS_id`
`modeC_code`        | `acRadar.code`
`flight_id`         | what `GetFlightId()` returns: `acInfoTexts.flightNum` or `acInfoTexts.tailNum` or `acInfoTexts.aptFrom`-`acInfoTexts.aptFrom`, whatever is available first; this is a virtual function that you can override in your implementation
`icao_type`         | `acIcaoType`

All dataRefs under `sim/cockpit2/tcas/targets/position/` are filled as expected
and documented except for `weight_on_wheels`, for which no value in
`XPMP2::Aircraft::v` is yet available:

Values are taken from `XPMP2::Aircraft::v`. Compare the indexes as defined
in the `enum DR_VALS` defined in [`XPMPAircraft.h`](html/XPMPAircraft_8h.html):

`.../tcas/targets/position/` | XPMP2's value from `XPMP2::Aircraft`
---------------------------- | ---------------------
`x`, `y`, `z`                | `drawInfo.x`, `drawInfo.y - ac.GetVertOfs()`, `drawInfo.z`
`vx`, `vy`, `vz`             | updated once a second with deltas of respective values in `drawInfo` devided by time passed
`psi`                        | `drawInfo.heading`
`the`                        | `drawInfo.pitch`
`phi`                        | `drawInfo.roll`
`gear_deploy`                | `v.[V_CONTROLS_GEAR_RATIO]`
`flap_ratio`                 | `v.[V_CONTROLS_FLAP_RATIO]`
`flap_ratio2`                | `v.[V_CONTROLS_FLAP_RATIO]`
`speedbrake_ratio`           | `v.[V_CONTROLS_SPEED_BRAKE_RATIO]`
`slat_ratio`                 | `v.[V_CONTROLS_SLAT_RATIO]`
`wing_sweep`                 | `v.[V_CONTROLS_WING_SWEEP_RATIO]`
`throttle`                   | `v.[V_CONTROLS_THRUST_RATIO]`
`yolk_pitch`                 | `v.[V_CONTROLS_YOKE_PITCH_RATIO]`
`yolk_yaw`                   | `v.[V_CONTROLS_YOKE_HEADING_RATIO]`
`yolk_roll`                  | `v.[V_CONTROLS_YOKE_ROLL_RATIO]`
`lights`                     | `v.[V_CONTROLS_TAXI_LITES_ON]`, `v.[V_CONTROLS_LANDING_LITES_ON]`, `v.[V_CONTROLS_BEACON_LITES_ON]`, `v.[V_CONTROLS_STROBE_LITES_ON]`, `v.[V_CONTROLS_NAV_LITES_ON]`
`weight_on_wheels`           | -

Classic AI/Multiplayer support
--

...is provided by X-Plane automatically when feeding above TCAS target dataRefs.
That means: The lower 19 planes are mirrored into the well-known
`sim/multiplayer/position/plane#_*` dataRefs. Plugins still relying on these
dataRefs will mainly "just work". And this _without_ any workarounds like
defining AI Aircraft in X-Plane's settings!

Please note that
[`XPLMCountAircraft`](https://developer.x-plane.com/sdk/XPLMPlanes/#XPLMCountAircraft)
can now return values larger than 20,
up to 64 at the moment.

XPMP2 still makes sure that unused `sim/multiplayer/position/plane#` slots
(ie. if less than 19 planes are shown) are initialised with `_x/y/z` set to `9999999.9`.
They are deliberately _not_ set to `0` if unused, because 0/0/0 is a totally
valid local geographic location.