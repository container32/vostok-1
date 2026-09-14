# Nasal scripts for Vostok-1 dialogs
# Thorsten Renk 2017

var ap_dlg = gui.Dialog.new("/sim/gui/dialogs/Vostok-1/ap/dialog","Aircraft/Vostok-1/Dialogs/ap.xml");


var config_dlg = gui.Dialog.new("/sim/gui/dialogs/Vostok-1/config/dialog","Aircraft/Vostok-1/Dialogs/config.xml");


var update_inclination = func {

# do not accept updates if guidance is running



if (getprop("/fdm/jsbsim/systems/autopilot/autolaunch-active") == 1) {return;}



var raw = getprop("/sim/gui/dialogs/Vostok-1/auto_launch/inclination");
var lat = getprop("/position/latitude-deg");

var inc = lat + (90 - lat) * raw;

setprop("/fdm/jsbsim/systems/ap/launch/inclination-target", inc);

var la_raw =  math.asin(math.cos(inc * math.pi/180.0) / math.cos (lat * math.pi/180.0));

#print ("Raw: ", la_raw * 180/math.pi);

# approx correction due to Earth's rotation
var v_orbit = 7700; # 300 km orbit
var v_earth = 465;
var v_xe = v_orbit * math.sin(la_raw) - v_earth * math.cos(lat * math.pi/180.0);
var v_ye = v_orbit * math.cos(la_raw);

#print("vx: ", v_xe, "vy: ", v_ye);

var la_north = math.atan2(v_xe,v_ye);

la_north = 180.0/math.pi * la_north;

var la_south = 180.0 - la_north;

setprop("/sim/gui/dialogs/Vostok-1/auto_launch/azimuth-north", la_north);
setprop("/sim/gui/dialogs/Vostok-1/auto_launch/azimuth-south", la_south);

var direction = getprop("/sim/gui/dialogs/Vostok-1/auto_launch/select-north");


if (direction == 1)
	{
	var launch_azimuth = la_north;
	}
else
	{
	var launch_azimuth = la_south;
	}


setprop("/fdm/jsbsim/systems/autopilot/roll-target", launch_azimuth);



var apoapsis_target = getprop("/sim/gui/dialogs/Vostok-1/auto_launch/apoapsis-target-km");


setprop("/fdm/jsbsim/systems/autopilot/apoapsis-target-km", apoapsis_target);

}



setlistener("/sim/gui/dialogs/Vostok-1/auto_launch/apoapsis-target-miles", update_inclination);
setlistener("/sim/gui/dialogs/Vostok-1/auto_launch/inclination", update_inclination);
