# Auto-launch for Vostok-1
# Thorsten Renk 2017 based on code developed for the Space Shuttle

var auto_launch_loop_flag = 0;
var stage = 0;

var fairing_flag = 0;
var pitchdown_flag = 0;
var apoapsis_tgt = 250.0;



var auto_launch_prestart = func {

	setprop("/fdm/jsbsim/systems/rightswitchpanel/cover-input", 1);
	setprop("/fdm/jsbsim/systems/gascontrol/lock-command", 0);

	apoapsis_tgt = getprop("/sim/gui/dialogs/Vostok-1/auto_launch/apoapsis-target-km");

	settimer (auto_launch_start, 2.0);

}

var auto_launch_start = func {

	#if (getprop("/fdm/jsbsim/systems/autopilot/autolaunch-selected") == 0)
	#	{return;}


	setprop("/fdm/jsbsim/systems/rightswitchpanel/one-two-ignition-input", 1);

	if (auto_launch_loop_flag == 1) {return;}

	print("Starting auto-launch guidance");

	auto_launch_loop_flag = 1;

	#setprop("/fdm/jsbsim/systems/autopilot/roll-target", getprop("/);
	setprop("/fdm/jsbsim/systems/autopilot/pitch-target", 0.0);

	setprop("/controls/engines/engine[0]/throttle", 1);

	auto_launch_loop();

}


var auto_launch_stop = func {
	print("Controls back to manual.");
	auto_launch_loop_flag = 0;
}

var auto_launch_loop = func {

	if (auto_launch_loop_flag == 0) {return;}


	if (stage == 0) # get clear off the ground
		{
		var alt_agl_ft = getprop("/position/altitude-agl-ft");

		if (alt_agl_ft > 300.0) 
			{
			stage = 1;
			setprop("/fdm/jsbsim/systems/autopilot/autolaunch-active", 1);
			}
		}
	else if (stage == 1) # roll to launch azimuth
		{
		var roll_error = getprop("/fdm/jsbsim/systems/autopilot/roll-error");

		if (math.abs(roll_error) < 0.1)
			{
			stage = 2;
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", -35.0);
			}

		}
	else if (stage == 2) # fly through max. qbar
		{

		var qbar=getprop("fdm/jsbsim/aero/qbar-kgm2");
		var alt = getprop("/position/altitude-ft");

		if (qbar > 4900.0)
			{
			setprop("/controls/engines/engine/throttle", 0.65);
			}
		else if (qbar > 3800.0)
			{
			setprop("/controls/engines/engine/throttle", 0.73);
			}

		if (alt > 42000.0)
			{
			stage = 3;
			setprop("/controls/engines/engine/throttle", 1.0);
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", -60.0);
			}

		}
	else if (stage == 3) # fly to booster separation
		{
		var g_force = getprop("fdm/jsbsim/accelerations/Nz");
		var alt = getprop("/position/altitude-ft");

		if ((alt > 165000.0) and (pitchdown_flag == 0))
			{
			pitchdown_flag = 1;
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", -85.0);
			}

		if (g_force > 3.8)
			{
			var throttle = getprop("/controls/engines/engine/throttle");
			throttle = 0.98 * throttle;
			setprop("/controls/engines/engine/throttle", throttle);
			}
		else if (g_force < 2.0)
			{
			stage = 4;
			setprop("/fdm/jsbsim/systems/rightswitchpanel/one-drop-input", 1);
			setprop("/controls/engines/engine/throttle", 1.0);
			setprop("/fdm/jsbsim/systems/autopilot/att-mode", 1);
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", 0.0);
			}

		}
	else if (stage == 4) # fly to second stage separation
		{
		var alt = getprop("/position/altitude-ft");

		if ((alt > 280000.0) and (fairing_flag == 0))
			{
			setprop("/fdm/jsbsim/systems/rightswitchpanel/fairings-drop-input", 1);
			fairing_flag = 1;
			}

		var apoapsis = getprop("/fdm/jsbsim/systems/enginespanel/apoapsis-km");

		if (apoapsis < 130.0)
			{
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", 3.0);
			}
		else if (apoapsis < 150.0)
			{
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", 1.5);
			}
		else
			{
			setprop("/fdm/jsbsim/systems/autopilot/pitch-target", 0.0);
			}

		var g_force = getprop("fdm/jsbsim/accelerations/Nz");

		if (g_force > 3.8)
			{
			var throttle = getprop("/controls/engines/engine/throttle");
			throttle = 0.98 * throttle;
			setprop("/controls/engines/engine/throttle", throttle);
			}
		else if (g_force < 1.0)
			{
			stage = 5;
			setprop("/fdm/jsbsim/systems/rightswitchpanel/three-ignition-input", 1); # hot staging
			setprop("/controls/engines/engine/throttle", 1.0);
			setprop("/fdm/jsbsim/systems/rightswitchpanel/two-drop-input", 1);
			setprop("/fdm/jsbsim/systems/autopilot/att-mode", 2);
			}
		}
	else if (stage == 5) # hold altitude to target apoapsis
		{
		var periapsis = getprop("/fdm/jsbsim/systems/enginespanel/periapsis-km");
		var apoapsis = getprop("/fdm/jsbsim/systems/enginespanel/apoapsis-km");
		var throttle = getprop("/controls/engines/engine/throttle");

		if ((periapsis > 0.0) and (throttle > 0.1))
			{
			throttle = throttle * 0.98;
			setprop("/controls/engines/engine/throttle", throttle);
			}

		if (apoapsis > apoapsis_tgt)
			{
			stage = 6;
			setprop("/fdm/jsbsim/systems/rightswitchpanel/three-drop-input", 1);
			}
		
		}
	else if (stage == 6)
		{
		print ("Autopilot signing off, have a nice flight!");
		
		auto_launch_loop_flag = 0;
		setprop("/fdm/jsbsim/systems/autopilot/autolaunch-active", 0);
		}

	settimer(auto_launch_loop, 1.0);
}