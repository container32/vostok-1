
# Simulation of co-orbiting discarded stages for Vostok-1
# Thorsten Renk 2017 based on code developed for the Space Shuttle


var ft_to_m = 0.30480;
var m_to_ft = 1.0/ft_to_m;
var earth_rotation_deg_s = 0.0041666666666666;

# hashes for managin separated TDU

var tduState = {};
var tduCoord = {};
var tduModel = {};

var tdu_loop_flag = 0;
var tdu_offset_vec = [];


# hashes for managing separated 3rd stage

var tsState = {};
var tsCoord = {};
var tsModel = {};

var ts_loop_flag = 0;
var ts_offset_vec = [];

# hashes for managing separated 2nd stage

var ssState = {};
var ssCoord = {};
var ssModel = {};

var ss_loop_flag = 0;
var ss_offset_vec = [];


###########################################################################
# basic state vector management for imposed external accelerations
###########################################################################

var stateVector = {
	new: func(x,y,z,vx,vy,vz,yaw,pitch,roll) {
	        
		var s = { parents: [stateVector] };
		
	    s.x = x;
		s.y = y;
		s.z = z;
		s.vx = vx;
		s.vy = vy;
		s.vz = vz;
		s.yaw = yaw;
		s.pitch = pitch;
		s.roll = roll;
		s.yaw_rate = 0.0;
		s.pitch_rate = 0.0;
		s.roll_rate = 0.0;
		return s;
	},
	update: func (ax, ay, az, a_yaw, a_pitch, a_roll, dt = nil) {

		if (dt == nil)
			{
			#var speedup = getprop("/sim/speed-up");
			dt = getprop("/sim/time/delta-sec");# * speedup;
			}

		me.x = me.x + me.vx * dt + 0.5 * ax * dt * dt;
		me.y = me.y + me.vy * dt + 0.5 * ay * dt * dt;
		me.z = me.z + me.vz * dt + 0.5 * az * dt * dt;

		me.vx = me.vx + ax * dt;
		me.vy = me.vy + ay * dt;
		me.vz = me.vz + az * dt;
	
		var croll = math.cos(me.roll * math.pi/180.0);
		var sroll = math.sin(me.roll * math.pi/180.0);
		var cpitch = math.cos(me.pitch * math.pi/180.0);
		var spitch = math.sin(me.pitch * math.pi/180.0);

		me.yaw_rate = me.yaw_rate + a_yaw * dt;
		me.pitch_rate = me.pitch_rate + a_pitch * dt;
		me.roll_rate = me.roll_rate + a_roll * dt;

		me.yaw = me.yaw + me.yaw_rate * dt * croll - me.pitch_rate * dt *  sroll;
		me.pitch = me.pitch + me.pitch_rate * dt * croll + me.yaw_rate * dt * sroll;
		me.roll = me.roll + me.roll_rate * dt - me.yaw_rate * dt * spitch * croll + me.pitch_rate * dt * spitch * sroll ;

		#print(me.pitch, " ", me.yaw, " ", me.roll);

		#print(a_yaw, " ", me.yaw_rate, " ", me.yaw);
	},
	
  
};

#####################################################################################
# this function provides the effect of three concatenated rotations in the Tait-Bryan 
# convention (better known as yaw, pitch and roll) on a given vector
# i.e. transforms from body coords to world
####################################################################################

var orientTaitBryan = func (v, h, p, r) {

var heading_rad = h * math.pi/180.0;
var pitch_rad = p * math.pi/180.0;
var roll_rad = r * math.pi/180.0;

var c1 = math.cos(heading_rad);
var s1 = math.sin(heading_rad);

var c2 = math.cos(pitch_rad);
var s2 = math.sin(pitch_rad);

var c3 = math.cos(roll_rad);
var s3 = math.sin(roll_rad);

var x = v[0];
var y = v[1];
var z = v[2];

var x1 = x * (c1 * c2) + y * (c1 * s2 * s3 - c3 * s1) + z * (-s1 * s3 - c1 * c3 * s2);
var y1 = x * (c2 * s1) + y * (c1 * c3 + s1 * s2 * s3) + z * (-c3 * s1 * s2 + c1 * s3);
var z1 = x * s2 + y * (-c2 * s3) + z * c2 * c3;

var out = [];

append(out, x1);
append(out, y1);
append(out, z1);

return out;
}

var orientTaitBryanPassive = func (v, h, p, r) {

var heading_rad = h * math.pi/180.0;
var pitch_rad = p * math.pi/180.0;
var roll_rad = r * math.pi/180.0;

var c1 = math.cos(heading_rad);
var s1 = math.sin(heading_rad);

var c2 = math.cos(pitch_rad);
var s2 = math.sin(pitch_rad);

var c3 = math.cos(roll_rad);
var s3 = math.sin(roll_rad);

var x = v[0];
var y = v[1];
var z = v[2];

var x1 = x * (c1 * c2) + y * (c2 * s1) + z * s2;
var y1 = x * (c1 * s2 * s3 - c3 * s1) + y * (c1 * c3 + s1 * s2 * s3) + z * (-c2 * s3);
var z1 = x * (-s1 * s3 - c1 * c3 * s2) + y * (-c3 * s1 * s2 + c1 * s3) + z * c2 * c3;




var out = [];

append(out, x1);
append(out, y1);
append(out, z1);

return out;
}


###########################################################################
# generic functions used by co-orbiting object routines
###########################################################################

var get_force = func (objState, vostokCoord) {


var G = [objState.x, objState.y, objState.z]; 
var Gnorm = math.sqrt(math.pow(G[0],2.0) + math.pow(G[1],2.0) + math.pow(G[2],2.0));
var g = getprop("/fdm/jsbsim/accelerations/gravity-ft_sec2") * 0.3048 * 1.00015;
G[0] = -G[0]/Gnorm * g;
G[1] = -G[1]/Gnorm * g;
G[2] = -G[2]/Gnorm * g;


# compensating acceleration to dampen the drift error by coordinate trafo
# this might actually be non-spherical gravity of JSBSim

var sin_lat = math.sin(vostokCoord.lat() * 3.1415/180.0);
var cos_lat = math.cos(vostokCoord.lat() * 3.1515/180.0);
var sin_lon = math.sin(vostokCoord.lon() * 3.1415/180.0);
var cos_lon = math.cos(vostokCoord.lon() * 3.1515/180.0);

A_mag = 0.0243 * sin_lat * cos_lat;



var A = [A_mag * cos_lon * sin_lat, A_mag * sin_lon * sin_lat, -A_mag * cos_lat];

var F = [G[0] + A[0], G[1] + A[1], G[2] + A[2]];

return F;
}


var set_coords = func (objString, objCoord, objState) {

var lon = getprop("/position/longitude-deg");
var groundtrack = getprop("/fdm/jsbsim/systems/calculations/groundtrack-course-deg");
var groundtrack_orig = getprop("/controls/vostok/"~objString~"/groundtrack-orig-deg");

var yaw_correction = groundtrack - groundtrack_orig;

setprop("/controls/vostok/"~objString~"/latitude-deg", objCoord.lat());
setprop("/controls/vostok/"~objString~"/longitude-deg", objCoord.lon());
setprop("/controls/vostok/"~objString~"/elevation-ft", objCoord.alt() * m_to_ft);
setprop("/controls/vostok/"~objString~"/pitch-deg", objState.pitch + lon);
setprop("/controls/vostok/"~objString~"/heading-deg", objState.yaw + yaw_correction);

}


var compute_state_correction = func (objState, objCoord, vostokCoord, v_offset_vec, delta_lon) {

objState.vx = objState.vx - v_offset_vec[0];
objState.vy = objState.vy - v_offset_vec[1];
objState.vz = objState.vz - v_offset_vec[2];

objCoord.set_xyz(vostokCoord.x(), vostokCoord.y(), vostokCoord.z());
objCoord.set_lon(objCoord.lon() + delta_lon);

objState.x = objCoord.x();
objState.y = objCoord.y();
objState.z = objCoord.z();

return objState;
}





var place_model = func(string, path, lat, lon, alt, heading, pitch, roll) {



var m = props.globals.getNode("models", 1);
		for (var i = 0; 1; i += 1)
			if (m.getChild("model", i, 0) == nil)
				break;
var model = m.getChild("model", i, 1);



setprop("/controls/vostok/"~string~"/latitude-deg", lat);
setprop("/controls/vostok/"~string~"/longitude-deg", lon);
setprop("/controls/vostok/"~string~"/elevation-ft", alt);
setprop("/controls/vostok/"~string~"/heading-deg", heading);
setprop("/controls/vostok/"~string~"/pitch-deg", pitch);
setprop("/controls/vostok/"~string~"/roll-deg", roll);

var groundtrack = getprop("/fdm/jsbsim/systems/calculations/groundtrack-course-deg");
setprop("/controls/vostok/"~string~"/groundtrack-orig-deg", groundtrack);


var tduModel = props.globals.getNode("/controls/vostok/"~string, 1);
var latN = tduModel.getNode("latitude-deg",1);
var lonN = tduModel.getNode("longitude-deg",1);
var altN = tduModel.getNode("elevation-ft",1);
var headN = tduModel.getNode("heading-deg",1);
var pitchN = tduModel.getNode("pitch-deg",1);
var rollN = tduModel.getNode("roll-deg",1);



model.getNode("path", 1).setValue(path);
model.getNode("latitude-deg-prop", 1).setValue(latN.getPath());
model.getNode("longitude-deg-prop", 1).setValue(lonN.getPath());
model.getNode("elevation-ft-prop", 1).setValue(altN.getPath());
model.getNode("heading-deg-prop", 1).setValue(headN.getPath());
model.getNode("pitch-deg-prop", 1).setValue(pitchN.getPath());
model.getNode("roll-deg-prop", 1).setValue(rollN.getPath());
model.getNode("load", 1).remove();


return model;
}


###########################################################################
# dropped TDU routines
###########################################################################

var init_tdu_ballistic = func {


var pitch = getprop("/orientation/pitch-deg");
var yaw =getprop("/orientation/heading-deg");
var roll = getprop("/orientation/roll-deg");

var lon = getprop("/position/longitude-deg");


tduCoord = geo.aircraft_position() ;

#print(tduCoord.x(), " ", tduCoord.y, " ", tduCoord.z);


tduState = stateVector.new (tduCoord.x(),tduCoord.y(),tduCoord.z(),0,0,0,yaw, pitch - lon, roll);

tduModel = place_model("tdu-ballistic", "Aircraft/Vostok-1/Models/Vostok-1-TDU-ballistic.xml", tduCoord.lat(), tduCoord.lon(), tduCoord.alt() * m_to_ft, yaw,pitch,roll);



var lat = getprop("/position/latitude-deg") * math.pi/180.0;
var lon = getprop("/position/longitude-deg") * math.pi/180.0;
var dt = getprop("/sim/time/delta-sec");

var vxoffset = 3.5 * math.cos(lon) * math.pow(dt/0.05,3.0);
var vyoffset = 3.5 * math.sin(lon) * math.pow(dt/0.05,3.0);
var vzoffset = 0.0;


settimer(func { 
		tduState.vx = getprop("/fdm/jsbsim/velocities/eci-x-fps") * ft_to_m + vxoffset;
		tduState.vy = getprop("/fdm/jsbsim/velocities/eci-y-fps") * ft_to_m + vyoffset;
		tduState.vz = getprop("/fdm/jsbsim/velocities/eci-z-fps") * ft_to_m + vzoffset;
		tdu_loop_flag = 1;
		update_tdu(0.0); },0);

}

var update_tdu = func (delta_lon) {

var vostokCoord = geo.aircraft_position();
var dt = getprop("/sim/time/delta-sec");# * getprop("/sim/speed-up");


delta_lon = delta_lon + dt * earth_rotation_deg_s * 1.004;

var F = get_force (tduState, vostokCoord);
tduState.update(F[0], F[1], F[2], 0.0,0.0,0.0);
tduCoord.set_xyz(tduState.x, tduState.y, tduState.z);
tduCoord.set_lon(tduCoord.lon() - delta_lon);


if (tdu_loop_flag < 3)
	{
	if (tdu_loop_flag ==1)
		{
		tdu_offset_vec = [tduCoord.x()-vostokCoord.x(), tduCoord.y()-vostokCoord.y(),tduCoord.z()-vostokCoord.z()];
		}
	if (tdu_loop_flag == 2)
		{
		var offset1_vec = [tduCoord.x()-vostokCoord.x(), tduCoord.y()-vostokCoord.y(),tduCoord.z()-vostokCoord.z()];
		var v_offset_vec = [(offset1_vec[0] - tdu_offset_vec[0]) / dt, (offset1_vec[1] - tdu_offset_vec[1]) / dt, (offset1_vec[2] - tdu_offset_vec[2]) / dt];
		#print(v_offset_vec[0], " ", v_offset_vec[1], " ", v_offset_vec[2]);


		tduState = compute_state_correction  (tduState, tduCoord, vostokCoord, v_offset_vec, delta_lon);


		#tduCoord.set_lon(tduCoord.lon() - delta_lon);
		}
	tdu_loop_flag = tdu_loop_flag + 1;


	}

set_coords("tdu-ballistic", tduCoord, tduState);

var dist = vostokCoord.distance_to(tduCoord);
if (dist > 5000.0) 
	{
	print ("TDU simulation ends");
	tduModel.remove();
	tdu_loop_flag = 0;
	}

if (tdu_loop_flag >0 ) {settimer(func{ update_tdu(delta_lon);} ,0.0);}
}


###########################################################################
# dropped 3rd stage routines
###########################################################################

var init_ts_ballistic = func {


var pitch = getprop("/orientation/pitch-deg");
var yaw =getprop("/orientation/heading-deg");
var roll = getprop("/orientation/roll-deg");

var lon = getprop("/position/longitude-deg");


tsCoord = geo.aircraft_position() ;



tsState = stateVector.new (tsCoord.x(),tsCoord.y(),tsCoord.z(),0,0,0,yaw, pitch - lon, roll);

tsModel = place_model("ts-ballistic", "Aircraft/Vostok-1/Models/Vostok-1-Stage-3-ballistic.xml", tsCoord.lat(), tsCoord.lon(), tsCoord.alt() * m_to_ft, yaw,pitch,roll);



var lat = getprop("/position/latitude-deg") * math.pi/180.0;
var lon = getprop("/position/longitude-deg") * math.pi/180.0;
var dt = getprop("/sim/time/delta-sec");

var vxoffset = 3.5 * math.cos(lon) * math.pow(dt/0.05,3.0);
var vyoffset = 3.5 * math.sin(lon) * math.pow(dt/0.05,3.0);
var vzoffset = 0.0;


settimer(func { 
		tsState.vx = getprop("/fdm/jsbsim/velocities/eci-x-fps") * ft_to_m + vxoffset;
		tsState.vy = getprop("/fdm/jsbsim/velocities/eci-y-fps") * ft_to_m + vyoffset;
		tsState.vz = getprop("/fdm/jsbsim/velocities/eci-z-fps") * ft_to_m + vzoffset;
		ts_loop_flag = 1;
		update_ts(0.0); },0);

}

var update_ts = func (delta_lon) {

var vostokCoord = geo.aircraft_position();
var dt = getprop("/sim/time/delta-sec");# * getprop("/sim/speed-up");


delta_lon = delta_lon + dt * earth_rotation_deg_s * 1.004;

var F = get_force (tsState, vostokCoord);
tsState.update(F[0], F[1], F[2], 0.0,0.0,0.0);
tsCoord.set_xyz(tsState.x, tsState.y, tsState.z);
tsCoord.set_lon(tsCoord.lon() - delta_lon);


if (ts_loop_flag < 3)
	{
	if (ts_loop_flag ==1)
		{
		ts_offset_vec = [tsCoord.x()-vostokCoord.x(), tsCoord.y()-vostokCoord.y(),tsCoord.z()-vostokCoord.z()];
		}
	if (ts_loop_flag == 2)
		{
		var offset1_vec = [tsCoord.x()-vostokCoord.x(), tsCoord.y()-vostokCoord.y(),tsCoord.z()-vostokCoord.z()];
		var v_offset_vec = [(offset1_vec[0] - ts_offset_vec[0]) / dt, (offset1_vec[1] - ts_offset_vec[1]) / dt, (offset1_vec[2] - ts_offset_vec[2]) / dt];
		#print(v_offset_vec[0], " ", v_offset_vec[1], " ", v_offset_vec[2]);


		tsState = compute_state_correction  (tsState, tsCoord, vostokCoord, v_offset_vec, delta_lon);


		#tsCoord.set_lon(tsCoord.lon() - delta_lon);
		}
	ts_loop_flag = ts_loop_flag + 1;


	}

set_coords("ts-ballistic", tsCoord, tsState);

var dist = vostokCoord.distance_to(tsCoord);
if (dist > 5000.0) 
	{
	print ("Third stage simulation ends");
	tsModel.remove();
	ts_loop_flag = 0;
	}

if (ts_loop_flag >0 ) {settimer(func{ update_ts(delta_lon);} ,0.0);}
}


###########################################################################
# dropped 2nd stage routines
###########################################################################

var init_ss_ballistic = func {


var pitch = getprop("/orientation/pitch-deg");
var yaw =getprop("/orientation/heading-deg");
var roll = getprop("/orientation/roll-deg");

var lon = getprop("/position/longitude-deg");


ssCoord = geo.aircraft_position() ;



ssState = stateVector.new (ssCoord.x(),ssCoord.y(),ssCoord.z(),0,0,0,yaw, pitch - lon, roll);

ssModel = place_model("ss-ballistic", "Aircraft/Vostok-1/Models/Vostok-1-Stage-2-ballistic.xml", ssCoord.lat(), ssCoord.lon(), ssCoord.alt() * m_to_ft, yaw,pitch,roll);



var lat = getprop("/position/latitude-deg") * math.pi/180.0;
var lon = getprop("/position/longitude-deg") * math.pi/180.0;
var dt = getprop("/sim/time/delta-sec");

var vxoffset = 3.5 * math.cos(lon) * math.pow(dt/0.05,3.0);
var vyoffset = 3.5 * math.sin(lon) * math.pow(dt/0.05,3.0);
var vzoffset = 0.0;


settimer(func { 
		ssState.vx = getprop("/fdm/jsbsim/velocities/eci-x-fps") * ft_to_m + vxoffset;
		ssState.vy = getprop("/fdm/jsbsim/velocities/eci-y-fps") * ft_to_m + vyoffset;
		ssState.vz = getprop("/fdm/jsbsim/velocities/eci-z-fps") * ft_to_m + vzoffset;
		ss_loop_flag = 1;
		update_ss(0.0); },0);

}

var update_ss = func (delta_lon) {

var vostokCoord = geo.aircraft_position();
var dt = getprop("/sim/time/delta-sec");# * getprop("/sim/speed-up");


delta_lon = delta_lon + dt * earth_rotation_deg_s * 1.004;

var F = get_force (ssState, vostokCoord);
ssState.update(F[0], F[1], F[2], 0.0,0.0,0.0);
ssCoord.set_xyz(ssState.x, ssState.y, ssState.z);
ssCoord.set_lon(ssCoord.lon() - delta_lon);


if (ss_loop_flag < 3)
	{
	if (ss_loop_flag ==1)
		{
		ss_offset_vec = [ssCoord.x()-vostokCoord.x(), ssCoord.y()-vostokCoord.y(),ssCoord.z()-vostokCoord.z()];
		}
	if (ss_loop_flag == 2)
		{
		var offset1_vec = [ssCoord.x()-vostokCoord.x(), ssCoord.y()-vostokCoord.y(),ssCoord.z()-vostokCoord.z()];
		var v_offset_vec = [(offset1_vec[0] - ss_offset_vec[0]) / dt, (offset1_vec[1] - ss_offset_vec[1]) / dt, (offset1_vec[2] - ss_offset_vec[2]) / dt];
		#print(v_offset_vec[0], " ", v_offset_vec[1], " ", v_offset_vec[2]);


		ssState = compute_state_correction  (ssState, ssCoord, vostokCoord, v_offset_vec, delta_lon);


		#ssCoord.set_lon(ssCoord.lon() - delta_lon);
		}
	ss_loop_flag = ss_loop_flag + 1;


	}

set_coords("ss-ballistic", ssCoord, ssState);

var dist = vostokCoord.distance_to(ssCoord);
if (dist > 5000.0) 
	{
	print ("Second stage simulation ends");
	ssModel.remove();
	ss_loop_flag = 0;
	}

if (ss_loop_flag >0 ) {settimer(func{ update_ss(delta_lon);} ,0.0);}
}
