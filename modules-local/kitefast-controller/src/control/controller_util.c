#include "control/controller_util.h"


#include "common/c_math/filter.h"
#include "common/c_math/mat2.h"
#include "common/c_math/vec2.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void min_max_lateral_integrators(CrosswindParams *crosswind){

const Mat2 inter = {{{crosswind->inner.lateral_gains_min_airspeed[0][4], crosswind->inner.lateral_gains_min_airspeed[0][5]},
					 {crosswind->inner.lateral_gains_min_airspeed[1][4], crosswind->inner.lateral_gains_min_airspeed[1][5]}}};

Mat2 inter_inv = {{{0,0},{0,0}}};
printf("I made it to controller_util.c");
const Mat2 *inter_inv_tmp = Mat2Inv(&inter, &inter_inv);
const Vec2 firsthalf 	= {inter_inv.d[0][0], inter_inv.d[0][1]};
const Vec2 sechalf 	= {inter_inv.d[1][0], inter_inv.d[1][1]};
const Vec2 limits = {0.0, 0.3};
double min_lat_integrators[2]; 
double max_lat_integrators[2];
min_lat_integrators[0] = Vec2Dot(&firsthalf, &limits);
min_lat_integrators[1] = Vec2Dot(&sechalf, &limits);
max_lat_integrators[0] = -min_lat_integrators[0];
max_lat_integrators[1] = -min_lat_integrators[1];

crosswind->inner.int_tether_roll_min 	= min_lat_integrators[kCrosswindLateralInputAileron];
crosswind->inner.int_tether_roll_max	= max_lat_integrators[kCrosswindLateralInputAileron];
crosswind->inner.int_beta_min	= min_lat_integrators[kCrosswindLateralInputRudder];
crosswind->inner.int_beta_max 	= max_lat_integrators[kCrosswindLateralInputRudder];

}

void loadPlaybook(CrosswindParams *crosswind){
	// Load playbook parameterized
	Temp_playbook play_load[] = {
		// first entry
		{.airspeed_tmp = {
			29.507373240543348,
    		34.057437078647425,
        	43.840174840031111,
        	44.079763603321808,
        	37.927800701371197,
        	33.4780711107963},
		.alpha_tmp = {
			0.017729262087597869,
        	0.057500987479755475,
        	0.076939499761004407,
        	0.057240099129220498,
        	0.072435511085818097,
        	0.014871379684593136},
		.beta_tmp = {
			0.033667755233262875,
        	0.033667755233262875,
        	0.033667755233262875,
        	0.033667755233262875,
        	0.033667755233262875,
        	0.033667755233262875},
		.azi_offset_tmp = 0.13655005257627995,
		.elevation_tmp = 0.93993696119458892,
		.radius_tmp = 177.35938133482975,
		.wind_speed_tmp = 1.0
		},
		// second entry
		{.airspeed_tmp = {
			30.556099468382499,
        	40.064675729765248,
        	48.643611071824893,
        	45.80678556757308,
        	41.521916677471921,
        	40.875488770337405},
		.alpha_tmp = {
			-0.010262550468967457,
       		0.034407419544235859,
       		0.0077941996925978835,
       		0.073249281591669949,
       		0.014817133435430813,
       		0.018233217998506523},
		.beta_tmp = {
			0.014074802312009277,
        	0.014074802312009277,
        	0.014074802312009277,
        	0.014074802312009277,
        	0.014074802312009277,
        	0.014074802312009277},
		.azi_offset_tmp = 0.14701780942752141,
		.elevation_tmp = 0.82747718517191349,
		.radius_tmp = 172.65114161768233,
		.wind_speed_tmp = 3.0
		},	
		// third entry
		{.airspeed_tmp = {
			33.819744940959929,
        	42.728336314982272,
        	51.443747134004106,
        	53.297777490286052,
        	39.580404546631677,
        	34.270621538783161},
		.alpha_tmp = {
			-0.025611738134539999,
        	0.04828702993027105,
        	0.071709103575379865,
        	0.079452365001361699,
        	0.047501290730409693,
        	0.068949348618474474},
		.beta_tmp = {
			-0.075267793783958897,
        	-0.075267793783958897,
        	-0.075267793783958897,
        	-0.075267793783958897,
        	-0.075267793783958897,
        	-0.075267793783958897},
		.azi_offset_tmp = 0.15657609536040296,
		.elevation_tmp = 0.72161531063268147,
		.radius_tmp = 166.0985889535751,
		.wind_speed_tmp = 5.0
		},
		// fourth entry
		{.airspeed_tmp = {
			35.372061587561063,
        	50.338178016287728,
        	50.82994907207754,
        	52.110738976984806,
        	40.042613269852986,
        	37.177777988239129},
		.alpha_tmp = {
			0.057031065941469186,
        	0.028135561833523018,
        	0.060005146728576195,
        	0.05847921083269704,
        	0.070439384907833177,
        	0.039180948344768526},
		.beta_tmp = {
			-0.019126744636255771,
        	-0.019126744636255771,
        	-0.019126744636255771,
        	-0.019126744636255771,
        	-0.019126744636255771,
        	-0.019126744636255771},
		.azi_offset_tmp = 0.21607168593378923,
		.elevation_tmp = 0.79857735921480666,
		.radius_tmp = 161.05596346328986,
		.wind_speed_tmp = 7.0
		},
		// fifth entry
		{.airspeed_tmp = {
			40.131365786001062,
        	50.018902591641378,
        	54.465949099557932,
        	52.382488603115043,
        	48.447760213035224,
        	37.796257761554557},
		.alpha_tmp = {
			0.069807885418362067,
        	0.06322052089316392,
        	0.075045467142128217,
        	0.089715694443176383,
        	0.087736874270875967,
        	0.058705672990769356},
		.beta_tmp = {
			-0.072983369919259772,
        	-0.072983369919259772,
        	-0.072983369919259772,
        	-0.072983369919259772,
        	-0.072983369919259772,
        	-0.072983369919259772},
		.azi_offset_tmp = 0.26178482031417261,
		.elevation_tmp = 0.6936460985856221,
		.radius_tmp = 160.19488380753839,
		.wind_speed_tmp = 9.0
		},
		// sixth entry
		{.airspeed_tmp = {
			44.604665791855105,
        	51.096088713816833,
        	55.065306892436901,
        	60.239248325870406,
        	44.788318866481866,
        	42.296117365988081},
		.alpha_tmp = {
			0.033779843826066733,
        	0.054386314636403213,
        	0.060818819385344794,
        	0.042643604646670094,
        	0.081576819008437482,
        	0.077468397439992301},
		.beta_tmp = {
			-0.056680262484383903,
        	-0.056680262484383903,
        	-0.056680262484383903,
        	-0.056680262484383903,
        	-0.056680262484383903,
        	-0.056680262484383903},
		.azi_offset_tmp = 0.34242136266846224,
		.elevation_tmp = 0.73933251710630554,
		.radius_tmp = 170.62215534742327,
		.wind_speed_tmp = 11.0
		},
		// seventh entry
		{.airspeed_tmp = {
			37.598453832064443,
        	48.353642402512918,
        	57.421944052055736,
        	57.69516812458123,
        	50.744339329955004,
        	46.956675202591441},
		.alpha_tmp = {
			0.068045259041148262,
        	0.076532664601668368,
        	0.079875313185744909,
        	0.068185564180445371,
        	0.090161986727896062,
        	0.066829473596123332},
		.beta_tmp = {
			-0.01566420500828411,
        	-0.01566420500828411,
        	-0.01566420500828411,
        	-0.01566420500828411,
        	-0.01566420500828411,
        	-0.01566420500828411},
		.azi_offset_tmp = 0.35756907320370068,
		.elevation_tmp = 0.70812278896380554,
		.radius_tmp = 162.84887283777925,
		.wind_speed_tmp = 13.0
		},
		// eigth entry
		{.airspeed_tmp = {
			41.169122531732356,
        	47.139256016440072,
        	58.189951957594658,
        	60.499259443082813,
        	55.458377407240675,
        	47.433113050554766},
		.alpha_tmp = {
			0.04934420022786222,
        	0.088061825656263845,
        	0.080058673975318115,
        	0.050895228859941813,
        	0.029036341519380324,
        	0.074159248342583264},
		.beta_tmp = {
			0.010047767449306272,
        	0.010047767449306272,
        	0.010047767449306272,
        	0.010047767449306272,
        	0.010047767449306272,
        	0.010047767449306272},
		.azi_offset_tmp = 0.49359773471974755,
		.elevation_tmp = 0.76196515502920892,
		.radius_tmp = 151.81102202997101,
		.wind_speed_tmp = 15.0
		},
		// ninth entry
		{.airspeed_tmp = {
			38.912918388751741,
    		47.063124101215536,
    		54.344874135755852,
    		57.756864098352672,
    		59.833996883927981,
    		49.02479917102761},
		.alpha_tmp = {
			0.04077945417328814,
        	0.073359671077524627,
        	0.072945925620839971,
        	0.064257219949141173,
        	0.042624695304199781,
        	0.044746416650456824},
		.beta_tmp = {
			0.035126426140575401,
        	0.035126426140575401,
        	0.035126426140575401,
        	0.035126426140575401,
        	0.035126426140575401,
        	0.035126426140575401},
		.azi_offset_tmp = 0.62829766882361593,
		.elevation_tmp = 0.76303730402330916,
		.radius_tmp = 168.27405362588058,
		.wind_speed_tmp = 17.0
		},
		// tenth entry
		{.airspeed_tmp = {
			46.809465142369064,
        	49.622980355501767,
        	53.020313907484436,
        	57.094386623933566,
        	58.289766174302443,
        	52.68284056958197},
		.alpha_tmp = {
			-0.015113342636508334,
        	0.016043704179280957,
        	0.07895027546758622,
        	0.056538953869148215,
        	0.064217434599103992,
        	0.070038760996930446},
		.beta_tmp = {
			0.0043759688457945811,
        	0.0043759688457945811,
        	0.0043759688457945811,
        	0.0043759688457945811,
        	0.0043759688457945811,
        	0.0043759688457945811},
		.azi_offset_tmp = 0.84064833595182797,
		.elevation_tmp = 0.78216049290536205,
		.radius_tmp = 159.01164274084874,
		.wind_speed_tmp = 19.0
		}
	};

	// copy Temp_playbook over to crosswind->playbook
	for (int i = 0; i < sizeof(play_load)/sizeof(play_load[0]); i++){
		memcpy(crosswind->playbook.entries[i].airspeed_lookup, play_load[i].airspeed_tmp, sizeof(crosswind->playbook.entries[i].airspeed_lookup));
		memcpy(crosswind->playbook.entries[i].alpha_lookup, play_load[i].alpha_tmp, sizeof(crosswind->playbook.entries[i].alpha_lookup));
		crosswind->playbook.entries[i].azi_offset = play_load[i].azi_offset_tmp;
		memcpy(crosswind->playbook.entries[i].beta_lookup, play_load[i].beta_tmp, sizeof(crosswind->playbook.entries[i].beta_lookup));
		crosswind->playbook.entries[i].elevation = play_load[i].elevation_tmp;
		crosswind->playbook.entries[i].path_radius_target = play_load[i].radius_tmp;
		crosswind->playbook.entries[i].wind_speed = play_load[i].wind_speed_tmp;
	}
	crosswind->playbook.num_entries = sizeof(play_load)/sizeof(play_load[0]);
}

void loadFallback(CrosswindParams *crosswind){
	//crosswind->playbook_fallback
	// place holder for fallback playbook
}

// Loads Crosswind Parameter
void loadcrosswind(CrosswindParams *crosswind) {
	//Inner params
	crosswind->inner.elevator_flap_ratio = -0.039;

	// This ratio[rad / rad] is used to create a feed - forward term for
	// the elevator deflection.It was necessary to limit high
	// tensions by smoothing the transition to crosswind in the high
	// wind IEC cases.
	crosswind->inner.delevator_dalpha = -0.5;

	// Gain[rad / #] between a change in C_L command and a delta.
	crosswind->inner.kp_flap = 0.5;

	// Table of airspeeds[m / s] that are used to schedule and limit
	// gains for the inner loops. kCrosswindInnerNumAirspeeds
	double airspeeds_tmp[kCrosswindInnerNumAirspeeds] = { 30.0, 60.0, 90.0 };

	// assign to sturctures
	memcpy(crosswind->inner.airspeed_table, airspeeds_tmp, sizeof(crosswind->inner.airspeed_table));

	// Gain matrices to control the longitudinal plant calculated at
	// the minimum, nominal, and maximum airspeeds.The state and
	// input vectors are :
	//
	//   x = [z, dz / dt, alpha, q, int_alpha]'
	//   u = [elevator, motor_pitch]'
	//
	double longitudinal_gains_min_airspeed_tmp[kNumCrosswindLongitudinalInputs][kNumCrosswindLongitudinalStates] = { { -0.008, 0.032, -0.782, -0.381, 0.311 },{ 15.392, -62.956, 1495.102, 735.692, -591.274 } };
	double longitudinal_gains_nominal_airspeed_tmp[kNumCrosswindLongitudinalInputs][kNumCrosswindLongitudinalStates] = { { 0.009, 0.014, -0.601, -0.194, 0.316 },{ -4.069, -6.777, 281.996, 93.918, -148.75 } };
	double longitudinal_gains_max_airspeed_tmp[kNumCrosswindLongitudinalInputs][kNumCrosswindLongitudinalStates] = { { 0.007, 0.008, -0.492, -0.15, 0.316 },{ -1.433, -1.689, 100.809, 32.223, -65.64 } };
	// assign to sturctures
	memcpy(crosswind->inner.longitudinal_gains_min_airspeed, longitudinal_gains_min_airspeed_tmp, sizeof(crosswind->inner.longitudinal_gains_min_airspeed));
	memcpy(crosswind->inner.longitudinal_gains_nominal_airspeed, longitudinal_gains_nominal_airspeed_tmp, sizeof(crosswind->inner.longitudinal_gains_nominal_airspeed));
	memcpy(crosswind->inner.longitudinal_gains_max_airspeed, longitudinal_gains_max_airspeed_tmp, sizeof(crosswind->inner.longitudinal_gains_max_airspeed));

	// Minimum and maximum values[rad - s] for the integrated alpha
	// error.  These roughly correspond to an integrated elevator
	// deflection between[-0.1, 0.15] rad assuming an integral gain
	// around 0.3.
	crosswind->inner.int_alpha_min = -0.333;
	crosswind->inner.int_alpha_max = 0.5;

	// Gain matrices to control the lateral plant calculated at the
	// minimum, nominal, and maximum airspeeds.The state and input
	// vectors are :
	//
	//   x = [tether_roll, beta, p, r, int_tether_roll, int_beta]'
	//   u = [aileron, rudder, motor_yaw]'
	//

	//lateral_gains_min_airspeed
	double lateral_gains_min_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{ 1.458, -1.592, -0.418, 0.293, -0.747, 0.331},
		{ 0.25, 1.044, 0.06, -0.562, -0.428, -0.28}, 
		{ -19964.958, -42475.91, -1919.714, 29393.075, 25428.189, 12486.527} };
	memcpy(crosswind->inner.lateral_gains_min_airspeed, lateral_gains_min_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_min_airspeed));

	//lateral_gains_nominal_airspeed
	double lateral_gains_nominal_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{1.057, -1.497, -0.227, 0.193, -0.764, 0.322},
		{ 0.419, 1.126, 0.033, -0.394, -0.617, -0.372},
		{ -7266.908, -11636.044, -298.718, 5330.468, 9263.574, 4409.115} };
	memcpy(crosswind->inner.lateral_gains_nominal_airspeed, lateral_gains_nominal_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_nominal_airspeed));

	//lateral_gains_max_airspeed
	double lateral_gains_max_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{ 0.983, -1.416, -0.194, 0.148, -0.777, 0.315},
		{ 0.312, 1.109, 0.025, -0.292, -0.624, -0.386},
		{ -2591.322, -4972.762, -106.725, 1785.541, 4170.422, 2023.546} };
	memcpy(crosswind->inner.lateral_gains_max_airspeed, lateral_gains_max_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_max_airspeed));

	//lateral_gains_alt_min_airspeed
	double lateral_gains_alt_min_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{ 1.529, -1.676, -0.433, 0.3, -0.738, 0.338},
		{ 0.62, 1.17, 0.051, -0.824, -0.672, -0.368},
		{ -235.071, -276.764, -7.323, 250.525, 222.042, 98.666} };
	memcpy(crosswind->inner.lateral_gains_alt_min_airspeed, lateral_gains_alt_min_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_alt_min_airspeed));

	//lateral_gains_alt_nominal_airspeed
	double lateral_gains_alt_nominal_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{ 1.064, -1.505, -0.228, 0.194, -0.765, 0.322},//, -0.112},
		{ 0.452, 1.134, 0.033, -0.406, -0.644, -0.382},//, -1.126},
		{ -43.702, -66.078, -1.645, 30.995, 54.317, 25.637} };//, 354.637} };
	memcpy(crosswind->inner.lateral_gains_alt_nominal_airspeed, lateral_gains_alt_nominal_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_alt_nominal_airspeed));

	//lateral_gains_alt_max_airspeed
	double lateral_gains_alt_max_airspeed_tmp[kNumCrosswindLateralInputs][kNumCrosswindLateralStates] = {
		{ 0.984, -1.418, -0.194, 0.148, -0.777, 0.315},//, -0.091},
		{ 0.318, 1.111, 0.025, -0.294, -0.629, -0.389},//, -0.821},
		{ -14.803, -28.014, -0.599, 10.104, 23.664, 11.458} };//, 103.57} };
	memcpy(crosswind->inner.lateral_gains_alt_max_airspeed, lateral_gains_alt_max_airspeed_tmp, sizeof(crosswind->inner.lateral_gains_alt_max_airspeed));

	// Minimum and maximum values[rad - s] for the integrated beta
	// error and tether roll error.
	min_max_lateral_integrators(crosswind);

	// Airspeed error[m / s] is saturated to plus / minus this value.
	crosswind->inner.max_airspeed_error = 5.0;

	// Gains for the airspeed loop.The negative integrator limit
	// [N] must be relatively small to avoid windup when the
	// propellers are at their maximum advance ratio.
	crosswind->inner.airspeed_pid.kp = 3.42e+03;
	crosswind->inner.airspeed_pid.ki = 1.60e+03;
	crosswind->inner.airspeed_pid.kd = 0.00;
	crosswind->inner.airspeed_pid.int_output_min = -5000.0;
	crosswind->inner.airspeed_pid.int_output_max = 5000.0;
	
	// Maximum magnitude of the aerodynamic power[W] that the airspeed
	// controller is allowed to command in generation and motoring.This is
	// used to roughly bound the thrusts.Detailed constraints on the motor
	// power are used in the motor solver.
	crosswind->inner.max_airspeed_control_power_gen = 850e3;
	crosswind->inner.max_airspeed_control_power_motor = 650e3;

	// Maximum approximate slew rate of the thrust command [N/s].
	crosswind->inner.max_airspeed_control_thrust_rate = 1.50e4;
	
	// Thrust command at crosswind initialization.
	//	'initial_thrust': params['trans_in']['longitudinal']['thrust_cmd'],
	crosswind->inner.initial_thrust = 28000.0;

	// Airspeed errors[m / s] at which the spoiler turns on / off and
	// how far / fast it deflects[rad] / [rad / s].
	double delta_spoiler = -25.0 * 3.14159265359 / 180.0;
	crosswind->inner.airspeed_error_spoiler_on	= 8.0;
	crosswind->inner.airspeed_error_spoiler_off = 4.0;
	crosswind->inner.delta_spoiler				= delta_spoiler;
	crosswind->inner.delta_spoiler_on_rate		= 0.4 * delta_spoiler;
	crosswind->inner.delta_spoiler_off_rate		= -1.0 * delta_spoiler;

	// Rate[1 / s] to fade alternate inner loop gains in or out.
	crosswind->inner.alt_gain_fade_rate = 1.0;

	// load playbook
	loadPlaybook(crosswind);
	// load fallback playbook
	loadFallback(crosswind);
}

void loadcontroller(ControlParams *control_params){

//load various relevant structures
	loadcrosswind(&control_params->crosswind);

}
