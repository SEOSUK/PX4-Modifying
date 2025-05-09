

using namespace time_literals;

__EXPORT int listen_and_speak_main(int argc, char *argv[]);

void Run();

int listen_and_speak(int argc, char *argv[])
{
	PX4_INFO("listen and speak moduel on !!");
	
	perf_counter_t	_loop_perf;

	uORB::SubscriptionInterval _servo_angle_sub{ORB_ID(servo_angle), 1000}; // micro second : 1000 -> 1 milli second

	uORB::Publication<wrench_command_s>	_wrench_command_pub{ORB_ID(wrench_command)};
	uORB::Publication<thrust_command_s> 	_thrust_command_pub{ORB_ID(thrust_command)};


	//int servo_angle_sub = orb_subscribe(ORB_ID(servo_angle));
	//orb_set_interval(servo_angle_sub,200); // millisecond

	struct servo_angle_s servo_ang;
	struct wrench_command_s wrench_cmd;
	struct thrust_command_s thrust_cmd;

	memset(&servo_ang, 0, sizeof(servo_ang));
	memset(&wrench_cmd, 0, sizeof(wrench_cmd));
	memset(&thrust_cmd, 0, sizeof(thrust_cmd));

	orb_advert_t wrench_command_pub = orb_advertise(ORB_ID(wrench_command), &wrench_cmd);
	orb_advert_t thrust_command_pub = orb_advertise(ORB_ID(thrust_command), &wrench_cmd);


	


}

void Run(){

}
