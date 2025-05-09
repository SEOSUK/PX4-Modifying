#include <px4_platform_common/module.h>
#include "listen_and_speak.hpp"


float vibration = 0.0;
float sine_wave = 0.0;
float time_count = 0.0;
float end = 0.0;
float start = 0.0;
float delta_t = 0.0;

ListenAndSpeak::ListenAndSpeak() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
   _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")) // 1ms 주기 실행
{
	_wrench_command_pub.advertise();
	_thrust_command_pub.advertise();
}

ListenAndSpeak::~ListenAndSpeak()
{
    
    ScheduleClear();
    perf_free(_loop_perf);
}

bool ListenAndSpeak::init()
{
    //ScheduleOnInterval(1000_us); // 1ms (1000마이크로초) 주기로 실행
    
    #ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
    #endif

    PX4_INFO("listenand speak init good");
    return true;
}

int ListenAndSpeak::task_spawn(int argc, char *argv[])
{
	ListenAndSpeak *instance = new ListenAndSpeak();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void ListenAndSpeak::Run()
{
    PX4_INFO("GOOOOOOOOOOOOOOOOOOD");
    
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
	
    #ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
    #endif

    end=hrt_absolute_time();
    delta_t=end-start;
    start=hrt_absolute_time();


    float freq = 0.3; // rad/s
    vibration = 1*sin(freq*time_count);
    sine_wave = vibration;
    time_count += delta_t;


    servo_angle_s theta_msg{};
    if (_servo_angle_sub.update(&theta_msg)) {
	        
        PX4_INFO("Received servo_angle: servo1=%.3f, servo2=%.3f, servo3=%.3f, servo4=%.3f",
                 (double)theta_msg.servo_angle[0], 
                 (double)theta_msg.servo_angle[1], 
                 (double)theta_msg.servo_angle[2],
		         (double)theta_msg.servo_angle[3]);
    }

  
	if(_manual_control_setpoint_sub.update(&_manual_control_setpoint)){

	_manual_control_setpoint_for_tau_yaw = _manual_control_setpoint.yaw;
	_manual_control_setpoint_for_force_z = _manual_control_setpoint.throttle;

	
	// send neutral setpoints if no update for 1 s
	if (hrt_elapsed_time(&_manual_control_setpoint.timestamp) > 1_s) {
		_manual_control_setpoint_for_tau_yaw = 0.f;
		_manual_control_setpoint_for_force_z = 0.f;
	}
}   
    


        

	PX4_INFO("GOOOOOOOOOOOOOOOOOOD_END");
   	
    perf_end(_loop_perf);	
}

int ListenAndSpeak::custom_command(int argc, char *argv[])
{
	return 0;
}

int ListenAndSpeak::print_usage()
{
	PX4_INFO("ListenAndSpeak module is running.");
	return 0;
}

int ListenAndSpeak::print_status()
{
    PX4_INFO("ListenAndSpeak module is running.");
    PX4_INFO("%f",(double)_manual_control_setpoint_for_tau_yaw);
    PX4_INFO("%f",(double)_manual_control_setpoint_for_force_z);
    return 0;
}

// PX4 모듈 실행 함수
extern "C" __EXPORT int listen_and_speak_main(int argc, char *argv[]);

int listen_and_speak_main(int argc, char *argv[])
{

/*
    ListenAndSpeak instance;
    
    if (!instance.init()) {
        PX4_ERR("Initialization failed");
        return -1;
    }

    instance.Run();
    return 0;
*/
return ListenAndSpeak::main(argc, argv);

}
