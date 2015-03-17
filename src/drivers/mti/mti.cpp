/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <drivers/drv_mti.h>
#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_mti_position.h>
#include <uORB/topics/gnssins_report.h>
#include <uORB/topics/satellite_info.h>
#include <termios.h>

#include <board_config.h>

//#include "ubx.h"
//#include "mtk.h"
//#include "ashtech.h"
#include "mti.h"

#define TIMEOUT_333HZ 3
#define TIMEOUT_5KHZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* class for dynamic allocation of satellite info data */
class MTI_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};


class mti : public device::CDev
{
public:
	mti(const char *uart_path, bool fake_mti, bool enable_sat_info);
	virtual ~mti();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

	void 				print_error_count();
	void print_vel();
	void print_accel();
	void print_euler();
	void print_gry();
	void print_lon_lat_alt();
	void print_status();

private:
///< flag to make the main worker task exit
	bool				_task_should_exit;
	///< serial interface to mti
	int				_serial_fd;
	int				_serial_fd1;
	///< current baudrate
	unsigned			_baudrate;
	///< device / serial port path
	char				_port[20];
	///< worker task
	volatile int			_task;
	///< flag to signal if the mti is ok/
	bool				_healthy;
	//< flag to signal that the baudrate with the mti has changed
	bool				_baudrate_changed;
	///< flag that the GPS mode has changed //���Բ���
	bool				_mode_changed;
	bool 				serial_configured;

	///< current mode ��ͷ�ļ�drv_gps�С�
	mti_driver_mode_t		_mode;


   ///< instance of GPS parser // �����������MTI_Helper.h �������࣬
   //��MTI_helper.cpp�ж�����غ���
	mti_Helper			*_Helper;
	///< instance of mti's GPS sat info data object
	MTI_Sat_Info			*_Sat_Info;
	///< uORB topic for gps position //ָҪ���͵����ݰ��ṹ��
//	struct vehicle_mti_position_s	_report_mti_pos;
	struct GNSSINSReportS	_report_mti_pos;
	///< uORB pub for gps position //ORB topic advertiser handle.
	orb_advert_t			_report_mti_pos_pub;
	///< pointer to uORB topic for satellite info //ָҪ���͵����ݰ��ṹ��
	struct satellite_info_s		*_p_report_sat_info;
	///< uORB pub for satellite info	//ORB topic advertiser handle.
	orb_advert_t			_report_sat_info_pub;
	///< position update rate
	float				_rate;
	///< fake gps output
	bool				_fake_mti;
//	uint8_t buf[128]; // buffer

	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void			 	config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};
//class mti end

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int mti_main(int argc, char *argv[]);

namespace
{

mti	*g_dev;

}


mti::mti(const char *uart_path, bool fake_mti, bool enable_sat_info) :
	CDev("mti", MTI_DEVICE_PATH),
	_task_should_exit(false),
	_baudrate(921600),
	_healthy(false),
	_mode_changed(false),
	serial_configured(false),
	_mode(MTI_DRIVER_MODE_MTI), //��ʼ��MTIģ�顣�ֲ�֪�к���
	_Helper(nullptr),
	_Sat_Info(nullptr),
	_report_mti_pos_pub(-1),
	_p_report_sat_info(nullptr),
	_report_sat_info_pub(-1),
	_rate(0.0f),
	_fake_mti(fake_mti)

{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	memset(&_report_mti_pos, 0, sizeof(_report_mti_pos));

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_Sat_Info = new(MTI_Sat_Info);
		_p_report_sat_info = &_Sat_Info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info)); //�����ڴ浥Ԫ
	}

	_debug_enabled = true;
}

mti::~mti()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;

}

int
mti::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;
	/* start the GPS driver worker task */// �������ȼ��Ƿ���Ҫ�޸ģ�
	_task = task_spawn_cmd("mti", SCHED_DEFAULT,
				SCHED_PRIORITY_SLOW_DRIVER, 1500, (main_t)&mti::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}else{
		printf("task start success!!!\n");
	}

	ret = OK;
out:
	return ret;
}

int
mti::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();
	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;
	default:
		/* give it to parent if no one wants it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}
	unlock();
	return ret;
}

void
mti::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}
void
mti::task_main()
{
	log("MTI starting");
	static uint16_t error_count = 0;
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR|O_NONBLOCK);
	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}else
	{
		printf("open serial port success, serial_fd: %d !!!\n",_serial_fd);
//		::write(_serial_fd, "cmz11\n\r", sizeof("cmz11\n\r"));
	}
//	uint64_t last_rate_measurement = hrt_absolute_time();
//	unsigned last_rate_count = 0;
	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_Helper != nullptr) {
		delete(_Helper);
		/* set to zero to ensure parser is not used while not instantiated */
		_Helper = nullptr;
		}
		if(_mode == MTI_DRIVER_MODE_MTI)
		{
			////��ʼ��MTI����ͨ������_report_mti_pos���ã��������ݣ���������Ϊvehicle_mti_position_s
			// MTI�ڲ�����Ӧ�õ�mti_positionʵ�ʾ���_report_mti_pos
			_Helper = new MTI(_serial_fd, &_report_mti_pos,_p_report_sat_info);
		}
		unlock();
		if (_Helper->configure(_baudrate) == 0) {
				unlock();
				// GPS is obviously detected successfully, reset statistics
				_Helper->reset_update_rates();
				uint8_t helper_ret;
				while (!_task_should_exit) {
					helper_ret = _Helper->receive(TIMEOUT_333HZ);
						/* opportunistic publishing - else invalid data would end up on the bus */
					if (!(_pub_blocked)) {
						switch(helper_ret)
						{
							case RX_RIGHT:
								if (_report_mti_pos_pub > 0) {
									orb_publish(ORB_ID(gnssins_report), _report_mti_pos_pub, &_report_mti_pos);
								} else {
									_report_mti_pos_pub = orb_advertise(ORB_ID(gnssins_report), &_report_mti_pos);
								}
								break;
							default:
								_report_mti_pos.GNSSINSReportStatus = IsInvaild;
								error_count++;
								_report_mti_pos.mti_error_count1  = error_count;
								if (_report_mti_pos_pub > 0) {
									orb_publish(ORB_ID(gnssins_report), _report_mti_pos_pub, &_report_mti_pos);
								} else {
									_report_mti_pos_pub = orb_advertise(ORB_ID(gnssins_report), &_report_mti_pos);
								}
								break;
//							case RX_POLL_ERR:
//								if (helper_ret == MTDATA_ALL_COMP) {
//									if (_report_mti_pos_pub > 0) {
//										orb_publish(ORB_ID(gnssins_report), _report_mti_pos_pub, &_report_mti_pos);
//									} else {
//										_report_mti_pos_pub = orb_advertise(ORB_ID(gnssins_report), &_report_mti_pos);
//									}
//								}
//								break;
						}// end switch(helper_ret)
					} //   end  if (!(_pub_blocked))
//					if (helper_ret & 1) {	// consider only pos info updates for rate calculation */
//						last_rate_count++;
//					}
//					/* measure update rate every 5 seconds */
//					if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
//						_rate = last_rate_count / ((float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f);
//						last_rate_measurement = hrt_absolute_time();
//						last_rate_count = 0;
//						_Helper->store_update_rates();
//						_Helper->reset_update_rates();
//					}
//					if (!_healthy) {
//						const char *mode_str = "unknown";
//						if(_mode == MTI_DRIVER_MODE_MTI)
//						{
//							mode_str = "THIS MODE IS MTI!!";
//						}
//						warnx("module found: %s \n", mode_str);
//	//					printf("the healthy is ture!!!!\n");
//						_healthy = true;
//					}
//					printf("health:%d!\n",_healthy);
				}// end while ()
//				printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
//				if (_healthy) {
//					warnx("module lost");
//					_healthy = false;
//					_rate = 0.0f;
//				}
				lock();
			}//end if (_Helper->configure(_baudrate) == 0)
			lock();
		}// end while (!_task_should_exit)

	warnx("exiting");
	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}
void
mti::cmd_reset()
{
#ifdef GPIO_GPS_NRESET
	warnx("Toggling GPS reset pin");
	stm32_configgpio(GPIO_GPS_NRESET);
	stm32_gpiowrite(GPIO_GPS_NRESET, 0);
	usleep(100);
	stm32_gpiowrite(GPIO_GPS_NRESET, 1);
	warnx("Toggled GPS reset pin");
#endif
}

void
mti::print_info()
{
	switch (_mode) {
	case MTI_DRIVER_MODE_UBX:
		warnx("protocol: UBX \n");
		break;
	case MTI_DRIVER_MODE_MTK:
		warnx("protocol: MTK \n");
		break;
	case MTI_DRIVER_MODE_MTI:
		warnx("protocol: MTI \n");
		break;
	case MTI_DRIVER_MODE_ASHTECH:
		warnx("protocol: ASHTECH \n");
		break;
	default:
		break;
	}

	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	warnx("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");

//	if (_report_mti_pos.timestamp_position != 0) {
//		warnx("position lock: %dD, satellites: %d, last update: %8.4fms ago", (int)_report_mti_pos.fix_type,
//				_report_mti_pos.satellites_used, (double)(hrt_absolute_time() - _report_mti_pos.timestamp_position) / 1000.0);
//		warnx("lat: %d, lon: %d, alt: %d", _report_mti_pos.lat, _report_mti_pos.lon, _report_mti_pos.alt);
//		warnx("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)_report_mti_pos.vel_n_m_s,
//			(double)_report_mti_pos.vel_e_m_s, (double)_report_mti_pos.vel_d_m_s);
//		warnx("eph: %.2fm, epv: %.2fm", (double)_report_mti_pos.eph, (double)_report_mti_pos.epv);
//		warnx("rate position: \t%6.2f Hz", (double)_Helper->get_position_update_rate());
//		warnx("rate velocity: \t%6.2f Hz", (double)_Helper->get_velocity_update_rate());
//		warnx("rate publication:\t%6.2f Hz", (double)_rate);
//
//	}

//	//angular velocity
//	printf("the AngularVelocity is : x-->%f\n",(double)_report_mti_pos.AngularVelocityX);
//	printf("the AngularVelocity is : y-->%f\n",(double)_report_mti_pos.AngularVelocityY);
//	printf("the AngularVelocity is : z-->%f\n",_report_mti_pos.AngularVelocityZ);
//	//faccel
//	printf("the Acceleration is : x-->%f\n",_report_mti_pos.LinearAccelerationX);
//	printf("the Acceleration is : y-->%f\n",_report_mti_pos.LinearAccelerationX);
//	printf("the Acceleration is : z-->%f\n",_report_mti_pos.LinearAccelerationX);
// sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);

	printf("the pitch is :%6.3f\n",(double)raw.pitch);
	printf("the roll is :%6.3f\n",(double)raw.roll);
	printf("the yaw is :%6.3f\n",(double)raw.yaw);
	printf("the stamp time is : %d\n",(double)raw.TimeStamp);
	printf("the error count is : %u\n",raw.mti_error_count1);
	printf("the accelx is :%6.3f\n",(double)raw.LinearAccelerationX);
	printf("the accely is :%6.3f\n",(double)raw.LinearAccelerationY);
	printf("the accelz is :%6.3f\n",(double)raw.LinearAccelerationZ);
	printf("the gryx is :%6.3f\n",(double)raw.AngularVelocityX);
	printf("the gryy is :%6.3f\n",(double)raw.AngularVelocityY);
	printf("the gryz is :%6.3f\n",(double)raw.AngularVelocityZ);
	printf("the VELX is :%6.3f\n",(double)raw.LinearVelocityN);
	printf("the VELY is :%6.3f\n",(double)raw.LinearVelocityE);
	printf("the VELZ is :%6.3f\n",(double)raw.LinearVelocityD);
	printf("the lat is :%6.3f\n",(double)raw.Latitude);
	printf("the lon is :%6.3f\n",(double)raw.Longitude);
	printf("the alt is :%6.3f\n",(double)raw.Altitude);
	printf("the status is : %0x\n",raw.Status);
	printf("the report status is : %d\n",raw.GNSSINSReportStatus);
//	usleep(100000);
}
void
mti::print_vel(){
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the VELX is :%6.3f\n",(double)raw.LinearVelocityN);
	printf("the VELY is :%6.3f\n",(double)raw.LinearVelocityE);
	printf("the VELZ is :%6.3f\n",(double)raw.LinearVelocityD);
}
void
mti::print_accel(){
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the accelx is :%6.3f\n",(double)raw.LinearAccelerationX);
	printf("the accely is :%6.3f\n",(double)raw.LinearAccelerationY);
	printf("the accelz is :%6.3f\n",(double)raw.LinearAccelerationZ);
}
void
mti::print_euler(){
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the pitch is :%6.3f\n",(double)raw.pitch);
	printf("the roll is :%6.3f\n",(double)raw.roll);
	printf("the yaw is :%6.3f\n",(double)raw.yaw);
}
void
mti::print_gry()
{
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the gryx is :%6.3f\n",(double)raw.AngularVelocityX);
	printf("the gryy is :%6.3f\n",(double)raw.AngularVelocityY);
	printf("the gryz is :%6.3f\n",(double)raw.AngularVelocityZ);
}
void
mti::print_lon_lat_alt()
{
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the lat is :%6.3f\n",(double)raw.Latitude);
	printf("the lon is :%6.3f\n",(double)raw.Longitude);
	printf("the alt is :%6.3f\n",(double)raw.Altitude);
}
void
mti::print_error_count()
{
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the error count is : %u\n",raw.mti_error_count1);
}
void
mti::print_status()
{
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
	orb_set_interval(gnss_report_fd, 1000);
	orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
	printf("the status is : %u\n",raw.Status);
	printf("the report status is : %d\n",raw.GNSSINSReportStatus);
}
/**
 * Local functions in support of the shell command.
 */
namespace mti_namespace
{

mti	*g_dev;

void	start(const char *uart_path, bool fake_mti, bool enable_sat_info);
void	stop();
void	test();
void	reset();
void	info();
void info_error_count();
void info_vel();
void info_accel();
void info_euler();
void info_gry();
void info_lon_lat_alt();
void info_status();
/**
 * Start the driver.
 */
void
start(const char *uart_path, bool fake_mti, bool enable_sat_info)
// uart_path ��Ĭ�ϵ�ַΪ��GPS_DEFAULT_UART_PORT "/dev/ttyS5"
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "mti already started!!\n");

	/* create the driver */
	g_dev = new mti(uart_path, fake_mti, enable_sat_info);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MTI_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", MTI_DEVICE_PATH);
		goto fail;
	}else
	{
		printf("open mti device path successes!!\n");
	}

	exit(0);
fail:
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}
	errx(1, "driver start failed!!");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;
	if(g_dev == nullptr)
	{
		printf("MTI have stopped!!!\n");
	}else
	{
		printf("MTI stopped failed!!!\n");
	}
	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */


/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(MTI_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "MTI open failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void test()
{
//	g_dev->test();
}
void info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_info();

	exit(0);
}
void info_error_count()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_error_count();
	exit(0);
}
void info_vel()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_vel();
	exit(0);
}
void info_gry()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_gry();
	exit(0);
}
void info_accel()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_accel();
	exit(0);
}
void info_lon_lat_alt()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_lon_lat_alt();
	exit(0);
}
void info_euler()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_euler();
	exit(0);
}\
void info_status()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");
	g_dev->print_status();
	exit(0);
}
} // namespace


int
mti_main(int argc, char *argv[])
{

	/* set to default */
	const char *device_name = MTI_DEFAULT_UART_PORT;
	bool fake_mti = false;
	bool enable_sat_info = false;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				goto out;
			}
		}

		/* Detect fake gps option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-f"))
				fake_mti = true;
		}

		/* Detect sat info option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-s"))
				enable_sat_info = true;
		}

		printf("the fake_mti = %b;\n device_name =%s;\n enable_st = %d \n",fake_mti,device_name,enable_sat_info);

		mti_namespace::start(device_name, fake_mti, enable_sat_info);
	}

	if (!strcmp(argv[1], "stop"))
		mti_namespace::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		mti_namespace::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		mti_namespace::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		mti_namespace::info();
	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		mti_namespace::info_status();
	/*
	 * Print driver velocity.
	 */
	if (!strcmp(argv[1], "vel"))
		mti_namespace::info_vel();
	/*
	 * Print driver acceleration.
	 */
	if (!strcmp(argv[1], "accel"))
		mti_namespace::info_accel();
	/*
	 * Print driver euler angles.
	 */
	if (!strcmp(argv[1], "eula"))
		mti_namespace::info_euler();
	/*
	 * Print driver gry.
	 */
	if (!strcmp(argv[1], "gry"))
		mti_namespace::info_gry();
	/*
	 * Print driver lon_lat_alt.
	 */
	if (!strcmp(argv[1], "lon_lat_alt"))
		mti_namespace::info_lon_lat_alt();
	/*
	 * Print error_count.
	 */
	if (!strcmp(argv[1], "errorcount"))
		mti_namespace::info_error_count();
out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' ,'status','info','vel','accel'"
			" 'eula','gry','lon_lat_alt','errorcount' [-d /dev/ttyS0-n][-f][-s]");
}
