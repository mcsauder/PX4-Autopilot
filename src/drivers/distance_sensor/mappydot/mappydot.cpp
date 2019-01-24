/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mappydot.cpp
 * @author Mohammed Kabir (mhkabir@mit.edu)
 *
 * Driver for the Mappydot infrared rangefinders connected via I2C.
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <perf/perf_counter.h>

#include <parameters/param.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define MAPPYDOT_BUS_DEFAULT        PX4_I2C_BUS_EXPANSION2
#define MAPPYDOT_BASE_ADDR           0x08
#define MAPPYDOT_DEVICE_PATH         "/dev/mappydot"

/* MappyDot Registers */
/* Basics */
#define MAPPYDOT_CHECK_INTERRUPT                             (0x49)
#define MAPPYDOT_MEASUREMENT_BUDGET                          (0x42)
#define MAPPYDOT_PERFORM_SINGLE_RANGE                        (0x53)
#define MAPPYDOT_RANGING_MEASUREMENT_MODE                    (0x6d)
#define MAPPYDOT_READ_ACCURACY                               (0x52)
#define MAPPYDOT_READ_DISTANCE                               (0x72)
#define MAPPYDOT_READ_ERROR_CODE                             (0x45)
#define MAPPYDOT_SET_CONTINUOUS_RANGING_MODE                 (0x63)
#define MAPPYDOT_SET_SINGLE_RANGING_MODE                     (0x73)

/* Configuration */
#define MAPPYDOT_AVERAGING_ENABLE                            (0x56)
#define MAPPYDOT_AVERAGING_DISABLE                           (0x76)
#define MAPPYDOT_AVERAGING_SAMPLES                           (0x69)
#define MAPPYDOT_CALIBRATE_DISTANCE_OFFSET                   (0x61)
#define MAPPYDOT_CALIBRATE_CROSSTALK                         (0x78)
#define MAPPYDOT_CALIBRATE_SPAD                              (0x75)
#define MAPPYDOT_DISABLE_CROSSTALK_COMPENSATION              (0x6b)
#define MAPPYDOT_ENABLE_CROSSTALK_COMPENSATION               (0x4b)
#define MAPPYDOT_FILTERING_ENABLE                            (0x46)
#define MAPPYDOT_FILTERING_DISABLE                           (0x66)
#define MAPPYDOT_INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY     (0x51)
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_ENABLE      (0x54)
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_DISABLE     (0x74)
#define MAPPYDOT_INTERSENSOR_CROSSTALK_TIMEOUT               (0x71)
#define MAPPYDOT_INTERSENSOR_SYNC_ENABLE                     (0x59)
#define MAPPYDOT_INTERSENSOR_SYNC_DISABLE                    (0x79)
#define MAPPYDOT_REGION_OF_INTEREST                          (0x70)
#define MAPPYDOT_SET_GPIO_MODE                               (0x67)
#define MAPPYDOT_SET_GPIO_THRESHOLD_DISTANCE_IN_MM           (0x6f)
#define MAPPYDOT_SET_LED_MODE                                (0x6c)
#define MAPPYDOT_SET_LED_THRESHOLD_DISTANCE_IN_MM            (0x65)
#define MAPPYDOT_SIGMA_LIMIT_CHECK_VALUE                     (0x4C)
#define MAPPYDOT_SIGNAL_LIMIT_CHECK_VALUE                    (0x47)

/* Settings */
#define MAPPYDOT_DEVICE_NAME                                 (0x64)
#define MAPPYDOT_FIRMWARE_VERSION                            (0x4e)
#define MAPPYDOT_NAME_DEVICE                                 (0x6e)
#define MAPPYDOT_READ_CURRENT_SETTINGS                       (0x62)
#define MAPPYDOT_RESTORE_FACTORY_DEFAULTS                    (0x7a)
#define MAPPYDOT_WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT  (0x77)

/* Advanced */
#define MAPPYDOT_AMBIENT_RATE_RETURN                         (0x41)
#define MAPPYDOT_READ_NONFILTERED_VALUE                      (0x6a)
#define MAPPYDOT_RESET_VL53L1X_RANGING                       (0x58)
#define MAPPYDOT_SIGNAL_RATE_RETURN                          (0x4A)
#define MAPPYDOT_VL53L1X_NOT_SHUTDOWN                        (0x48)
#define MAPPYDOT_VL53L1X_SHUTDOWN                            (0x68)

/* Super Advanced */
#define MAPPYDOT_ENTER_FACTORY_MODE                          (0x23) //"#"//"!#!#!#"
#define MAPPYDOT_WIPE_ALL_SETTINGS                           (0x3c) //"<"//"><><><" (Must be in factory mode)

/* Ranging Modes */
#define MAPPYDOT_LONG_RANGE                                  (0x6c)
#define MAPPYDOT_MED_RANGE                                   (0x6d)
#define MAPPYDOT_SHORT_RANGE                                 (0x73)

/* LED Modes */
#define MAPPYDOT_LED_OFF                                     (0x66)
#define MAPPYDOT_LED_ON                                      (0x6f)
#define MAPPYDOT_LED_MEASUREMENT_OUTPUT                      (0x6d)
#define MAPPYDOT_LED_PWM_ENABLED                             (0x70)
#define MAPPYDOT_LED_THRESHOLD_ENABLED                       (0x74)

/* GPIO Modes */
#define MAPPYDOT_GPIO_HIGH                                   (0x6f)
#define MAPPYDOT_GPIO_LOW                                    (0x66)
#define MAPPYDOT_GPIO_MEASUREMENT_INTERRUPT                  (0x6d)
#define MAPPYDOT_GPIO_PWM_ENABLED                            (0x70)
#define MAPPYDOT_GPIO_THRESHOLD_ENABLED                      (0x74)

/* I2C Bootloader */
#define MAPPYDOT_REBOOT_TO_BOOTLOADER                        (0x01)

/* Device limits */
#define MAPPYDOT_MIN_DISTANCE                                (0.20f)
#define MAPPYDOT_MAX_DISTANCE                                (4.00f)

#define MAPPYDOT_CONVERSION_INTERVAL                         (100000) /* 10ms */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MappyDot : public device::I2C
{
public:
	MappyDot(int bus = MAPPYDOT_BUS_DEFAULT, int address = MAPPYDOT_BASE_ADDR);
	virtual ~MappyDot();

	/**
	 * @brief
	 */
	virtual int init();

	/**
	 * @brief
	 */
	virtual int ioctl(device::file_t *file_pointer, int cmd, unsigned long arg);

	/**
	 * @brief
	 */
	virtual ssize_t read(device::file_t *file_pointer, char *buffer, size_t buflen);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

protected:

private:

	/**
	 * @brief Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();

	/**
	 * @brief Stop the automatic measurement state machine.
	 */
	void stop();

	/**
	 * @brief
	 */
	int collect();

	/**
	 * @brief
	 */
	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address    The I2C bus address to probe.
	 * @return           True if the device is present.
	 */
	int probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg        Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(void *arg);

	ringbuffer::RingBuffer *_reports{nullptr};

	orb_advert_t _distance_sensor_topic{nullptr};  // Change to _distance_sensor_topic.

	perf_counter_t _comms_errors;
	perf_counter_t _sample_perf;

	bool _collect_phase{false};
	bool _sensor_ok{false};

	int _class_instance{-1};
	int _measure_ticks{0};
	int _orb_class_instance{-1};

	std::vector<uint8_t> _sensor_addresses{};

	work_s _work{};
};

/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int mappydot_main(int argc, char *argv[]);

MappyDot::MappyDot(int bus, int address) :
	I2C("MappyDot", MAPPYDOT_DEVICE_PATH, bus, address, 100000),
	_sample_perf(perf_alloc(PC_ELAPSED, "mappydot_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mappydot_com_err"))
{
}

MappyDot::~MappyDot()
{
	// Ensure we are truly inactive.
	stop();

	// Free any existing reports.
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// Free perf counters.
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int
MappyDot::collect()
{
	int ret = -EIO;

	//set_device_address(_sensor_addresses[mappy_address]);

	// Read from the sensor.
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	//struct distance_sensor_s reports[_sensor_addresses.size()];

	for (unsigned mappy_address = 0; mappy_address < _sensor_addresses.size(); mappy_address++) {
		set_device_address(_sensor_addresses[mappy_address]);

		// @TODO - Why is this usleep occurring?
		usleep(50000);

		ret = transfer(nullptr, 0, &val[0], 2);

		if (ret < 0) {
			PX4_INFO("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		uint16_t distance_mm = val[0] << 8 | val[1];

		struct distance_sensor_s report;

		report.timestamp = hrt_absolute_time();
		report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;  //INFRARED DIDNT WORK WITH MAVROS?
		report.current_distance = distance_mm / 10;
		report.min_distance = MAPPYDOT_MIN_DISTANCE;
		report.max_distance = MAPPYDOT_MAX_DISTANCE;
		report.covariance = 0;
		report.signal_quality = 0;
		report.orientation = 0;
		report.id = get_device_address(); // used to be 0

		// Publish it, if we are the primary.
		if (_distance_sensor_topic != nullptr) {
			orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
		}

		_reports->force(&report);

		// Notify anyone waiting for data.
		poll_notify(POLLIN);
	}

	perf_end(_sample_perf);
	return OK;
}

void
MappyDot::cycle()
{
	if (_collect_phase) {

		// Perform collection.
		if (OK != collect()) {
			PX4_INFO("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		// Next phase is measurement.
		_collect_phase = false;

		if (_measure_ticks > USEC2TICK(MAPPYDOT_CONVERSION_INTERVAL)) {

			// Schedule a fresh cycle call when we are ready to measure again.
			work_queue(HPWORK, &_work, (worker_t) & MappyDot::cycle_trampoline, this,
				   _measure_ticks - USEC2TICK(MAPPYDOT_CONVERSION_INTERVAL));
			return;
		}
	}

	// Next phase is collection.
	_collect_phase = true;

	// Schedule a fresh cycle call when we are ready to measure again.
	work_queue(HPWORK, &_work, (worker_t) & MappyDot::cycle_trampoline, this,
		   _measure_ticks - USEC2TICK(MAPPYDOT_CONVERSION_INTERVAL));

}

void
MappyDot::cycle_trampoline(void *arg)
{
	MappyDot *dev = (MappyDot *)arg;

	dev->cycle();
}

int
MappyDot::init()
{
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Allocate basic report buffers.
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	set_device_address(MAPPYDOT_BASE_ADDR);

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// Get a publish handle on the obstacle distance topic.
	struct distance_sensor_s distance_sensor_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &distance_sensor_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	uint8_t sensor_address = MAPPYDOT_BASE_ADDR;

	// Check for connected rangefinders on each i2c port,
	// starting from the base address 0x08 and counting upwards
	for (unsigned counter = 0; counter <= MAPPYDOT_MAX_RANGEFINDERS; counter++) {
		sensor_address = MAPPYDOT_BASE_ADDR + counter;
		set_device_address(sensor_address);

		// Check if sensor is present and store I2C address.
		if (measure() == 0) {
			PX4_INFO("Add sensor");
			_sensor_addresses.push_back(sensor_address);
		}
	}

	PX4_INFO("Total MappyDots connected: %d", _sensor_addresses.size());

	for (unsigned add_counter = 0; add_counter < _sensor_addresses.size(); add_counter++) {
		PX4_INFO("MappyDot %d with address %d added", add_counter, _sensor_addresses[add_counter]);
	}

	set_device_address(MAPPYDOT_BASE_ADDR);
	_sensor_ok = true;
	return OK;
}

int
MappyDot::ioctl(device::file_t *file_pointer, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			// Switching to manual polling.
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			// External signalling (DRDY) not supported.
			case SENSOR_POLLRATE_EXTERNAL:

			// Zero would be bad.
			case 0:
				return -EINVAL;

			// Set default/max polling rate.
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					// Do we need to start internal polling?.
					bool want_start = (_measure_ticks == 0);

					// Set interval for next measurement to minimum legal value.
					_measure_ticks = USEC2TICK(MAPPYDOT_CONVERSION_INTERVAL);

					// If we need to start the poll state machine, do it.
					if (want_start) {
						start();

					}

					return OK;
				}

			// Adjust to a legal polling interval in Hz.
			default: {
					// Do we need to start internal polling?
					bool want_start = (_measure_ticks == 0);

					// Convert hz to tick interval via microseconds.
					int ticks = USEC2TICK(1000000 / arg);

					// Check against maximum rate.
					if (ticks < USEC2TICK(MAPPYDOT_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					// Update interval for next measurement.
					_measure_ticks = ticks;

					// If we need to start the poll state machine, do it.
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			// Lower bound is mandatory, upper bound is a sanity check.
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			ATOMIC_ENTER;

			if (!_reports->resize(arg)) {
				ATOMIC_LEAVE;
				return -ENOMEM;
			}

			ATOMIC_LEAVE;

			return OK;
		}

	case SENSORIOCRESET:
		// XXX implement this.
		return -EINVAL;

	default:
		// Give it to the superclass.
		return I2C::ioctl(file_pointer, cmd, arg);
	}
}

int
MappyDot::measure()
{
	uint8_t cmd = MAPPYDOT_PERFORM_SINGLE_RANGE;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != OK) {
		perf_count(_comms_errors);
		PX4_INFO("i2c::transfer returned %d", ret);
		return ret;
	}

	return OK;
}

void
MappyDot::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

ssize_t
MappyDot::read(device::file_t *file_pointer, char *buffer, size_t buflen)
{
	int ret = 0;

	unsigned int count = buflen / sizeof(struct distance_sensor_s);

	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is enabled.
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		// If there was no data, warn the caller.
		return ret ? ret : -EAGAIN;
	}

	// Manual measurement - run one conversion.
	do {
		_reports->flush();

		// Trigger a measurement.
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		// Wait for it to complete.
		usleep(MAPPYDOT_CONVERSION_INTERVAL * 2);

		// Run the collection phase.
		if (collect() != OK) {
			ret = -EIO;
			break;
		}

		// State machine will have generated a report, copy it out.
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}
	} while (0);

	return ret;
}

void
MappyDot::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;
	_reports->flush();

	// Schedule a cycle to start things.
	work_queue(HPWORK, &_work, (worker_t)&MappyDot::cycle_trampoline, this, 1); // Last var in work_queue was 5, not 1.
}

void
MappyDot::stop()
{
	work_cancel(HPWORK, &_work);
}

/**
 * Local functions in support of the shell command.
 */
namespace mappydot
{

MappyDot *g_dev;

int info();
int reset();
int start();
int start_bus(int i2c_bus);
int stop();
int test();

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return OK;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = px4_open(MAPPYDOT_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	px4_close(fd);
	return OK;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == OK) {
			return OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new Mappydot(i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(MAPPYDOT_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	px4_close(fd);
	return OK;

fail:

	if (fd >= 0) {
		px4_close(fd);
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver.
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = px4_open(MAPPYDOT_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'mappydot start' if the driver is not running)", MAPPYDOT_DEVICE_PATH);
		return PX4_ERROR;
	}

	// Do a simple demand read.
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	// Start the sensor polling at 2Hz.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	// Read the sensor 5x and report each value.
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		// Wait for data to be ready.
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		// Now go get it.
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	// Reset the sensor polling to default rate.
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	PX4_INFO("PASS");
	return OK;
}

} // namespace mappydot


static void
mappydot_usage()
{
	PX4_INFO("Usage: mappydot <command> [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", MAPPYDOT_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|reset|info");
}

int
mappydot_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = MAPPYDOT_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			//PX4_INFO("Specific I2C Bus started: %d", i2c_bus);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	// Start/load the driver.
	PX4_INFO("Starting Driver now");

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			PX4_INFO("Starting regular");
			return mappydot::start();

		} else {
			PX4_INFO("Starting specific bus type %d", i2c_bus);
			return mappydot::start_bus(i2c_bus);
		}
	}

	PX4_INFO("Started Driver");

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return mappydot::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return mappydot::test();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return mappydot::reset();
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return mappydot::info();
	}

out_error:
	mappydot_usage();
	return PX4_ERROR;
}
