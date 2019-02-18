/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file MappyDot.cpp
 * @author Mohammed Kabir (mhkabir@mit.edu)
 *
 * Driver for the Mappydot infrared rangefinders connected via I2C.
 */


#include "MappyDot.h"


/* MappyDot Registers */
/* Basics */
#define MAPPYDOT_MEASUREMENT_BUDGET                         0x42
#define MAPPYDOT_READ_ERROR_CODE                            0x45
#define MAPPYDOT_CHECK_INTERRUPT                            0x49
#define MAPPYDOT_READ_ACCURACY                              0x52
#define MAPPYDOT_PERFORM_SINGLE_RANGE                       0x53
#define MAPPYDOT_SET_CONTINUOUS_RANGING_MODE                0x63
#define MAPPYDOT_RANGING_MEASUREMENT_MODE                   0x6D
#define MAPPYDOT_READ_DISTANCE                              0x72
#define MAPPYDOT_SET_SINGLE_RANGING_MODE                    0x73

/* Configuration */
#define MAPPYDOT_FILTERING_ENABLE                           0x46
#define MAPPYDOT_SIGNAL_LIMIT_CHECK_VALUE                   0x47
#define MAPPYDOT_ENABLE_CROSSTALK_COMPENSATION              0x4B
#define MAPPYDOT_SIGMA_LIMIT_CHECK_VALUE                    0x4C
#define MAPPYDOT_INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY    0x51
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_ENABLE     0x54
#define MAPPYDOT_AVERAGING_ENABLE                           0x56
#define MAPPYDOT_INTERSENSOR_SYNC_ENABLE                    0x59
#define MAPPYDOT_CALIBRATE_DISTANCE_OFFSET                  0x61
#define MAPPYDOT_SET_LED_THRESHOLD_DISTANCE_IN_MM           0x65
#define MAPPYDOT_FILTERING_DISABLE                          0x66
#define MAPPYDOT_SET_GPIO_MODE                              0x67
#define MAPPYDOT_AVERAGING_SAMPLES                          0x69
#define MAPPYDOT_DISABLE_CROSSTALK_COMPENSATION             0x6B
#define MAPPYDOT_SET_LED_MODE                               0x6C
#define MAPPYDOT_SET_GPIO_THRESHOLD_DISTANCE_IN_MM          0x6F
#define MAPPYDOT_REGION_OF_INTEREST                         0x70
#define MAPPYDOT_INTERSENSOR_CROSSTALK_TIMEOUT              0x71
#define MAPPYDOT_INTERSENSOR_CROSSTALK_REDUCTION_DISABLE    0x74
#define MAPPYDOT_CALIBRATE_SPAD                             0x75
#define MAPPYDOT_AVERAGING_DISABLE                          0x76
#define MAPPYDOT_CALIBRATE_CROSSTALK                        0x78
#define MAPPYDOT_INTERSENSOR_SYNC_DISABLE                   0x79

/* Settings */
#define MAPPYDOT_FIRMWARE_VERSION                           0x4E
#define MAPPYDOT_READ_CURRENT_SETTINGS                      0x62
#define MAPPYDOT_DEVICE_NAME                                0x64
#define MAPPYDOT_NAME_DEVICE                                0x6E
#define MAPPYDOT_WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT 0x77
#define MAPPYDOT_RESTORE_FACTORY_DEFAULTS                   0x7A

/* Advanced */
#define MAPPYDOT_AMBIENT_RATE_RETURN                        0x41
#define MAPPYDOT_VL53L1X_NOT_SHUTDOWN                       0x48
#define MAPPYDOT_SIGNAL_RATE_RETURN                         0x4A
#define MAPPYDOT_RESET_VL53L1X_RANGING                      0x58
#define MAPPYDOT_VL53L1X_SHUTDOWN                           0x68
#define MAPPYDOT_READ_NONFILTERED_VALUE                     0x6A

/* Super Advanced */
#define MAPPYDOT_ENTER_FACTORY_MODE                         0x23 //"#"//"!#!#!#"
#define MAPPYDOT_WIPE_ALL_SETTINGS                          0x3C //"<"//"><><><" (Must be in factory mode)

/* Ranging Modes */
#define MAPPYDOT_LONG_RANGE                                 0x6C
#define MAPPYDOT_MED_RANGE                                  0x6D
#define MAPPYDOT_SHORT_RANGE                                0x73

/* LED Modes */
#define MAPPYDOT_LED_OFF                                    0x66
#define MAPPYDOT_LED_MEASUREMENT_OUTPUT                     0x6D
#define MAPPYDOT_LED_ON                                     0x6F
#define MAPPYDOT_LED_PWM_ENABLED                            0x70
#define MAPPYDOT_LED_THRESHOLD_ENABLED                      0x74

/* GPIO Modes */
#define MAPPYDOT_GPIO_LOW                                   0x66
#define MAPPYDOT_GPIO_MEASUREMENT_INTERRUPT                 0x6D
#define MAPPYDOT_GPIO_HIGH                                  0x6F
#define MAPPYDOT_GPIO_PWM_ENABLED                           0x70
#define MAPPYDOT_GPIO_THRESHOLD_ENABLED                     0x74

/* I2C Bootloader */
#define MAPPYDOT_REBOOT_TO_BOOTLOADER                       0x01

/* Device limits */
#define MAPPYDOT_MIN_DISTANCE                               0.20f // meters
#define MAPPYDOT_MAX_DISTANCE                               4.00f // meters

#define MAPPYDOT_MEASUREMENT_INTERVAL                        100000 /* 10ms */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

MappyDot::MappyDot(int bus, int address) :
	I2C("MappyDot", MAPPYDOT_DEVICE_PATH, bus, address, MAPPYDOT_MEASUREMENT_INTERVAL),
	ModuleParams(nullptr)
{
	_param_sub = orb_subscribe(ORB_ID(parameter_update));
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

	// Unsubscribe from uORB topics.
	orb_unsubscribe(_param_sub);

	// Free perf counters.
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int
MappyDot::collect()
{
	int ret = -EIO;

	// Read from the sensor.
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	for (size_t index = 0; index < _sensor_addresses.size(); index++) {
		set_device_address(_sensor_addresses[index]);

		// @TODO - Why is this usleep occurring?
		// usleep(50000);

		ret = transfer(nullptr, 0, &val[0], 2);

		if (ret < 0) {
			PX4_INFO("error reading from sensor: %d", index);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		uint16_t distance_mm = val[0] << 8 | val[1];

		struct distance_sensor_s report;

		report.covariance       = 0;
		report.current_distance = distance_mm / 10;
		report.id               = get_device_address(); // used to be 0
		report.max_distance     = MAPPYDOT_MAX_DISTANCE;
		report.min_distance     = MAPPYDOT_MIN_DISTANCE;
		report.orientation      = 0;
		report.signal_quality   = 0;
		report.timestamp        = hrt_absolute_time();
		report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

		// Publish it, if we are the primary.
		if (_distance_sensor_topic != nullptr) {
			orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
		}

		_reports->force(&report);

		// Notify anyone waiting for data.
		poll_notify(POLLIN);
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

void
MappyDot::cycle()
{
	// Perform collection.
	if (collect() != PX4_OK) {
		PX4_INFO("sensor measurement collection error");
		// If error restart the measurement state machine.
		start();
		return;
	}

	// Schedule a fresh cycle call when we are ready to measure again.
	work_queue(HPWORK, &_work, (worker_t)&MappyDot::cycle_trampoline, this,
		   USEC2TICK(MAPPYDOT_MEASUREMENT_INTERVAL));
}

void
MappyDot::cycle_trampoline(const void *arg)
{
	MappyDot *dev = (MappyDot *)arg;

	dev->cycle();
}

int
MappyDot::init()
{
	int sensor_enabled = _p_sensor_enabled.get();

	if (sensor_enabled == 0) {
		PX4_WARN("Disabled");
		return PX4_ERROR;
	}

	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Allocate basic report buffers.
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// Get a publish handle on the obstacle distance topic.
	struct distance_sensor_s distance_sensor_report = {};

	_reports->get(&distance_sensor_report);

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &distance_sensor_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
		return PX4_ERROR;
	}

	uint8_t sensor_address = MAPPYDOT_BASE_ADDR;
	size_t sensor_count = 0;

	// Check for connected rangefinders on each i2c port,
	// starting from the base address 0x08 and incrementing.
	for (size_t i = 0; i <= RANGE_FINDER_MAX_SENSORS; i++) {
		set_device_address(sensor_address);

		// Check if sensor is present and store I2C address.
		if (measure() == 0) {
			_sensor_addresses[i] = sensor_address;
			sensor_address++;
			sensor_count++;
			PX4_INFO("sensor %d at address %d added", i, _sensor_addresses[i]);

		} else {
			break;
		}
	}

	PX4_INFO("%d sensors connected", sensor_count);

	_sensor_ok = true;
	return PX4_OK;
}

int
MappyDot::ioctl(device::file_t *file_pointer, const int cmd, const unsigned long arg)
{
	bool should_start = (_measure_ticks == 0);

	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			// Zero would be bad.
			case 0:
				return -EINVAL;

			// Set max polling rate.
			case SENSOR_POLLRATE_DEFAULT: {
					// Set interval for next measurement to minimum legal value.
					_measure_ticks = USEC2TICK(MAPPYDOT_MEASUREMENT_INTERVAL);

					// Start the poll state machine.
					if (should_start) {
						start();
					}

					return PX4_OK;
				}

			// Adjust to a legal polling interval in Hz.
			default: {
					// Convert hz to tick interval via microseconds.
					int ticks = USEC2TICK(1000000 / arg);

					// Check against maximum rate.
					if (ticks < USEC2TICK(MAPPYDOT_MEASUREMENT_INTERVAL)) {
						return -EINVAL;

					} else {
						// Update interval for next measurement.
						_measure_ticks = ticks;
					}

					// Start the poll state machine.
					if (should_start) {
						start();
					}

					return PX4_OK;
				}
			}
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
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
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
MappyDot::read(const device::file_t *file_pointer, char *buffer, const size_t buffer_length)
{
	size_t buffer_size = 0;
	unsigned int count = buffer_length / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *read_buffer = reinterpret_cast<struct distance_sensor_s *>(buffer);

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
			if (_reports->get(read_buffer)) {
				buffer_size += sizeof(*read_buffer);
				read_buffer++;
			}
		}

		// If there was no data, warn the caller.
		if (buffer_size == 0) {
			return -EAGAIN;
		}
	}

	// Manual measurement - run one conversion.
	_reports->flush();

	// Trigger a measurement.
	if (measure() != PX4_OK) {
		return -EIO;
	}

	// Wait for it to complete.
	usleep(MAPPYDOT_MEASUREMENT_INTERVAL);

	// Run the collection phase.
	if (collect() != PX4_OK) {
		return -EIO;
	}

	// State machine will have generated a report, copy it out.
	if (_reports->get(read_buffer)) {
		buffer_size = sizeof(*read_buffer);
	}

	return buffer_size;
}

int
MappyDot::start()
{
	// Reset the report ring and state machine.
	_reports->flush();

	// Schedule a cycle to start things.
	work_queue(HPWORK, &_work, (worker_t)&MappyDot::cycle_trampoline, this, 0);


	if (_is_running) {
		PX4_INFO("Driver already running.");
		return PX4_OK;
	}

	update_params(true);

	// Kick off the cycling. We can call it directly because we're already in the work queue context
	cycle();

	PX4_INFO("Driver started successfully.");

	return PX4_OK;
}

int
MappyDot::stop()
{
	_is_running = false;
	work_cancel(HPWORK, &_work);
	return PX4_OK;
}

void
MappyDot::update_params(const bool force)
{
	bool updated;
	parameter_update_s param_update;

	orb_check(_param_sub, &updated);

	if (updated || force) {
		ModuleParams::updateParams();
		orb_copy(ORB_ID(parameter_update), _param_sub, &param_update);
	}
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
		PX4_INFO("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
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
	return PX4_OK;
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

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == OK) {
			return PX4_OK;
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
	g_dev = new MappyDot(i2c_bus);

	if (g_dev == nullptr ||
	    g_dev->init() != OK) {
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
	return PX4_OK;

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

	return PX4_OK;
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
	for (size_t i = 0; i < 5; i++) {
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
	return PX4_OK;
}

} // namespace mappydot


static void
mappydot_usage()
{
	PX4_INFO("Usage: mappydot <command> [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-a --all");
	PX4_INFO("\t-b --bus i2cbus (%d)", MAPPYDOT_BUS_DEFAULT);
	PX4_INFO("command:");
	PX4_INFO("\tinfo|reset|start|status|stop|test");
}


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int mappydot_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = MAPPYDOT_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			PX4_INFO("Specific I2C Bus started: %d", i2c_bus);
			break;

		default:
			PX4_WARN("Unknown option!");
			mappydot_usage();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		mappydot_usage();
		return PX4_ERROR;
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return mappydot::reset();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			PX4_INFO("Starting driver");
			return mappydot::start();

		} else {
			PX4_INFO("Starting specific bus type %d", i2c_bus);
			return mappydot::start_bus(i2c_bus);
		}
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return mappydot::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return mappydot::test();
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "help") ||
	    !strcmp(argv[myoptind], "info") ||
	    !strcmp(argv[myoptind], "status")) {
		return mappydot::info();
	}

	mappydot_usage();
	return PX4_ERROR;
}
