/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AS5048B.cpp
 * Driver for AS5048B
 */

#include <math.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <sys/types.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_wind_angle.h>

using namespace time_literals;

#define AS5048B_BUS_DEFAULT                      1  // PX4_I2C_BUS_ONBOARD
#define AS5048B_BASE_DEVICE_PATH                 "/dev/as5048b"
#define AS5048B_BUS_CLOCK                        1000000 // 100 kHz

static constexpr uint8_t AS5048B_BASE_ADDR       = 0x40;

/* Measurement rate is 100Hz */
static constexpr uint64_t MEASUREMENT_INTERVAL   = 10_ms;

/* Sensor Related */
static constexpr uint8_t  AS5048B_PROG_REG       = 0x03;
static constexpr uint8_t  AS5048B_ADDR_REG       = 0x15;
static constexpr uint8_t  AS5048B_ZEROMSB_REG    = 0x16;    // bits 0..7
static constexpr uint8_t  AS5048B_ZEROLSB_REG    = 0x17;    // bits 0..5
static constexpr uint8_t  AS5048B_GAIN_REG       = 0xFA;
static constexpr uint8_t  AS5048B_DIAGNOSTIC_REG = 0xFB;
static constexpr uint8_t  AS5048B_MAGNMSB_REG    = 0xFC;    // bits 0..7
static constexpr uint8_t  AS5048B_MAGNLSB_REG    = 0xFD;    // bits 0..5
static constexpr uint8_t  AS5048B_ANGLMSB_REG    = 0xFE;    // bits 0..7
static constexpr uint8_t  AS5048B_ANGLLSB_REG    = 0xFF;    // bits 0..5
static constexpr uint16_t AS5048B_RESOLUTION     = 16384;   // 14 bits

/**
 * Moving Exponential Average on angle - beware heavy calculation for some Arduino boards
 * This is a 1st order low pass filter
 * Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
 * Keep in mind the moving average will be impacted by the measurement frequency too.
 */
static constexpr uint8_t EXP_MOVAVG_N    = 5; // History length impact on moving average impact.
static constexpr uint8_t EXP_MOVAVG_LOOP = 1; // Number of measurements before starting moving Average.

/** Unit constants for readability. */
static constexpr uint8_t U_RAW           = 1;
static constexpr uint8_t U_TRN           = 2;
static constexpr uint8_t U_DEG           = 3;
static constexpr uint8_t U_RAD           = 4;
static constexpr uint8_t U_GRAD          = 5;
static constexpr uint8_t U_MOA           = 6;
static constexpr uint8_t U_SOA           = 7;
static constexpr uint8_t U_MILNATO       = 8;
static constexpr uint8_t U_MILSE         = 9;
static constexpr uint8_t U_MILRU         = 10;


class AMS_AS5048B : public device::I2C, public px4::ScheduledWorkItem
{
public:
	AMS_AS5048B(const uint8_t bus, const uint8_t address = AS5048B_BASE_ADDR,
		    const char *path = AS5048B_BASE_DEVICE_PATH);

	virtual ~AMS_AS5048B();

	virtual int init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Start the driver.
	 */
	void start();

	/**
	 * Stop the driver.
	 */
	void stop();

private:

	int collect();

	/**
	 * RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
	 */
	float convert_angle(const int unit, const float angle);

	/**
	 * Flash values to the slave address OTP register.
	 */
	void flash_prog();

	/**
	 * Flash values to the zero position OTP register.
	 */
	void flash_prog_zero();

	/**
	 * Reads the 1 bytes auto gain register value
	 * @returns Returns the auto gain register value.
	 */
	uint8_t get_auto_gain();

	/**
	 * Reads the 1 bytes diagnostic register value
	 * @returns Returns the diagnostic register value
	 */
	uint8_t read_diagnostic_reg();

	float get_exp_avg_raw_angle();

	/**
	 * Gets the exponential moving averaged angle in the desired units.
	 * @param unit The desired units applied to the angle value being retrieved.
	 * @returns Returns the exponential moving averaged angle value.
	 */
	float get_moving_avg_exp(const int unit = U_RAW);

	/**
	 * Reads the 2 bytes magnitude register value.
	 * @return Returns the magnitude register value trimmed on 14 bits.
	 */
	uint16_t magnitude_R();

	/**
	 * Writes OTP control register.
	 * @param reg_val register value to be written.
	 */
	void prog_register(const uint8_t reg_val);

	/**
	 * Reads the I2C address register value.
	 * @return Returns the address register value.
	 */
	uint8_t read_address_reg();

	/**
	 * Reads the current angle value and converts it into the desired unit or
	 * gets last measurement with unit conversion :
	 *    RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
	 * @param unit The desired units applied to the angle value being retrieved.
	 * @param new_measurement  Boolean to indicate if a new measurement should be taken or the previous value returned.
	 * @return Returns the current angle value with units conversion.
	 */
	float read_angle(const int unit = U_RAW, const bool new_measurement = true);

	/**
	 * Read raw value of the angle register.
	 */
	uint16_t read_angle_reg();

	uint8_t read_reg_8(const uint8_t address);

	/**
	 * 16 bit value got from 2x8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value.
	 */
	uint16_t read_reg_16(const uint8_t address);

	/**
	 * Reads the 2 bytes Zero position register value.
	 * @return Returns the Zero register value trimmed on 14 bits.
	 */
	uint16_t read_zero_reg();

	/**
	 * Reset Exponential Moving Average calculation values.
	 */
	void reset_moving_avg_exp();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Set/unset clock wise counting - sensor counts CCW natively, default false (native sensor).
	 * @param cw true: CW, false: CCW
	 */
	void set_clock_wise(const bool cw = false);

	/**
	 * Sets the current angle as the zero position.
	 */
	void set_zero_reg();

	/**
	 * Toggles the debug output start/stop to serial debug.
	 */
	void toggle_debug();

	/**
	 * Performs an exponential moving average on the angle.
	 * Works on Sine and Cosine of the angle to avoid issues 0°/360° discontinuity
	 */
	void update_moving_avg_exp();

	/**
	 * Writes the I2C address value (5 bits) into the address register, changing the chip address.
	 * @param reg_val The register value to be written.
	 */
	void write_address_reg(const uint8_t reg_val);

	void write_reg(const uint8_t address, const uint8_t reg_val);

	/**
	 * Writes the 2 bytes Zero position register value.
	 * @param reg_val The Zero register value to be written.
	 */
	void write_zero_reg(const uint16_t reg_val);

	bool _clock_wise{false};
	bool _debug_flag{false};
	bool _initialized{false};

	int _measure_interval{MEASUREMENT_INTERVAL};
	int _moving_avg_count_loop{0};
	int _orb_class_instance{-1};

	uint8_t _address_reg_val{0};
	uint8_t	_chip_address{0};

	uint16_t _scale{0};
	uint16_t _zero_reg_val{0};

	float _last_angle_raw{0.f};
	float _moving_avg_exp_alpha{0.f};
	float _moving_avg_exp_angle{0.f};
	float _moving_avg_exp_cos{0.f};
	float _moving_avg_exp_sin{0.f};

	orb_advert_t _wind_angle_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "as5048b_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "as5048b_read")};
};


AMS_AS5048B::AMS_AS5048B(const uint8_t bus, const uint8_t address, const char *path) :
	I2C("as5048", path, bus, address, AS5048B_BUS_CLOCK),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id()))
{
	_chip_address = address;
}

AMS_AS5048B::~AMS_AS5048B()
{
	// Ensure we are truly inactive.
	stop();

	// Unadvertise uORB topics.
	if (_wind_angle_topic != nullptr) {
		orb_unadvertise(_wind_angle_topic);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
AMS_AS5048B::collect()
{
	perf_begin(_sample_perf);

	sensor_wind_angle_s report{};

	report.timestamp           = hrt_absolute_time();
	report.wind_magnetic_angle = read_angle();

	orb_publish_auto(ORB_ID(sensor_wind_angle), &_wind_angle_topic, &report,
			 &_orb_class_instance, ORB_PRIO_DEFAULT);

	perf_count(_sample_perf);
	perf_end(_sample_perf);

	return PX4_OK;
}

float
AMS_AS5048B::convert_angle(const int unit, const float angle)
{
	// Convert raw sensor reading into angle unit.
	float angleConv = 0.f;

	switch (unit) {
	case U_RAW:
		// Sensor raw measurement
		angleConv = angle;
		break;

	case U_TRN:
		// Full turn ratio
		angleConv = (angle / AS5048B_RESOLUTION);
		break;

	case U_DEG:
		// Degree
		angleConv = (angle / AS5048B_RESOLUTION) * 360.0f;
		break;

	case U_RAD:
		// Radian
		angleConv = (angle / AS5048B_RESOLUTION) * static_cast<float>(2.0 * M_PI);
		break;

	case U_MOA:
		// Minute of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.f * 360.f;
		break;

	case U_SOA:
		// Second of arc
		angleConv = (angle / AS5048B_RESOLUTION) * 60.f * 60.f * 360.f;
		break;

	case U_GRAD:
		// Grade
		angleConv = (angle / AS5048B_RESOLUTION) * 400.f;
		break;

	case U_MILNATO:
		// NATO MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6400.f;
		break;

	case U_MILSE:
		// Swedish MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6300.f;
		break;

	case U_MILRU:
		// Russian MIL
		angleConv = (angle / AS5048B_RESOLUTION) * 6000.f;
		break;

	default:
		// No conversion => raw angle measurement
		angleConv = angle;
		break;
	}

	return angleConv;
}

void
AMS_AS5048B::flash_prog()
{
	// Enable special programming mode.
	prog_register(0xFD);
	usleep(10000);

	// Set the burn bit: enables automatic programming procedure.
	prog_register(0x08);
	usleep(10000);

	// Disable special programming mode.
	prog_register(0x00);
	usleep(10000);
}

void
AMS_AS5048B::flash_prog_zero()
{
	// This will burn the zero position OTP register like described
	// in the datasheet to enable programming mode.
	prog_register(0x01);
	usleep(10000);

	// Set the burn bit: enables automatic programming procedure.
	prog_register(0x08);
	usleep(10000);

	// Read angle information (equals to 0).
	read_reg_16(AS5048B_ANGLMSB_REG);
	usleep(10000);

	// Enable verification.
	prog_register(0x40);
	usleep(10000);

	// Read angle information (equals to 0).
	read_reg_16(AS5048B_ANGLMSB_REG);
	usleep(10000);
}

int
AMS_AS5048B::init()
{
	set_device_address(AS5048B_BASE_ADDR);

	// Perform I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	_zero_reg_val    = read_zero_reg();
	_address_reg_val = read_address_reg();

	reset_moving_avg_exp();

	_initialized = true;

	float angle = read_angle(U_RAW, true);

	if (angle < 16384) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

uint8_t
AMS_AS5048B::get_auto_gain()
{
	return read_reg_8(AS5048B_GAIN_REG);
}

float
AMS_AS5048B::get_exp_avg_raw_angle()
{
	float angle = 0.f;
	float two_PI = 2 * M_PI;

	if (_moving_avg_exp_sin < 0) {
		// Ensure positive angle.
		angle = two_PI - static_cast<float>(acos(_moving_avg_exp_cos));

	} else {
		angle = static_cast<float>(acos(_moving_avg_exp_cos));
	}

	angle *= AS5048B_RESOLUTION / two_PI;
	return angle;
}

float
AMS_AS5048B::get_moving_avg_exp(const int unit)
{
	return convert_angle(unit, _moving_avg_exp_angle);
}

uint16_t
AMS_AS5048B::magnitude_R()
{
	return read_reg_16(AS5048B_MAGNMSB_REG);
}

void
AMS_AS5048B::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("measurement interval:  %u\n", _measure_interval);
}

void
AMS_AS5048B::prog_register(const uint8_t reg_val)
{
	write_reg(AS5048B_PROG_REG, reg_val);
}

uint8_t
AMS_AS5048B::read_address_reg()
{
	return read_reg_8(AS5048B_ADDR_REG);
}

void
AMS_AS5048B::write_address_reg(const uint8_t reg_val)
{
	// Write the new chip address to the register
	write_reg(AS5048B_ADDR_REG, reg_val);

	// Update our chip address with our 5 programmable bits
	// the MSB is internally inverted, so we flip the leftmost bit
	_chip_address = ((reg_val << 2) | (_chip_address & 0b11)) ^ (1 << 6);
}

float
AMS_AS5048B::read_angle(const int unit, const bool new_measurement)
{
	float angle_raw = 0.f;

	if (new_measurement) {
		if (_clock_wise) {
			angle_raw = static_cast<float>((0b11111111111111 - read_reg_16(AS5048B_ANGLMSB_REG)));

		} else {
			angle_raw = static_cast<float>(read_reg_16(AS5048B_ANGLMSB_REG));
		}

		_last_angle_raw = angle_raw;

	} else {
		angle_raw = _last_angle_raw;
	}

	return convert_angle(unit, angle_raw);
}

uint16_t
AMS_AS5048B::read_angle_reg()
{
	return read_reg_16(AS5048B_ANGLMSB_REG);
}

uint8_t
AMS_AS5048B::read_diagnostic_reg()
{
	return read_reg_8(AS5048B_DIAGNOSTIC_REG);
}

uint8_t
AMS_AS5048B::read_reg_8(const uint8_t address)
{
	// First send to sensor register that we want to read
	uint8_t cmd = address;
	int ret = transfer(&cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		return ret;
	}

	uint8_t read_value = 0;
	ret = transfer(nullptr, 0, &read_value, sizeof(read_value));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return read_value;
}

uint16_t
AMS_AS5048B::read_reg_16(const uint8_t address)
{
	//16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
	uint16_t read_value = 0;

	// First send to sensor register that we want to read
	uint8_t cmd = address;
	int ret = transfer(&cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
	}

	// Read the value that comes back
	uint8_t val[2];
	ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	read_value = (((uint16_t) val[0]) << 6);
	read_value += (val[1] & 0x3F);

	return read_value;
}

uint16_t
AMS_AS5048B::read_zero_reg()
{
	return read_reg_16(AS5048B_ZEROMSB_REG);
}

void
AMS_AS5048B::Run()
{
	if (!_initialized) {
		_initialized = init();
	}

	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("sensor measurement collection error");
		return;
	}
}

void
AMS_AS5048B::reset_moving_avg_exp()
{
	_moving_avg_exp_angle  = 0.f;
	_moving_avg_count_loop = 0;
	_moving_avg_exp_alpha  = 2.f / (EXP_MOVAVG_N + 1.f);
}

void
AMS_AS5048B::set_clock_wise(const bool cw)
{
	_clock_wise = cw;
	_last_angle_raw = 0.f;
	reset_moving_avg_exp();
}

void
AMS_AS5048B::set_zero_reg()
{
	// Issue closed by @MechatronicsWorkman and @oilXander.
	// The last sequence avoids any offset for the new Zero position
	write_zero_reg((uint16_t) 0x00);
	uint16_t new_zero = read_reg_16(AS5048B_ANGLMSB_REG);
	write_zero_reg(new_zero);
}

void
AMS_AS5048B::start()
{
	// Schedule the driver to run at regular intervals.
	PX4_INFO("driver started.");
	ScheduleOnInterval(_measure_interval);
}

void
AMS_AS5048B::stop()
{
	ScheduleClear();
}

void
AMS_AS5048B::toggle_debug()
{
	_debug_flag = !_debug_flag;
}

void
AMS_AS5048B::update_moving_avg_exp()
{
	// Sine and cosine calculation on angles in radians.
	float angle = read_angle(U_RAD, true);

	if (_moving_avg_count_loop < EXP_MOVAVG_LOOP) {
		_moving_avg_exp_sin += static_cast<float>(sin(angle));
		_moving_avg_exp_cos += static_cast<float>(cos(angle));

		if (_moving_avg_count_loop == (EXP_MOVAVG_LOOP - 1)) {
			_moving_avg_exp_sin = _moving_avg_exp_sin / EXP_MOVAVG_LOOP;
			_moving_avg_exp_cos = _moving_avg_exp_cos / EXP_MOVAVG_LOOP;
		}

		_moving_avg_count_loop ++;

	} else {
		_moving_avg_exp_sin += _moving_avg_exp_alpha * (static_cast<float>(sin(angle)) - _moving_avg_exp_sin);
		_moving_avg_exp_cos += _moving_avg_exp_alpha * (static_cast<float>(cos(angle)) - _moving_avg_exp_cos);
		_moving_avg_exp_angle  = get_exp_avg_raw_angle();
	}
}

void
AMS_AS5048B::write_reg(const uint8_t address, const uint8_t reg_val)
{
	uint8_t cmd[2] = {address, reg_val};

	int ret = transfer(cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("write_reg()");
	}
}

void
AMS_AS5048B::write_zero_reg(const uint16_t reg_val)
{
	write_reg(AS5048B_ZEROMSB_REG, (uint8_t)(reg_val >> 6));
	write_reg(AS5048B_ZEROLSB_REG, (uint8_t)(reg_val & 0x3F));
}


/**
 * Local functions in support of the shell command.
 */
namespace as5048b
{
AMS_AS5048B *g_dev = nullptr;

int reset();
int start();
int start_bus(const uint8_t i2c_bus = AS5048B_BASE_ADDR);
int status();
int stop();
int test();
int usage();

/**
 * Reset the driver.
 */
int
reset()
{
	if (g_dev != nullptr) {
		g_dev->stop();
		g_dev->start();
		return PX4_OK;
	}

	return PX4_ERROR;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
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
start_bus(const uint8_t i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new AMS_AS5048B(i2c_bus, AS5048B_BASE_ADDR, AS5048B_BASE_DEVICE_PATH);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();
	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
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
	int fd = px4_open(AS5048B_BASE_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'as5048b start -a' if the driver is not running)", AS5048B_BASE_DEVICE_PATH);
		return PX4_ERROR;
	}

	px4_close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

/**
 * Print usage info about the driver.
 */
int
usage()
{
	PX4_INFO("usage: as5048b command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", AS5048B_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|reset");
	return PX4_OK;
}

} // namespace as5048b


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int as5048b_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	bool start_all = false;

	int myoptind = 1;
	int ch;

	uint8_t i2c_bus = AS5048B_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			PX4_INFO("Specific I2C Bus selected: %i", i2c_bus);
			break;

		default:
			PX4_WARN("Unknown option!");
			return as5048b::usage();
		}
	}

	if (myoptind >= argc) {
		return as5048b::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return as5048b::reset();
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return as5048b::start();

		} else {
			return as5048b::start_bus(i2c_bus);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return as5048b::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return as5048b::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return as5048b::test();
	}

	return as5048b::usage();
}
