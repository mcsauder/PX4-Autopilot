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
 * @file MappyDot.h
 * @author Mohammed Kabir (mhkabir@mit.edu)
 *
 * Driver for the Mappydot infrared rangefinders connected via I2C.
 */

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <board_config.h>
#include <containers/Array.hpp>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module_params.h>
#include <px4_workqueue.h>
#include <sys/types.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>


/* Configuration Constants */
#define MAPPYDOT_BASE_ADDR        0x08
#define MAPPYDOT_BUS_DEFAULT      PX4_I2C_BUS_EXPANSION2
#define MAPPYDOT_DEVICE_PATH      "/dev/mappydot"

class MappyDot : public device::I2C, public ModuleParams
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
	virtual int ioctl(device::file_t *file_pointer, const int cmd, const unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * @brief
	 */
	virtual ssize_t read(const device::file_t *file_pointer, char *buffer, const size_t buflen);

protected:

private:

	/**
	 * @brief
	 */
	int collect();

	/**
	 * @brief Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(const void *arg);

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
	int probe_address(const uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	int start();

	/**
	 * @brief Stop the automatic measurement state machine.
	 */
	int stop();

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	ringbuffer::RingBuffer *_reports{nullptr};

	orb_advert_t _distance_sensor_topic{nullptr};  // Change to _distance_sensor_topic.

	perf_counter_t _comms_errors{perf_alloc(PC_ELAPSED, "mappydot_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "mappydot_read")};

	bool _is_running{false};
	bool _sensor_ok{false};

	int _class_instance{-1};
	int _measure_ticks{0};
	int _orb_class_instance{-1};
	int _param_sub{0};

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};

	work_s _work{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_MPDT>) _p_sensor_enabled
	);
};
