/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_multirotor.generated.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

namespace navio_sysfs_pwm_out
{
static px4_task_t _task_handle = -1;
volatile bool _task_should_exit = false;
static bool _is_running = false;

static const int NUM_PWM = 4;
static char _device[32] = "/sys/class/pwm/pwmchip0";
static int  _pwm_fd[NUM_PWM];

static const int FREQUENCY_PWM = 400;
static const char *MIXER_FILENAME = "";

// subscriptions
int     _controls_sub;
int     _armed_sub;

// publications
orb_advert_t    _outputs_pub = nullptr;
orb_advert_t    _rc_pub = nullptr;

// topic structures
actuator_controls_s _controls;
actuator_outputs_s  _outputs;
actuator_armed_s    _armed;

pwm_limit_t     _pwm_limit;

// esc parameters
int32_t _pwm_disarmed;
int32_t _pwm_min;
int32_t _pwm_max;

MultirotorMixer *_mixer = nullptr;

void usage();

void start();

void stop();

int pwm_write_sysfs(char *path, int value);

int pwm_initialize(const char *device);

void pwm_deinitialize();

void send_outputs_pwm(const uint16_t *pwm);

void task_main_trampoline(int argc, char *argv[]);

void task_main(int argc, char *argv[]);

/* mixer initialization */
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);


int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	return 0;
}


int initialize_mixer(const char *mixer_filename)
{
	char buf[2048];
	size_t buflen = sizeof(buf);

	PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

	int fd_load = ::open(mixer_filename, O_RDONLY);

	if (fd_load != -1) {
		int nRead = ::read(fd_load, buf, buflen);
		close(fd_load);

		if (nRead > 0) {
			_mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

			if (_mixer != nullptr) {
				PX4_INFO("Successfully initialized mixer from config file");
				return 0;

			} else {
				PX4_ERR("Unable to parse from mixer config file");
				return -1;
			}

		} else {
			PX4_WARN("Unable to read from mixer config file");
			return -2;
		}

	} else {
		PX4_WARN("No mixer config file found, using default mixer.");

		/* Mixer file loading failed, fall back to default mixer configuration for
		* QUAD_X airframe. */
		float roll_scale = 1;
		float pitch_scale = 1;
		float yaw_scale = 1;
		float deadband = 0;

		_mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
					     MultirotorGeometry::QUAD_X,
					     roll_scale, pitch_scale, yaw_scale, deadband);

		// TODO: temporary hack to make this compile
		(void)_config_index[0];

		if (_mixer == nullptr) {
			PX4_ERR("Mixer initialization failed");
			return -1;
		}

		return 0;
	}
}

int pwm_write_sysfs(char *path, int value)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);
	int n;
	char *data;

	::free(path);

	if (fd == -1) {
		return -errno;
	}

	n = ::asprintf(&data, "%u", value);

	if (n > 0) {
		::write(fd, data, n);
		::free(data);
	}

	::close(fd);

	return 0;
}

int pwm_initialize(const char *device)
{
	int i;
	char *path;

	for (i = 0; i < NUM_PWM; ++i) {
		::asprintf(&path, "%s/export", device);

		if (pwm_write_sysfs(path, i) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::asprintf(&path, "%s/pwm%u/enable", device, i);

		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::asprintf(&path, "%s/pwm%u/period", device, i);

		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::asprintf(&path, "%s/pwm%u/duty_cycle", device, i);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);
		::free(path);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

void pwm_deinitialize()
{
	for (int i = 0; i < NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

void send_outputs_pwm(const uint16_t *pwm)
{
	int n;
	char *data;

	//convert this to duty_cycle in ns
	for (unsigned i = 0; i < NUM_PWM; ++i) {
		n = ::asprintf(&data, "%u", pwm[i] * 1000);
		::write(_pwm_fd[i], data, n);
	}
}



void task_main(int argc, char *argv[])
{
	_is_running = true;

	if (pwm_initialize(_device) < 0) {
		PX4_ERR("Failed to initialize PWM.");
		return;
	}

	// Subscribe for orb topics
	_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	// Start disarmed
	_armed.armed = false;
	_armed.prearmed = false;

	// Set up poll topic
	px4_pollfd_struct_t fds[1];
	fds[0].fd     = _controls_sub;
	fds[0].events = POLLIN;
	/* Don't limit poll intervall for now, 250 Hz should be fine. */
	//orb_set_interval(_controls_sub, 10);

	// Set up mixer
	if (initialize_mixer(MIXER_FILENAME) < 0) {
		PX4_ERR("Mixer initialization failed.");
		return;
	}

	pwm_limit_init(&_pwm_limit);

	// Main loop
	while (!_task_should_exit) {

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);

			_outputs.timestamp = _controls.timestamp;

			/* do mixing */
			_outputs.noutputs = _mixer->mix(_outputs.output,
							0 /* not used */,
							NULL);


			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs;
			     i < sizeof(_outputs.output) / sizeof(_outputs.output[0]);
			     i++) {
				_outputs.output[i] = NAN;
			}

			const uint16_t reverse_mask = 0;
			uint16_t disarmed_pwm[4];
			uint16_t min_pwm[4];
			uint16_t max_pwm[4];

			for (unsigned int i = 0; i < 4; i++) {
				disarmed_pwm[i] = _pwm_disarmed;
				min_pwm[i] = _pwm_min;
				max_pwm[i] = _pwm_max;
			}

			uint16_t pwm[4];

			// TODO FIXME: pre-armed seems broken
			pwm_limit_calc(_armed.armed,
				       false/*_armed.prearmed*/,
				       _outputs.noutputs,
				       reverse_mask,
				       disarmed_pwm,
				       min_pwm,
				       max_pwm,
				       _outputs.output,
				       pwm,
				       &_pwm_limit);

			send_outputs_pwm(pwm);

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
			}
		}

		bool updated;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		}
	}

	pwm_deinitialize();
	orb_unsubscribe(_controls_sub);
	orb_unsubscribe(_armed_sub);

	_is_running = false;

}

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pwm_out_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

}

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: pwm_out start -d /sys/class/pwm/pwmchip0");
	PX4_INFO("       pwm_out stop");
	PX4_INFO("       pwm_out status");
}

} // namespace navio_sysfs_pwm_out

/* driver 'main' command */
extern "C" __EXPORT int navio_sysfs_pwm_out_main(int argc, char *argv[]);

int navio_sysfs_pwm_out_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];

	} else {
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(navio_sysfs_pwm_out::_device, device, strlen(device));
			break;
		}
	}

	// gets the parameters for the esc's pwm
	param_get(param_find("PWM_DISARMED"), &navio_sysfs_pwm_out::_pwm_disarmed);
	param_get(param_find("PWM_MIN"), &navio_sysfs_pwm_out::_pwm_min);
	param_get(param_find("PWM_MAX"), &navio_sysfs_pwm_out::_pwm_max);

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (navio_sysfs_pwm_out::_is_running) {
			PX4_WARN("pwm_out already running");
			return 1;
		}

		navio_sysfs_pwm_out::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!navio_sysfs_pwm_out::_is_running) {
			PX4_WARN("pwm_out is not running");
			return 1;
		}

		navio_sysfs_pwm_out::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("pwm_out is %s", navio_sysfs_pwm_out::_is_running ? "running" : "not running");
		return 0;

	} else {
		navio_sysfs_pwm_out::usage();
		return 1;
	}

	return 0;
}
