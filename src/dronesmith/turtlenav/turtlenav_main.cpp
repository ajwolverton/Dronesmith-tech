
/**
 * Turtle Navigator
 *
 * Copyright 2016 Dronesmith Technologies, all rights reserved.
 *
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <fcntl.h>
 #include <errno.h>
 #include <math.h>
 #include <poll.h>
 #include <time.h>
 #include <sys/ioctl.h>
 #include <sys/types.h>
 #include <sys/stat.h>

 #include <drivers/device/device.h>
 #include <drivers/drv_hrt.h>
 #include <arch/board/board.h>

 #include <uORB/uORB.h>
 #include <uORB/topics/home_position.h>
 #include <uORB/topics/vehicle_status.h>
 #include <uORB/topics/mission.h>
 #include <uORB/topics/fence.h>
 #include <uORB/topics/fw_pos_ctrl_status.h>
 #include <uORB/topics/vehicle_command.h>
 #include <drivers/drv_baro.h>

 #include <systemlib/err.h>
 #include <systemlib/systemlib.h>
 #include <geo/geo.h>
 #include <dataman/dataman.h>
 #include <mathlib/mathlib.h>
 #include <systemlib/mavlink_log.h>

 #include "turtlenav.h"

extern "C" __EXPORT int turtlenav_main(int argc, char *argv[]);

namespace turtlenav
{

TurtleNav	*g_turtlenav;
}

TurtleNav::TurtleNav() :
  _task_should_exit(false),
  _turtlenav_task(-1)
{

}

TurtleNav::~TurtleNav()
{
  if (_turtlenav_task != -1) {

    /* task wakes up every 100ms or so at the longest */
    _task_should_exit = true;

    /* wait for a second for the task to quit at our request */
    unsigned i = 0;

    do {
      /* wait 20ms */
      usleep(20000);

      /* if we have given up, kill it */
      if (++i > 50) {
        px4_task_delete(_turtlenav_task);
        break;
      }
    } while (_turtlenav_task != -1);
  }

  turtlenav::g_turtlenav = nullptr;
}

void
TurtleNav::local_position_update()
{
	orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
}

void
TurtleNav::vehicle_status_update()
{
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		/* in case the commander is not running */
		_vstatus.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
	}
}

void
TurtleNav::vehicle_land_detected_update()
{
	orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &_land_detected);
}

void
TurtleNav::vehicle_control_mode_update()
{
	if (orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode) != OK) {
		/* in case the commander is not running */
		_control_mode.flag_control_auto_enabled = false;
		_control_mode.flag_armed = false;
	}
}

void
TurtleNav::params_update()
{
	parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), _param_update_sub, &param_update);
}

void
TurtleNav::sensor_combined_update()
{
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
}

void
TurtleNav::home_position_update(bool force)
{
	bool updated = false;
	orb_check(_home_pos_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

void
TurtleNav::task_main_trampoline(int argc, char *argv[])
{
	turtlenav::g_turtlenav->task_main();
}

void
TurtleNav::task_main()
{

	/* do subscriptions */
  _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

  // XXX missions?
  //_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
  //_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));

	/* copy all topics first time */
  vehicle_status_update();
  vehicle_land_detected_update();
  vehicle_control_mode_update();
  local_position_update();
  sensor_combined_update();
  home_position_update(true);
  params_update();

	/* wakeup source(s) */
  px4_pollfd_struct_t fds[2] = {};

	/* Setup of loop */
  fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_command_sub;
	fds[1].events = POLLIN;

  bool local_pos_available_once = false;

	while (!_task_should_exit) {

		/* wait for up to 200ms for data */
    int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

    if (pret == 0) {
      /* timed out - periodic check for _task_should_exit, etc. */
      if (local_pos_available_once) {
        local_pos_available_once = false;
        PX4_WARN("turtle: local position timeout");
      }
      /* Let the loop run anyway, don't do `continue` here. */

    } else if (pret < 0) {
      /* this is undesirable but not much we can do - might want to flag unhappy status */
      PX4_WARN("turtle: poll error %d, %d", pret, errno);
      continue;
    } else {
      /* success, global pos was available */
      local_pos_available_once = true;
    }

		perf_begin(_loop_perf);

		bool updated;

		/* sensors combined updated */
    orb_check(_sensor_combined_sub, &updated);
		if (updated) {
			sensor_combined_update();
		}

		/* parameters updated */
    orb_check(_param_update_sub, &updated);
		if (updated) {
			params_update();
			//updateParams();
		}

		/* vehicle control mode updated */
    orb_check(_control_mode_sub, &updated);
		if (updated) {
			vehicle_control_mode_update();
		}

		/* vehicle status updated */
    orb_check(_vstatus_sub, &updated);
		if (updated) {
			vehicle_status_update();
		}

		/* vehicle land detected updated */
    orb_check(_land_detected_sub, &updated);
		if (updated) {
			vehicle_land_detected_update();
		}

		/* home position updated */
    orb_check(_home_pos_sub, &updated);
		if (updated) {
			home_position_update();
		}

    orb_check(_vehicle_command_sub, &updated);
		if (updated) {
			vehicle_command_s cmd;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {
        warnx("VEHICLE_CMD_DO_REPOSITION -> Move drone in NED_REL coordinate frame.");

				// store current position as previous position and goal as next

				// Go on and check which changes had been requested

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
        warnx("VEHICLE_CMD_NAV_TAKEOFF -> Do an indoor takeoff");

				// store current position as previous position and goal as next

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE) {
				warnx("VEHICLE_CMD_DO_PAUSE_CONTINUE -> hold position");
			}
		}

		/* local position updated */
    if (fds[0].revents & POLLIN) {
			local_position_update();
		}

		/* Do stuff according to navigation state set by commander */
    switch (_vstatus.nav_state) {
			case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			case vehicle_status_s::NAVIGATION_STATE_ACRO:
			case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
        warnx("Nav state not valid for auto control. Turtle not doing anything.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
				warnx("Nav state mission. Turtle moves to a NED point.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
				warnx("Nav state loiter. Turtle just hangs out.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
				warnx("Nav state RC recover. Turtle does something?");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
				warnx("Nav state auto RTL. Turtle flies to takeoff point.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
				warnx("Nav state takeoff. Turtle takes off to 1 or 1.5 meters.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
				warnx("Nav state land. Turtle descends to ground.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_DESCEND:
				warnx("Nav state descend. Turtle perpetually descends?");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
				/* Use complex data link loss mode only when enabled via param
				* otherwise use rtl */
				warnx("Nav state auto RTGS. Will probably ignore this.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
				warnx("Nav state auto landing fail. Will probably ignore this.");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
				warnx("Nav state auto landing gps fail. We don't even use GPS, haha!");
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
				warnx("Nav state auto RTGS. We won't be implementing this.");
				break;
			default:
				//_navigation_mode = nullptr;
				//_can_loiter_at_sp = false;
				break;
		}

		/* if nothing is running, set position setpoint triplet invalid once */

		perf_end(_loop_perf);
	}
	warnx("exiting.");

	_turtlenav_task = -1;
	return;
}

int
TurtleNav::start()
{
	ASSERT(_turtlenav_task == -1);

	/* start the task */
	_turtlenav_task = px4_task_spawn_cmd("turtle",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT + 5,
					 1300,
					 (px4_main_t)&TurtleNav::task_main_trampoline,
					 nullptr);

	if (_turtlenav_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
TurtleNav::status()
{
  if (_turtlenav_task < 0) {
	   warnx("Turtle not running.");
  } else {
    warnx("Turtle is running.");
  }
}

static void
usage()
{
	warnx("usage: turtlenav {start|stop|status}");
}

int
turtlenav_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (turtlenav::g_turtlenav != nullptr) {
			warnx("already running");
			return 1;
		}

		turtlenav::g_turtlenav = new TurtleNav;

		if (turtlenav::g_turtlenav == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != turtlenav::g_turtlenav->start()) {
			delete turtlenav::g_turtlenav;
			turtlenav::g_turtlenav = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (turtlenav::g_turtlenav == nullptr) {
		warnx("not running");
		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		delete turtlenav::g_turtlenav;
		turtlenav::g_turtlenav = nullptr;
	} else if (!strcmp(argv[1], "status")) {
		turtlenav::g_turtlenav->status();
	} else {
		usage();
		return 1;
	}

	return 0;
}
