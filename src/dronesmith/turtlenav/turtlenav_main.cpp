
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
TurtleNav::task_main_trampoline(int argc, char *argv[])
{
	turtlenav::g_turtlenav->task_main();
}

void
TurtleNav::task_main()
{

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file
	 * else clear geofence data in datamanager */

	/* do subscriptions */


	/* copy all topics first time */

	/* wakeup source(s) */

	/* Setup of loop */


	while (!_task_should_exit) {

		/* wait for up to 200ms for data */

		perf_begin(_loop_perf);

		//bool updated;

		/* gps updated */

		/* sensors combined updated */

		/* parameters updated */

		/* vehicle control mode updated */

		/* vehicle status updated */

		/* vehicle land detected updated */

		/* navigation capabilities updated */

		/* home position updated */

		/* global position updated */

		/* Check geofence violation */

		/* Do stuff according to navigation state set by commander */

		/* iterate through navigation modes and set active/inactive for each */

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
