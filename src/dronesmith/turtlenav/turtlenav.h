#ifndef TURTLENAV_H_
#define TURTLENAV_H_

#include <systemlib/perf_counter.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>


class TurtleNav
{
public:
	/**
	 * Constructor
	 */
	TurtleNav();

	/**
	 * Destructor, also kills the turtle task.
	 */
	~TurtleNav();

	/**
	 * Start the turtle task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the turtle status.
	 */
	void		status();

	/**
	 * Setters
	 */

	/**
	 * Getters
	 */

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_turtlenav_task;		/**< task handle for sensor task */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void		task_main();

  /**
	 * this class has ptr data members, so it should not be copied,
	 * consequently the copy constructors are private.
	 */
	TurtleNav(const TurtleNav&);
	TurtleNav operator=(const TurtleNav&);
};

#endif /* TURTLENAV_H_ */
