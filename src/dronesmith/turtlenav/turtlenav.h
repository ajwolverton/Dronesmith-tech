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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/sensor_combined.h>

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

	bool	_task_should_exit;		  /**< if true, sensor task should exit */
	int		_turtlenav_task;		    /**< task handle for sensor task */

	perf_counter_t	_loop_perf;		/**< loop performance counter */

  int		_local_pos_sub;		      /**< local position subscription */
	int		_sensor_combined_sub;		/**< sensor combined subscription */
	int		_home_pos_sub;			    /**< home position subscription */
	int		_vstatus_sub;			      /**< vehicle status subscription */
	int		_land_detected_sub;		  /**< vehicle land detected subscription */
	int		_control_mode_sub;	    /**< vehicle control mode subscription */
	int		_param_update_sub;	    /**< param update subscription */
	int		_vehicle_command_sub;	  /**< vehicle commands (onboard and offboard) */

  vehicle_status_s				    _vstatus;		       /**< vehicle status */
	vehicle_land_detected_s		  _land_detected;		 /**< vehicle land_detected */
	vehicle_control_mode_s			_control_mode;	   /**< vehicle control mode */
	vehicle_local_position_s		_local_pos;		     /**< local vehicle position */
	sensor_combined_s				    _sensor_combined;	 /**< sensor values */
	home_position_s					    _home_pos;		     /**< home position for RTL */

	position_setpoint_triplet_s			_pos_sp_triplet;	    /**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet;	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet;	    /**< triplet for non-mission direct takeoff command */


  /**
   * Retrieve local position
   */
  void		local_position_update();
  
  /**
   * Retrieve sensor values
   */
  void		sensor_combined_update();

  /**
   * Retrieve home position
   */
  void		home_position_update(bool force=false);

  /**
   * Retrieve vehicle status
   */
  void		vehicle_status_update();

  /**
   * Retrieve vehicle land detected
   */
  void		vehicle_land_detected_update();

  /**
   * Retrieve vehicle control mode
   */
  void		vehicle_control_mode_update();

  /**
   * Update parameters
   */
  void		params_update();

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
