
* Starting up the robot

  - Make sure all USB devices are unplugged, in particular the base
    Hokuyo.

  - execute `roslaunch parsec_bringup robot.launch` on your PC.

    This loads a few ROS parameters required by the nodes running on
    the tablet and the robot description, a few bridge nodes
    (odom_relay), a wall clock publisher and the
    robot_state_publisher.

  - Plug in the USB devices one by one and start the corresponding
    apps on the tablet.

  - Check with `rostopic echo /odom` and `rostopic echo /base_scan` if
    the laser and the Arduino came up correctly. Data should be coming
    in.

  - Now it's time to start up navigation. There are two choices,
    gmapping based navigation for exploration and map building and
    amcl based navigation. Execute only one of the two commands.

     - roslaunch parsec_bringup navigation_gmapping.launch

     - roslaunch parsec_bringup navigation_amcl.launch.

    This also starts up waypoint navigation. For more information, see
    Section 'Waypoint navigation'.

* Starting up simulation

  The package `parsec_simulation_stage` contains launch files for
  starting up the simulator with a Willow Garage map. In particular,
  it contains the following launch files:

   - parsec_stage_willow.launch: starts up just stage, no navigation
     or similar things

   - parsec_stage_amcl_nav.launch: starts up stage, a map server, amcl
     plus the nav stack. I.e move_base is running.

   - parsec_stage_gmapping_nav.launch: starts up stage, gmapping and
     the nav stack. To be able to do exploration, two versions of
     move_base are started up. /move_base uses a static map and will
     therefore refuse to drive too far outside of know
     space. /move_base_dynamic doesn't use the static map and
     therefore can be used to drive into unknown space.

* Waypoint navigation

  Waypoint following is provided by the package
  `navigation_waypoints_server`. It provides an action to send a sequence of
  waypoints to the nav stack. In addition in can check if one move
  base can find a valid path by calling its global planner and use a
  fallback move_base if not. The reasoning behind that is that for
  sending the robot into unknown space while doing gmapping, we need a
  configuration of move base that doesn't use a static map.

  In addition to the package `navigation_waypoints_server`, we provide the
  package `interactive_waypoint_markers`.It provides an interface
  between navigation_waypoints_server and rviz using interactive markers. The
  package provides two launch files:

   - interactive_waypoint_markers.launch: starts up
     navigation_waypoints_server and interactive_waypoint_markers configured
     to use one move_base and no fallback.

   - interactive_waypoint_markers_exploration.launch: starts up
     navigation_waypoints_server and interactive_waypoint_markers configured
     to work for exploration, i.e. to work with gmapping and move_base
     by using a move_base fallback.

  Please note: normally it shouldn't be necessary to start any these
  launch files manually since they are already launched when bringing
  up the robot.

  In rviz, add an interactive marker and change the 2D Nav Goal topic
  on the right hand side from /goal or /move_base_simple/goal to
  /interactive_waypoint_markers/add_waypoint.

  Now it should be possible to set waypoints in rviz by using the '2D
  Nav Goal' button on top and clicking in the window. By using the
  'Interact' button, waypoints can be moved, deleted (using the
  context menu) or the interactive_waypoint_markers server can execute
  a circular path, i.e. proceed to the first waypoint when reaching
  the last.

* Base station

  Using the latpop as a base station:

  - Disconnect wifi via NetworkManager

  - Start hostapd: sudo hostapd hostapd.conf

  - Bring up the wlan0 interface: sudo ifconfig wlan0 192.168.0.1 up

  - Start dhcpd: sudo dhcpd -f
