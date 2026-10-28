# Roadmap and learning plan

Each stage ends with something that runs and something to show in the README.
The "You learn" lines are the ROS 2 concepts to understand before moving on.

## Stage 1: Attacks, rule-based detection, safe stop (done)

* `attack_injector`, `ids_monitor`, `safety_gate`, launch files, unit tests, CI.
* **You learn:** nodes, topics and message types (`Twist` vs `TwistStamped`),
  publishers and subscribers, timers, parameters and YAML files, QoS
  (sensor-data and transient-local "latched" topics), services, launch files,
  remapping, the ROS graph (`ros2 node list`, `ros2 topic info -v`), `colcon`.
* **Try:** run each attack, read the monitor log, then disable one detector in
  `config/monitor.yaml` and see which attacks get through.

## Stage 2: Measure it

* Script that runs each attack N times (plus attack-free runs) and logs
  `/ids/events` with `ros2 bag`.
* Report detection rate, false alarms per hour of normal driving and mean
  time-to-detect, as a table and plot in the README.
* Tune thresholds on the attack-free runs.
* **You learn:** `ros2 bag record/play`, `rosbag2_py` for reading bags in
  Python, `use_sim_time` and the `/clock` topic.

## Stage 3: Learned detector

* Record labelled windows of features (message rates, command/odometry
  residual, scan change statistics) for normal and attack runs.
* Train an MLP and an autoencoder (normal data only) offline; compare with
  the rule-based detectors on the same metrics.
* Run the trained model inside the monitor node.
* **You learn:** turning topics into a dataset, running a model inside a
  node without blocking callbacks, callback groups and executors.

## Stage 4: Autonomous navigation under attack

* Nav2 navigates the TurtleBot3 world through the safety gate.
* Show how each attack affects navigation (collisions, wrong goals) with and
  without the monitor.
* **You learn:** TF2, maps and localisation (AMCL), Nav2 behaviour trees,
  lifecycle nodes, RViz.

## Stage 5: Prevention with SROS2

* Enable SROS2 with keystores and access-control policies so only the safety
  gate may publish `/cmd_vel`.
* Re-run the attacks: which ones are now blocked, and which (for example a
  compromised but authorised node) still need detection?
* **You learn:** DDS security, enclaves, permissions files.

## Ideas for later

* micro-ROS on an STM32 motor controller, with the same checks on real hardware.
* Write the results up as a short workshop paper.
