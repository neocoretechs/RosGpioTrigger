ROSGpioTrigger:

This subsystem consists of Pi4J/WiringPi GPIO event trigger code trapping pin change events from RasberryPi and multiplexing them onto the ROS bus.
The class reads from a remapped command line property __alertfile and when a pin change occurs, the contents of that file are sent
via ROS DiagnosticMessage published on the robocore/status topic. Functionally, a pin change event listener is triggered and places a message
on a waiting blocking queue which is picked up by the ROS publishing loop via the blocking 'take' on the queue.
