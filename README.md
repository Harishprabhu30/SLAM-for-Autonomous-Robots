# SLAM-for-Autonomous-Robots

# Autonomous Systems:

Autonomous Systems are those systems that are developed to function without the interference of humans and also have the added benefit of working with humans (like HRC - Human-Robot Collaboration). These systems have semantic understanding, which makes them semantically understand static objects and dynamic objects in the environment.

Examples of autonomous systems are Robots deployed in various industries for instance Rovers for space exploration, Kiva - amazon developed robots for warehouses, robots for deliveries, and the most notable and recognized self-driving cars by Tesla and Google.

To achieve a system that works autonomously. Several algorithms and models are working in its embedded system. Here, I am glad to work on one of the algorithms, that is crucial and used widely in these systems.

# SLAM - Simultaneous Localization and Mapping

SLAM is a concept, where an autonomous system is provided with the ability to navigate. At the same time, it can locate itself in the environment where it is placed with little or no human intervention.

# Sensor Selection:

I have a selection of sensors for this project, but they have their disadvantages on using them  in indoor environment.

1. Reflection and shadows of metal surfaces, Glass
2. Bright Lights can cause mis-reading for camera used for detection of objects.
3. Problems with the range of the sensor (1m, 2m).

Here's a list of sensor, which I'm planning to simulate in Isaac Sim to see how it behaves when exposed to reflections, bright lights, range problems.

1. LiDar (range is good, works to some extent with reflections, works in low lighting); (poor for detecting dynamic objects)
2. Camera (detects and segregates moving humans, static objects, walls); (reflection, low lighting, even overly bright lighting)