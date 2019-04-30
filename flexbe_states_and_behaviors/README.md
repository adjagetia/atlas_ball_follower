## Flexbe Portion

# main components
```
custom flexbe states
custom flexbe behaviors
nodes the flexbe behavior will talk to
```

# dependencies
clone them to the catkin ws src directory (or make a flexbe_repos folder)
```
https://github.com/dcat52/flexbe_behavior_engine
https://github.com/dcat52/flexbe_app
```

# running

Start atlas
```
however you start it (start_dock maybe?)
```

lower the head
```
rosrun tough_examples head_control_example 0 30 0
```

Start the node
```
rosrun flexbe_loop_nodes chest_listener_node
```

start flexbe
```
roslaunch flexbe_app flexbe_full.launch
```


open up the flexbe behavior and run it!
```
load behavior --> pub_wait_for_feedback
runtime control --> start execution
```

# NOTES
- add a red ball to gazebo for the atlas to see
