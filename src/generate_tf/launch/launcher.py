#!/usr/bin/env python

import roslaunch

package = 'rqt_gui'
executable = 'rqt_gui'
node = roslaunch.core.Node(package, executable, "rqt_gui")

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
#print process.is_alive()
#process.stop()
