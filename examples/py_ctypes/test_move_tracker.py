
import os
import sys
sys.path.append(os.path.abspath('../../bindings/py_ctypes')) # finding and linking to the path

from psmove import *
import numpy as np

count  = PSMove.count_connected()

print "### Found {} controllers.\n".format(count)
print "Trying to init PSMoveTracker...\n"

psmove_tracker = PSMoveTracker()
psmove_tracker.set_mirror(True)

controllers = []; controllers_conn = []
for i in xrange(count):
	controllers.append(PSMove(i))
	controllers[i].connect()

	while True:
		print "Calibrating controller {}...".format(i+1)
		result = psmove_tracker.enable(controllers[i])

		if result != 0:
			break
		else:
			print "Error - retrying \n"

while True:
	psmove_tracker.update_image()
	for i in xrange(count):
		print psmove_tracker.get_location(controllers[i])
		# xcm, ycm, zcm = psmove_tracker.get_location(controllers[i])
		# print xcm, ycm, zcm

[psmove.disconnect(controllers[i]) for i in xrange(count)]
ctrl.tracker.free()

