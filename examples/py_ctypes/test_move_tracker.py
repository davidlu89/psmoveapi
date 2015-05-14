
import os
import sys
sys.path.append(os.path.abspath('../../bindings/py_ctypes')) # finding and linking to the path

from psmove import *
import numpy as np

psmove = PSMove()

count = psmove.count_connected()
print "### Found {} controllers.\n".format(count)
print "Trying to init PSMoveTracker...\n"

psmove.tracker = PSMoveTracker()
print psmove.tracker

psmove.tracker.set_mirror(1)

controllers = []
for i in xrange(count):
	controllers.append(psmove.connect_by_id(i))
	print controllers

	while True:
		print "Calibrating controller {}...".format(i+1)
		result = psmove.tracker.enable(controllers[i])

		if result != 0:
			auto_update_leds = psmove.tracker.get_auto_update_leds(controllers[i])
			# print "OK, auto_update_leds is {} \n".format(auto_update_leds==1)
			break
		else:
			print "Error - retrying \n"

xcm = c_float()
ycm = c_float()
zcm = c_float()
while True:
	psmove.tracker.update_image()
	psmove.tracker.update(0)
	psmove.tracker.annotate()

	for i in xrange(count):

		psmove.tracker.get_location(controllers[i], byref(xcm), byref(ycm), byref(zcm))
		# print xcm, ycm, zcm
		print type(xcm)

[psmove.disconnect(controllers[i]) for i in xrange(count)]
psmove.tracker.free()

