
import os
import sys
sys.path.append(os.path.abspath('../../bindings/py_ctypes')) # finding and linking to the path

from psmove import *

psmove = PSMove()

count = psmove.count_connected()
print "### Found {} controllers.\n".format(count)
move = psmove.connect()

print "Trying to init PSMoveTracker...\n"

psmove.tracker = PSMoveTracker()
print psmove.tracker

psmove.tracker.set_mirror(1)
# print 'psmove_tracker_get_dimming(): ', psmove.tracker.get_dimming()

while True:
	psmove.tracker.update_image()
	psmove.tracker.update(move)
	psmove.tracker.annotate()

	psmove.tracker.get_location(move, xcm, ycm, zcm)
	print xcm, ycm, zcm

