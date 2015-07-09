#! /usr/local/bin/python

from py_quad_control.vrep_sim.sparc import test_drone_vrep_nav

test_drone_vrep_nav.test_sparc_model(print_stuff=False, trajectory=True,
                                     record_trajectory=False, read_trajectory=True,
                                     control=[True, True, True, True],
                                     recordfiles=[True, True, True, True, True, True],
                                     readfiles=[False, False, False, False, False, False])

# test_sparc_model(print_stuff=False, trajectory=True, record_trajectory=False, read_trajectory=True,
#                  control=[True, True, True, True], readfiles=[True, True, True, True, True, True],
#                  recordfiles=[False, False, False, False, False, False])


