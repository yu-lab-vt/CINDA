import ctypes

def data_process(detection_arcs, transition_arcs):
	mtail = []
	mhead = []
	mlow = []
	macap = []
	mcost = []

	n_detection = len(detection_arcs)
	n_transition = len(transition_arcs)
	n_traj = n_detection * 3 + n_transition

	mlow = [0] * n_traj
	macap = [1] * n_traj

	# construct entry arc in detection_arcs
	mtail.extend([1] * n_detection)
	mhead.extend(detection_arcs[:, 0] * 2)
	mcost.extend(detection_arcs[:, 1])

	# construct existing arc in detection_arcs
	mtail.extend(detection_arcs[:, 0] * 2 + 1)
	mhead.extend([1] * n_detection)
	mcost.extend(detection_arcs[:, 2])

	# construct detection arc in detection_arcs
	mtail.extend(detection_arcs[:, 0] * 2)
	mhead.extend(detection_arcs[:, 0] * 2 + 1)
	mcost.extend(detection_arcs[:, 3])

	# construct transition arc
	mtail.extend(transition_arcs[:, 0] * 2 + 1)
	mhead.extend(transition_arcs[:, 1] * 2)
	mcost.extend(transition_arcs[:, 2])
	
	msz = [12, 2 * n_detection + 1, len(mtail)]
	return mtail, mhead, mlow, macap, mcost, msz

def mcc4mot(detection_arcs, transition_arcs):
# The min-cost circulation formulation of MAP solver for multi-object
# tracking.
# INPUT: 
# Assuming we have n detections in the video, then
# detection_arcs: a n x 4 matrix, each row corresponds to a detection in the
# form of [detection_id, C_i, C_i^en, C_i^ex];
# transition_arcs: a m x 3 matrix, each row corresponds to a transition arc
# in the form of [detection_id_i, detection_id_j, C_i,j]
# NOTE that the id should be unique and in the range of 1 to n. Detailed 
# defintion can be found in section 3 of the reference:

# OUTPUT:
# traj: cells containing the linking results; each cell contains a
# set of ordered detection ids, which indicate a trajectory
# cost: costs of these trajectories

	_cs2 = ctypes.CDLL('./libcs2.so')
	_cs2.pyCS2.argtypes = (ctypes.POINTER(ctypes.c_long), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double))
	_cs2.pyCS2.restype = ctypes.POINTER(ctypes.c_longlong)


	mtail, mhead, mlow, macap, mcost, msz = data_process(detection_arcs, transition_arcs)
	it_flag = False
	if isinstance(mcost[0], float):
		mcost = [int(n * 10**7) for n in mcost]
		it_flag = True
    

	inf_type = ctypes.c_long * msz[0]
	a_type = ctypes.c_double * msz[2]

	track_vec = _cs2.pyCS2(inf_type(*msz), a_type(*mtail), a_type(*mhead), a_type(*mlow), a_type(*macap), a_type(*mcost))

	cost = []
	traj = []
	sub_traj = []
#     print trac_vec[0]
	# print(track_vec[10])
	for i in range(1, track_vec[0]+1):
		# print(i, track_vec[i])
		if  track_vec[i] > 0:
			sub_traj.append(track_vec[i])
		else:
			cost.append(track_vec[i])
			# print(cost)
			new = [int(x/2) for x in sub_traj[::2]]
			traj.append(new)
			sub_traj = []
    
	if it_flag:
		cost = [(float(n) / 10**7) for n in cost]
        
	return traj, cost
