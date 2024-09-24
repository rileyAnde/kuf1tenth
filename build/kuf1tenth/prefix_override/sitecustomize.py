import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/r478a194/sim_ws/src/kuf1tenth/install/kuf1tenth'
