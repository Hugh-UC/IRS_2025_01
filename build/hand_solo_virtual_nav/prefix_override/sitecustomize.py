import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/irslab_ws/install/hand_solo_virtual_nav'
