import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rachit-linux/cart_pole_ws/install/cart_pole_bringup'
