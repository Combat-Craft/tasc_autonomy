import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mnt/Desktop/tasc_autonav_combined/install/autonomy_sensors'
