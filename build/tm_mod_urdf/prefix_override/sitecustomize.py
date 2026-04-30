import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hucenrotia/mahfud/Calibration_TM/install/tm_mod_urdf'
