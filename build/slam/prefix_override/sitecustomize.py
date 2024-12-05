import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/imangi/Documents/GitHub/Bharat-Forge-Inter-IIT-24/install/slam'
