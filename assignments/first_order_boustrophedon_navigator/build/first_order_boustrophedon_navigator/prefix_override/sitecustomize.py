import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ruili/ses598-space-robotics-and-ai-2026/assignments/first_order_boustrophedon_navigator/install/first_order_boustrophedon_navigator'
