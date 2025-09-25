import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ayasx9/arl_workshop/turtle_proj1/install/project_nodes'
