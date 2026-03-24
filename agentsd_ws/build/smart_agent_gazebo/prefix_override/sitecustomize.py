import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/test/wf3/gazebo/agentsd_ws/install/smart_agent_gazebo'
