from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('rviz', True)
    sl.declare_arg('manual', True)

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('ecn_aquabot', 'config.rviz'))

    with sl.group(ns = 'aquabot'):
        # Remappings are to change used to change topic names 
        sl.node('aquabot_ekf', 'gz2ros',
                remappings = {'gps_fix': 'sensors/gps/gps/fix',
                              'imu_raw': 'sensors/imu/imu/data'})
                
        # simple launch python is used to do exactly that
        sl.node('aquabot_motion', 'cmd.py')
        
        sl.node('aquabot_motion', 'planner.py')

        sl.node('robot_localization', 'ekf_node',
                parameters = [sl.find('aquabot_ekf', 'ekf.yaml')],
                remappings = {'odometry/filtered': 'odom'})
        
        sl.node('aquabot_motion', 'follower.py')
        
        

    with sl.group(if_arg='manual'):
        sl.node('slider_publisher',
                arguments = [sl.find('ecn_aquabot', 'props.yaml')])

    return sl.launch_description()
