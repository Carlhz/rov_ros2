import os, sys, time
os.environ['ROS_DOMAIN_ID'] = '42'
sys.path.insert(0, '/opt/ros/lib/python3.8/site-packages')
sys.path.insert(0, '/opt/ros/rov_ros2_ws')

import traceback

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float32, String
    from motor_controller import (
        MotorController, can_init, can_close, send_motor_rpm,
        build_ctrl_200, build_ctrl_201, _clamp, norm_to_rpm,
        allocate, MOTOR_IDS
    )
    
    print("Import OK")
    
    # Test CAN init
    if can_init():
        print("CAN init OK")
    else:
        print("CAN init FAILED")
        sys.exit(1)
    
    # Create a minimal node to test heartbeat_tick
    rclpy.init()
    node = rclpy.create_node('test_motor')
    
    # Create motor controller
    mc = MotorController.__new__(MotorController)
    mc.__init__()
    
    print("MotorController created")
    
    # Set up test state
    mc.depth_valid = True
    mc.current_depth = 0.1
    mc.last_depth_time = time.time()
    mc.ins_att_valid = True
    mc.ins_yaw = 0.0
    mc.ins_pitch = 0.0
    mc.ins_roll = 0.0
    mc.last_att_time = time.time()
    mc.last_dive_flag = 1.0
    mc.target_depth = 0.5
    mc.last_move = 0.0
    mc.last_yaw = 0.0
    mc.initialized = True
    mc._hb_log_count = 0
    
    print("State set up, calling heartbeat_tick...")
    
    # Call heartbeat_tick once
    try:
        mc.heartbeat_tick()
        print("heartbeat_tick() completed successfully!")
    except Exception as e:
        print("heartbeat_tick() FAILED:")
        traceback.print_exc()
    
    # Cleanup
    can_close()
    rclpy.shutdown()
    print("Test completed")
    
except Exception as e:
    print("FATAL ERROR:")
    traceback.print_exc()
