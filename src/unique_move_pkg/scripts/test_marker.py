#!/usr/bin/env python3
import rospy
from robot_msgs.srv import CheckMarkerId, CheckMarkerIdRequest

def test_marker_service():
    rospy.init_node('marker_service_test_client')
    
    # 等待服务可用
    rospy.wait_for_service('check_marker_id')
    
    try:
        # 创建服务代理
        check_marker = rospy.ServiceProxy('check_marker_id', CheckMarkerId)
        
        # 测试几个不同的marker id
        test_ids = [1, 2, 3, 4, 123]
        
        for marker_id in test_ids:
            try:
                # 发送请求
                request = CheckMarkerIdRequest()
                request.marker_id = marker_id
                response = check_marker(request)
                
                print(f"Marker ID {marker_id}: {'Found' if response.found else 'Not Found'}")
                
            except rospy.ServiceException as e:
                print(f"Service call failed for marker {marker_id}: {e}")
                
    except rospy.ServiceException as e:
        print(f"Service not available: {e}")

if __name__ == "__main__":
    test_marker_service()
