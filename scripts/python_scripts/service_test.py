#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

def set_bool_client(request_value):
    rospy.wait_for_service('setbool')  # Replace with your service name
    try:
        set_bool_service = rospy.ServiceProxy('setbool', SetBool)  # Replace with your service name
        request = SetBoolRequest()
        request.data = request_value  # Set the request value here (True or False)
        response = set_bool_service(request)
        return response.success  # Check if the service call was successful
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('set_bool_client')
    
    # Replace 'your_service_name_here' with the actual service name you want to call
    service_name = 'std_srvs/SetBool'
    
    request_value = True  # Set the request value (True or False) as needed
    
    success = set_bool_client(request_value)
    
    if success:
        print(f'Successfully called {service_name}')
    else:
        print(f'Failed to call {service_name}')
