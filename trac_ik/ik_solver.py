# rosrun tf static_transform_publisher -0.3 -0.2 0.5 0 1 0 base_link rnd_tf 100

# 0.3104


from trac_ik_python.trac_ik import IK 
urdfstring = ''.join(open('arm.urdf', 'r').readlines()) 
ik = IK('base_link', 'end', urdf_string=urdfstring)
pi=22/7
x_pos = -0.0
y_pos = -0.0
z_pos =  0.72
angleTuple  = ik.get_ik([0.0]*6, x_pos, y_pos, z_pos, 0, 0, 0, 1)
joint = ['Base Joint       ', 
         'Shoulder Joint   ', 
         'Elbow Joint      ', 
         'Lower Wrist Joint', 
         'Upper Wrist Joint', 
         'Suction          ']
         
print (x_pos**2 + y_pos**2 + z_pos**2)
if angleTuple != None :
	for angle in range(0,6) :
		 print(joint[angle] ," ", str(angleTuple[angle] * (180/pi)) )	 
	print(" ")	 
	
else :
	print(angleTuple)
	
print("rosrun tf static_transform_publisher", x_pos, y_pos, z_pos, "0 1 0 base_link rnd_tf 100")	 
