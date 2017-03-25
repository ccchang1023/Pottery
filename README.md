# Real-time operation of a mesh throwing pottery model, Developed by Chia-Chuan Chang, Su-Yu-Siang, Chiu-Yen-Chun, Sun-Hsin-Pei

We presented a system which can simply operate the 3D-model of throwing by fingers.The main concept is to establish a mesh model of
throwing first, and then with the usage of augmented reality, the 3D-model will be shown to a particular area. In order to use augmented
reality, we need to put on a special goggles, which is a monitor itself, and the model will appear at the area where we marked. As to 
the sensor, we choose leap motion which was just available recently. When users make a gesture which was formulated by us, 
the mesh model will chance in a reasonable way in order to simulate the effect of a real throwing pottery. 
The loading of mesh and those basic operations are mostly done by glut-related library in c++, as to transformation, 
we use the physics engine ‘bullet’to help us rationalize it. Our final target is to create a simulation system of throwing pottery 
which has a extremely high degree of verisimilitude.
