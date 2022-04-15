# C++ Implementaion of RIFT

This is a C++ implementation of TIP 2020 paper ["RIFT: Multi-Modal Image Matching Based on Radiation-Variation Insensitive Feature Transform"](https://ieeexplore.ieee.org/document/8935498/). 
Part of PhaseCongruency is borrowed from [RamilKadyrov: PhaseCongruency](https://github.com/RamilKadyrov/PhaseCongruency).

Specifically, given a pair of images, we first compute RIFT descriptors with phase congruency for each image, and then establish a matching representation with RANSAC for the image pair. 

To run this demo, openCV with c++ is required. After the step of make, just run RIFT_demo.cpp and the result of matching would be displayed. replace 'im1.jpg' and 'im2.jpg' in it with your own images.
You can also get the homography matrix by adding __cv::findHomography()__ in RIFT_demo.cpp. 
