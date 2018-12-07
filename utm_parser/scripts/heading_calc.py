#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

counter = 0
vector_list = [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
heading_list = [0 , 45, 90, 135, 180, 225, 270, 315]
for head_vec in vector_list:
    length = math.sqrt(head_vec[0]**2+head_vec[1]**2)
    vec_rad = math.atan2(head_vec[1], head_vec[0])
    #vec_rad = math.asin(head_vec[1]/length)
    vec_degree = math.degrees(vec_rad)


    if vec_degree > 0:
        heading = 90 - vec_degree + 360
    else:
        heading = 90 - (180 + 180 + vec_degree) + 360
    if heading >= 360:
        heading -= 360

    print "Head vec", head_vec
    print "lenght: ", length
    print "Rad from sine", vec_rad
    print "Degrees calced from sine: ", vec_degree
    print "Heading calced from vector angle", heading
    print "True heading", heading_list[counter]
    print " "
    counter +=1
