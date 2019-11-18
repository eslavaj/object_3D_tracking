#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 18 11:34:55 2019

@author: jeslava
"""

import subprocess

matcher_type= "MAT_BF" 
match_selector_type = "SEL_NN"
arg_testcases = []
arg_testcases.append(["SHITOMASI", "BRISK"])
arg_testcases.append(["SHITOMASI", "BRIEF"])
arg_testcases.append(["SHITOMASI", "ORB"])
arg_testcases.append(["SHITOMASI", "FREAK"])
arg_testcases.append(["SHITOMASI", "SIFT"])

arg_testcases.append(["HARRIS", "BRISK"])
arg_testcases.append(["HARRIS", "BRIEF"])
arg_testcases.append(["HARRIS", "ORB"])
arg_testcases.append(["HARRIS", "FREAK"])
arg_testcases.append(["HARRIS", "SIFT"])

arg_testcases.append(["FAST", "BRISK"])
arg_testcases.append(["FAST", "BRIEF"])
arg_testcases.append(["FAST", "ORB"])
arg_testcases.append(["FAST", "FREAK"])
arg_testcases.append(["FAST", "SIFT"])

arg_testcases.append(["BRISK", "BRISK"])
arg_testcases.append(["BRISK", "BRIEF"])
arg_testcases.append(["BRISK", "ORB"])
arg_testcases.append(["BRISK", "FREAK"])
arg_testcases.append(["BRISK", "SIFT"])

arg_testcases.append(["ORB", "BRISK"])
arg_testcases.append(["ORB", "BRIEF"])
arg_testcases.append(["ORB", "ORB"])
arg_testcases.append(["ORB", "FREAK"])
arg_testcases.append(["ORB", "SIFT"])

arg_testcases.append(["AKAZE", "BRISK"])
arg_testcases.append(["AKAZE", "BRIEF"])
arg_testcases.append(["AKAZE", "ORB"])
arg_testcases.append(["AKAZE", "FREAK"])
arg_testcases.append(["AKAZE", "AKAZE"])
arg_testcases.append(["AKAZE", "SIFT"])

arg_testcases.append(["SIFT", "BRISK"])
arg_testcases.append(["SIFT", "BRIEF"])
arg_testcases.append(["SIFT", "FREAK"])
arg_testcases.append(["SIFT", "SIFT"])


# Run test cases
for i in range(len(arg_testcases)):
    print("**********************************************  Test case ", i )
    print("**********************************************  ", arg_testcases[i] )
    subprocess.run([ "../build/./3D_object_tracking", arg_testcases[i][0], arg_testcases[i][1], matcher_type, match_selector_type ])
    
    

