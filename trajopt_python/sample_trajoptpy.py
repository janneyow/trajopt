#!/usr/bin/env python3

import trajoptpy
import json

request = {
			"basic_info": {
				"n_steps": 3,
				"manip" : "xarm6",
				"start_fixed" : True,
				"max_iter" : 10
			},
			"costs": [
			{
				"type": "joint_vel",
				"params": {
					"targets": [0],
					"coeffs": [1]
				}
			}
            ],
			"init_info": {
				"type": "stationary",
			}
        }
JSON_FILE_NAME = "/home/janne/ros_ws/ferl/src/trajopt/trajopt_ros/config/trajopt_test.json"

if __name__ == "__main__":
	print(trajoptpy.greet())

	s = json.dumps(request)
	prob = trajoptpy.ConstructProblem(s)
	result = trajoptpy.OptimizeProblem(prob)
	print(result.GetTraj())

	print("Finished")
