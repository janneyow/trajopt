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
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

if __name__ == "__main__":
	print(trajoptpy.greet())

	s = json.dumps(request)
	prob = trajoptpy.ConstructProblem(s, [0.94, -0.5, -0.4, 0.82, -0.7, -0.63], joint_names)
	result = trajoptpy.OptimizeProblem(prob)
	print(result.GetTraj())

	print("Finished")
