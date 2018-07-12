#!/usr/bin/env python

# import openravepy
from hand_feature_generation import *
import numpy as np
import ast
from math import sqrt
from stl_generator_1 import *
import pdb
'''
1. without converting to local frame , find points that are closest to chosen robot hand point
	- Closest to x and y, then z
2. with coverting to global frame, find points that are smallest in x and y, then find z
3. Increment along z direction
** see which one is more accurate
'''

# Things to improve:
# Rework on grid generation, finer and smaller is better
# ...or collect every points on the hand 

# Generate grasp metric by implementing coordinate system on each finger
# x is horizontal axis, y is vertical axis, z is axis normal away

# Identify points that closest to the xz plane
# Find the shortest distance on the mesh from the chosen grid point
def closest_pt_in_XZplane(robot_vert, object_verts):
	z_dist = []
	temp = []
	# print 
	for i in range(len(object_verts)):
		print object_verts[i][1]
		# print abs(robot_vert[1] - object_verts[i][1])
		if abs(robot_vert[1] - object_verts[i][1]) < 0.01:
			temp.append(object_verts[i])
			# z_dist.append(abs(robot_vert[2] - object_verts[i][2]))
			z_dist.append(get_Distance(robot_vert, object_verts[i]))

	min_z_dist = z_dist.index(min(z_dist))
	min_z_obj_vert = temp[min_z_dist]
	print min(z_dist), min_z_obj_vert
	return min_z_obj_vert 

# def overallminZDistance(robot_verts, object_verts):
# 	for i in range

# Increment pt in z direction, find the min distance
# This works now, but still need to justify where to stop at the increment (SOLVED)
def closest_pt_in_Z_direction(robot_vert, object_verts, extent, lower_bound, upper_bound):
	arr = []
	print "robot vertice : ", robot_vert
	new_robot_vert = robot_vert
	# new_robot_vert[1] = 0
	while True:
		temp = []		
		new_robot_vert[2] += 0.001 # increment can be changed
		index1 = extent[1] * (new_robot_vert[0] - lower_bound[0])/(upper_bound[0] - lower_bound[0])
		index2 = extent[3] * (new_robot_vert[1] - lower_bound[1])/(upper_bound[1] - lower_bound[1])
		index3 = extent[5] * (new_robot_vert[2] - lower_bound[2])/(upper_bound[2] - lower_bound[2])

		if index1 > extent[1]:
			index1 = extent[1]
		if index1 < 0:
			index1 = 0
		if index2 > extent[3]:
			index2 = extent[3]
		if index2 < 0:
			index2 = 0
		if index3 > extent[5]:
			index3 = extent[5]
		if index3 < 0:
			index3 = 0

		signed_distance_function_distance = field[int(index3),int(index2),int(index1)]

		if signed_distance_function_distance < 0.0:
			for i in xrange(len(object_verts)):
				temp.append(get_Distance(new_robot_vert, object_verts[i]))				
			min_dist = temp.index(min(temp))
			return object_verts[min_dist], min(temp)
			break

# Takes in hand grids and run metrics
def Z_dir_metrics_Gen(robot_verts, object_verts, extent, lower_bound, upper_bound):
	arr = []
	for i in xrange(len(robot_verts)):
		# run closest_pt_in_Z_direction
		object_verts_min_dist, dist = closest_pt_in_Z_direction(robot_verts[i], object_verts, extent, lower_bound, upper_bound)
		temp = [object_verts_min_dist, dist]
		arr.append(temp)
	closest_contact = []
	temp = []
	
	# Closest pt on the hand grids
	for i in range(len(arr)):
		temp.append(arr[i][1])
		# print arr[i][1]

	index = temp.index(min(temp))
	# index_closest = arr.index(min(arr[:][1]))
	closest_contact = arr[index][0]
	closest_dist = min(temp)

	# Average distance
	# pdb.set_trace()
	avg_dist = sum(temp) / len(temp)

	return arr, closest_contact, closest_dist, avg_dist




# Transform and localize every point in the mesh and find the pt with shortest distance in the xz plane
def getPts_with_minZDistance2(robot_vert, object_verts):
	arr = []
	z_dist = []
	# y is away from robot moving up (vertical axis)
	# really is to find whichever points that lie on x-z plane
	for i in range(len(object_verts)):
		if abs(object_verts[i][1]) < 0.005:
			arr.append(object_verts[i])
			z_dist.append(get_Distance(robot_vert, object_verts[i]))

	min_z_index = z_dist.index(min(z_dist))
	min_z_obj_vert = arr[min_z_index]
	print min(z_dist), min_z_obj_vert
	return min_z_obj_vert

def get_Distance(pt1, pt2):
	return sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2 + (pt1[2] - pt2[2])**2)

# Scale down and tranform object points into palm local coordinate systems
def scaleandtransform_pts(object_verts, Tlocal):
	# print Tlocal
	for i in xrange(len(object_verts)):
		# object_verts[i] = list(transformPoint(np.linalg.inv(Tlocal), object_verts[i])[0:3])
		object_verts[i] = list(transformPoint(Tlocal, object_verts[i])[0:3])
	return object_verts

def TransformationMatrix(robot_vert):
	arr = [[1,0,0,robot_vert[0]], [0, 1, 0, robot_vert[1]], [0, 0, 1, robot_vert[2]], [0,0,0,1]]
	return np.array(arr)


if __name__ == '__main__':
	env = openravepy.Environment()
	robot = loadRobot(env)
	obj = loadObject(env, 'SprayBottle')
	# Reposition object to be in front of the hand
	object_move = [[1,0,0,-0.05], [0,1,0,-0.05], [0,0,1,0.1], [0,0,0,1]]
	object_move = np.array(object_move)
	obj.SetTransform(object_move)
	# stl_gen = stl_Gen(obj)
	# stl_gen.generateSTL('NewSprayBottle.STL')
	'''
	# orig, extents = BoundingBox_param(obj)
	# # face 1,2 (+y, -y)
	# f1_1, f1_2, f2_1, f2_2 = draw_BoundingBox(orig, extents, 'y')
	# # face 3,4 (+z, -z)
	# f3_1, f3_2, f4_1, f4_2 = draw_BoundingBox(orig, extents, 'z')
	# # face 5, 6 (+x, -x)
	# f5_1, f5_2, f6_1, f6_2 = draw_BoundingBox(orig, extents, 'x')

	# # print 'f3_1', f3_1
	# # print 'f3_2', f3_2
	# hand_pts = getGridOnHand(robot, obj, )
	'''

	## Palm
	robot_hand_verts = getRobotVerts(robot)
	# print robot_hand_verts
	hand_points = getManuallyLabelledPoints()
	vert_pt = []
	vert_pt.append(ast.literal_eval(hand_points['handbase']))
	# print vert_pt
	sf_norms, centerRet = getBarryPoints(robot_hand_verts, vert_pt)
	points_in_hand_plane = getGridOnHand(robot, ['handbase'], centerRet, sf_norms)
	pts_and_normal = points_in_hand_plane[0][:]

	pts_on_plane = []
	normals_on_plane = []
	for j in xrange(len(pts_and_normal)):
		pts_on_plane.append(pts_and_normal[j][0])
		normals_on_plane.append(pts_and_normal[j][1])

	# rand_handpts = pts_on_plane[0]
	# pdb.set_trace()
	object_verts = getObjectVerts('SprayBottle.out')
	object_verts = scaleandtransform_pts(object_verts, object_move)
	# rand_handpts_trans = TransformationMatrix(rand_handpts)
	env.SetViewer('qtcoin')

	# 1. Simple method - find the closest point
	# min_z_obj_vert = closest_pt_in_XZplane(rand_handpts, object_verts)
	# print min_z_obj_vert

	# 2. Localize mesh points
	# object_verts = scaleandtransform_pts(object_verts, rand_handpts_trans)
	# min_z_obj_vert2 = getPts_with_minZDistance2(rand_handpts, object_verts)	
	# env.SetViewer('qtcoin')

	# 3. Find point along z direction
	# object_vert_closest, dist = closest_pt_in_Z_direction(rand_handpts, object_verts)
	# arr, closest_contact, closest_dist, avg_dist = Z_dir_metrics_Gen(pts_on_plane, object_verts)

	bounding_item = bounding_box(obj)
	field, bounds, extent, spacing = processVTI('SprayBottle')
	lower_bound, upper_bound, offset = centerItem(robot, obj, bounds, bounding_item)
	for i in xrange(len(pts_on_plane)):
		pts_on_plane[i][0] -= offset[0]
		pts_on_plane[i][1] -= offset[1]
		pts_on_plane[i][2] -= offset[2]

	rand_handpts = pts_on_plane[0]
	# object_vert_closest, dist = closest_pt_in_Z_direction(rand_handpts, object_verts, extent, lower_bound, upper_bound)
	# print pts_on_plane
	arr, closest_contact, closest_dist, avg_dist = Z_dir_metrics_Gen(pts_on_plane, object_verts, extent, lower_bound, upper_bound)
	print arr

