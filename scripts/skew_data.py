#!/usr/bin/env python

import openravepy
from hand_feature_generation import *
import numpy as np
import ast
from math import sqrt
from stl_generator_1 import *
import pdb
from statistics import median
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

# Increment pt in z direction, find the min distance
# This works now, but still need to justify where to stop at the increment (SOLVED)
def closest_pt_in_Z_direction(robot_vert, object_verts, extent, lower_bound, upper_bound, Tlocal, sdf_transform):
	arr = (robot_vert[0], robot_vert[1], robot_vert[2])
	# arr = list(transformPoint(np.linalg.inv(Tlocal), robot_vert)[0:3])
	# pdb.set_trace()
	# print "robot vertice : ", robot_vert
	new_robot_vert = robot_vert[:]
	# new_robot_vert[1] = 0
	# local_robot_vert = list(transformPoint(np.linalg.inv(Tlocal), new_robot_vert[:])[0:3]) # localize	
	while True:
		temp = []
		new_robot_vert[2] += 0.001 # move towards z direction, increment can change
		# world_robot_vert = list(transformPoint(Tlocal, local_robot_vert)[0:3])
		pt_wrt_sdf = transform(new_robot_vert[:],sdf_transform)
		print 'robot vert', new_robot_vert
		# print 'new world vert', world_robot_vert
		print 'pt_sdf', pt_wrt_sdf
		# pdb.set_trace()
		index1 = extent[1] * (pt_wrt_sdf[0] - lower_bound[0])/(upper_bound[0] - lower_bound[0])
		index2 = extent[3] * (pt_wrt_sdf[1] - lower_bound[1])/(upper_bound[1] - lower_bound[1])
		index3 = extent[5] * (pt_wrt_sdf[2] - lower_bound[2])/(upper_bound[2] - lower_bound[2])

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
		# pdb.set_trace()
		signed_distance_function_distance = field[int(index3),int(index2),int(index1)]		
		print 'sdf', signed_distance_function_distance
		if signed_distance_function_distance < 0.0:
			for i in xrange(len(object_verts)):
				temp.append(get_Distance(new_robot_vert, object_verts[i]))				
			min_dist = temp.index(min(temp))
			dist_pt_toMesh = get_Distance(object_verts[min_dist], np.array(arr))
			# pdb.set_trace()
			return object_verts[min_dist], min(temp), dist_pt_toMesh
			break


# Takes in hand grids and run metrics
# Overall function
def Z_dir_metrics_Gen(robot_verts, object_verts, extent, lower_bound, upper_bound, Tlocal, sdf_transform):
	arr = []
	for i in xrange(len(robot_verts)):
		# run closest_pt_in_Z_direction
		object_verts_min_dist, min_dist, dist_pt_toMesh = closest_pt_in_Z_direction(robot_verts[i], object_verts, extent, lower_bound, upper_bound, Tlocal, sdf_transform)
		temp = [object_verts_min_dist, min_dist, dist_pt_toMesh]
		arr.append(temp)
	closest_contact = []
	temp1 = []
	
	# Closest pt on the hand grids
	for i in range(len(arr)):
		temp1.append(arr[i][2]) # taking the distance among these robot vertices

	index = temp1.index(min(temp1))
	closest_contact = arr[index][0]
	closest_dist = min(temp1) # taking the shortest distance among these robot vertices
	# Average distance
	# pdb.set_trace()
	avg_dist = sum(temp1) / len(temp1)

	return arr, closest_contact, closest_dist, avg_dist

# Total 108 sampling points on xz plane
def sampling_sweep_area(robot_vert):
	pts = []
	x = np.arange(-0.1,0.11,0.05)
	nslope = np.arange(-1.0,-120.0, -20.0)
	pslope = np.arange(1.0,120.0,20.0)
	delta_z = 0.1 # increment from robot_vert[2] to robot_vert[2] + 0.1
	# negative slope side
	for i in nslope:
		x_new = ( delta_z / i ) + robot_vert[0]
		x_range = np.arange(robot_vert[0] + -(abs(robot_vert[0] - x_new) / 10), x_new, -(abs(robot_vert[0] - x_new) / 10))
		# pdb.set_trace()
		temp = []
		for j in x_range:
			z_new = i*(j - robot_vert[0]) + robot_vert[2]
			pts.append([j, robot_vert[1], z_new]) # x_new with corresponding z
			# pdb.set_trace()
		# pdb.set_trace()
		# pts.append(temp)
	# Positive slope side
	for k in pslope:
		x_new_p = ( delta_z / k ) + robot_vert[0]
		x_range_p = np.arange(robot_vert[0] + (abs(robot_vert[0] - x_new_p) / 10), x_new_p, (abs(robot_vert[0] - x_new_p) / 10))
		# pdb.set_trace()
		temp = []
		for m in x_range_p:
			z_new_p = k*(m - robot_vert[0]) + robot_vert[2]
			pts.append([m, robot_vert[1],z_new_p]) # x_new with corresponding z
			# pdb.set_trace()
		# pdb.set_trace()
		# pts.append(temp)
	return pts

def sampling_sweep_area_2(robot_vert):
	# Generate points on xz plane using linear combination
	return None

#######################################
# Find object points within the threshold in terms of y coordinates
def threshold_(robot_vert, object_verts, object_norms, obj):
	# localize robot vert
	robot_vert_local = list(localize_vertices(robot_vert, obj)[0:3])
	object_verts_local = []
	object_norms_local = []
	pts_within_world = []
	for i in xrange(len(object_verts)):
		object_vert_local = list(localize_vertices(object_verts[i], obj)[0:3])
		object_verts_local.append(object_vert_local)
	# pdb.set_trace()
	# Now robot vert and object verts are localized, implement threshold
	pts_within = []
	norms_within = []
	for i in xrange(len(object_verts_local)):
		if abs(object_verts_local[i][1] - robot_vert_local[1]) < 0.001:
			pts_within.append(object_verts_local[i])
			norms_within.append(object_norms[i])
			pts_within_world.append(object_verts[i])

	return pts_within, norms_within, robot_vert_local, object_verts_local, pts_within_world

# localize vertices wrt object coordinate system
def localize_vertices(robot_vert, obj):
	TransformationMatrix = obj.GetTransform()
	temp = np.array(robot_vert + [1])
	return np.matmul(np.linalg.inv(TransformationMatrix), temp)

# Generate histogram
def histogram_(robot_vert, object_verts, obj):
	pts_within, robot_vert_local, object_verts_local = threshold_(robot_vert, object_verts, obj)
	dists = []
	for i in pts_within:
		dists.append(get_Distance(robot_vert_local, i))
	dists.sort()
	med = median(dists)
	avg = np.mean(dists)
	min_dist = min(dists)
	max_dist = max(dists)
	return dists, med, avg, min_dist, max_dist
#########################################

# Should run points within y range first (increase computation time)
# alpha is the angle of the wedge you want
def wedge(plane_normal, object_verts, object_norms, alpha, obj):
	# define wedge, (z-0, x-1/3 width of finger)
	# alpha = np.pi / 6 # degree
	threshold_angle = np.pi - alpha
	inv_plane_normal = -plane_normal[:]
	# pdb.set_trace()
	pts_to_include = []
	for i in xrange(len(object_verts)):
		if np.dot(object_norms[i], inv_plane_normal) < 0:
			pts_to_include.append(object_verts[i])
	return pts_to_include

def get_min_distance(robot_vert, object_verts):
	temp = []
	for i in xrange(len(object_verts)):
		temp.append(get_Distance(robot_vert, object_verts[i]))
	min_dist_index = temp.index(min(temp))
	return temp[min_dist_index] # return minimum distance

def distances_XZ_plane(robot_verts, object_verts, sdf_filename, robot, obj):
	# first for each robot vert, get points on xz plane
	pts = []
	bounding_item = bounding_box(obj)
	field, bounds, extent, spacing = processVTI(sdf_filename)
	lower_bound, upper_bound, offset = centerItem(robot, obj, bounds, bounding_item) # Center object to the signed distance field and reposition robot	
	sdf_transform = np.array([[1,0,0,offset[0]], [0,1,0, offset[1]], [0,0,1,offset[2]], [0,0,0,1]])	
	for i in robot_verts:
		xz_plane_pts = sampling_sweep_area(i)
		# for each point in xz plane, check whether is in sdf
		for j in xrange(len(xz_plane_pts)):
			pts_dist = []
			pt_wrt_sdf = transform(xz_plane_pts[j][:], sdf_transform)
			in_obj,signed_distance_function_distance = check_signed_distance_field(extent, pt_wrt_sdf, lower_bound, upper_bound, field)
			if in_obj == 'no':
				pt_dist = get_min_distance(xz_plane_pts[j], object_verts)
				# pdb.set_trace()
				pts_dist.append(pt_dist)
		index_min_dist_xz = pts_dist.index(min(pts_dist))
		dist = get_Distance(xz_plane_pts[index_min_dist_xz], i)
		pts.append([xz_plane_pts[index_min_dist_xz], dist])
	return pts


def check_signed_distance_field(extent, pt_wrt_sdf, lower_bound, upper_bound, field):
	index1 = extent[1] * (pt_wrt_sdf[0] - lower_bound[0])/(upper_bound[0] - lower_bound[0])
	index2 = extent[3] * (pt_wrt_sdf[1] - lower_bound[1])/(upper_bound[1] - lower_bound[1])
	index3 = extent[5] * (pt_wrt_sdf[2] - lower_bound[2])/(upper_bound[2] - lower_bound[2])

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
	# pdb.set_trace()
	signed_distance_function_distance = field[int(index3),int(index2),int(index1)]		
	if signed_distance_function_distance < 0.0:
		in_obj = 'yes'
	else:
		in_obj = 'no'
	return in_obj, signed_distance_function_distance

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

# This function generates arbitrary points on each link of the hand
def robot_hand_point_sampler(Tlocal, boundX, boundY, boundZ, robot_name):	# boundX and boundY must be tuple
	points = []
	# pt = np.array([boundX[0]+(abs(boundX[1] - boundX[0])/2),boundY[0]+(abs(boundY[1] - boundY[0])/2),boundZ[0]+(abs(boundZ[1] - boundZ[0])/2)])
	minWorld = np.array([boundX[0],boundY[0], boundZ[0]])
	maxWorld = np.array([boundX[1], boundY[1], boundZ[1]])
	new_robot_vert_min = list(transformPoint(np.linalg.inv(Tlocal), minWorld)[0:3]) # convert to local 
	new_robot_vert_max = list(transformPoint(np.linalg.inv(Tlocal), maxWorld)[0:3]) # convert to local
	# pt_trans = list(transformPoint(np.linalg.inv(Tlocal), pt)[0:3]) # convert to local
	# pdb.set_trace()
	rangeX = np.arange(new_robot_vert_min[0], new_robot_vert_max[0]+((new_robot_vert_max[0] - new_robot_vert_min[0])/4), ((new_robot_vert_max[0] - new_robot_vert_min[0])/4))
	rangeY = np.arange(new_robot_vert_min[1], new_robot_vert_max[1]+((new_robot_vert_max[1] - new_robot_vert_min[1])/4), ((new_robot_vert_max[1] - new_robot_vert_min[1])/4))
	if robot_name == 'handbase':
		rangeZ = np.ones(5)
		rangeZ = rangeZ * new_robot_vert_min[2]
	else:
		rangeZ = np.arange(new_robot_vert_min[2], new_robot_vert_max[2]+((new_robot_vert_max[2] - new_robot_vert_min[2])/4), ((new_robot_vert_max[2] - new_robot_vert_min[2])/4))
	
	# pdb.set_trace()

		for y in rangeY:
			for x in range(len(rangeX)):
				u = np.array([rangeX[x],0,0])
				v = np.array([0,y,0])
				w = np.array([0,0,rangeZ[x]])
				print w
				arr = u+v+w
				points.append(arr)
	return points # points on local coordinate system

def point_sampler_plane_equation(p1, p2, p3, boundX, boundY):
	u = p1 - p2
	v = p1 - p3
	normal = np.cross(u,v)
	# Plane equation normal[0]*(x-p1[0]) + normal[1]*(y-p1[1]) + normal[2]*(z-p1[2]) = 0
	rangeX = np.arange(boundX[0], boundX[1] + (abs(boundX[1] - boundX[0]) / 4), (abs(boundX[1] - boundX[0]) / 4))
	rangeY = np.arange(boundY[0], boundY[1] + (abs(boundY[1] - boundY[0]) / 4), (abs(boundY[1] - boundY[0]) / 4))
	arr = []
	# pdb.set_trace()
	for y in rangeY:
		for x in rangeX:
			z = (-normal[0]*(x-p1[0]) - normal[1]*(y-p1[1]) + normal[2]*p1[2]) / normal[2]
			pt = [x, y, z]
			arr.append(pt)

	return arr,normal

# For the purpose of trasforming to the sdf frame
def transform(point, TransformationMatrix):
	point[0] -= TransformationMatrix[0][3]
	point[1] -= TransformationMatrix[1][3]
	point[2] -= TransformationMatrix[2][3]
	return point

def getObjectvert_and_norm(filename):
	b = numpy.loadtxt(filename, dtype=float)
	b = list(numpy.reshape(b, (-1,6)))
	return b	

def sort_vert_norm(object_verts_norms):
	object_verts = []
	object_norms = []
	for i in object_verts_norms:
		object_verts.append([i[0], i[1], i[2]])
		object_norms.append([i[3], i[4], i[5]])
	return object_verts, object_norms

if __name__ == '__main__':
	env = openravepy.Environment()
	robot = loadRobot(env)
	obj = loadObject(env, 'SprayBottle')
	# pdb.set_trace()
	# Reposition object to be in front of the hand
	object_move = [[1,0,0,-0.05], [0,1,0,-0.05], [0,0,1,0.1], [0,0,0,1]]
	object_move = np.array(object_move)
	obj.SetTransform(object_move)

	# Get Object Vertices
	object_verts_norms = getObjectvert_and_norm('SprayBottle.out')

	# sort those vertices and norm into two arrays
	object_verts, object_norms = sort_vert_norm(object_verts_norms)
	object_verts = scaleandtransform_pts(object_verts, object_move)
	# object_norms = scaleandtransform_pts(object_norms, object_move)
	# rand_handpts_trans = TransformationMatrix(rand_handpts)

	# Point Sampling Methpoints: Create a plane and sample points on it
	# Finger 2-1
	# Tlocal = robot.GetLink('Finger2-1').GetTransform()
	# hand_bound = [(-0.10599, -0.06983), (-0.00664,0.00664), (0.104, 0.105)]	
	# p1 = np.array([-0.10388, -0.00512, 0.10421])
	# p2 = np.array([-0.08720, 0.00624, 0.10482])
	# p3 = np.array([-0.07039, -0.00015, 0.10564])

	# Handbase 
	# Tlocal = robot.GetLink('handbase').GetTransform()
	# hand_bound = [(-0.02200, 0.02200), (-0.04,0.04), (0.09496, 0.09496)]	
	# p1 = np.array([-0.01142, 0.00282, 0.09496])
	# p2 = np.array([0.00380, 0.01811, 0.09496])
	# p3 = np.array([0.01026, -0.01408, 0.09496])

	# # Finger 2-2
	Tlocal = robot.GetLink('Finger2-2').GetTransform()
	hand_bound = [(-0.14415, -0.12006), (-0.00665,0.00818), (0.10559, 0.13514)]	
	p1 = np.array([-0.13808, 0.00077, 0.12782])
	p2 = np.array([-0.13049, -0.00337, 0.11878])
	p3 = np.array([-0.12533, 0.00669, 0.11263])

	pts_on_plane, plane_normal = point_sampler_plane_equation(p1, p2, p3, hand_bound[0], hand_bound[1])
	env.SetViewer('qtcoin')

	# bounding_item = bounding_box(obj)
	# field, bounds, extent, spacing = processVTI('SprayBottle')
	# lower_bound, upper_bound, offset = centerItem(robot, obj, bounds, bounding_item) # Center object to the signed distance field and reposition robot
	# pdb.set_trace()

	# sdf_transform = np.array([[1,0,0,offset[0]], [0,1,0, offset[1]], [0,0,1,offset[2]], [0,0,0,1]])
	# pts = distances_XZ_plane(pts_on_plane, object_verts, 'SprayBottle', robot, obj)
	# Localize pointse
	# for j in xrange(len(pts_on_plane)):
	# 	pts_on_plane[j] = list(transformPoint(np.linalg.inv(Tlocal), pts_on_plane[j])[0:3])
	# print pts_on_plane
	# pdb.set_trace()
	# pts = sampling_sweep_area(pts_on_plane[0])

	# arr, closest_contact, closest_dist, avg_dist = Z_dir_metrics_Gen(pts_on_plane, object_verts, extent, lower_bound, upper_bound, Tlocal, sdf_transform)
	# print arr

	# hist, med, avg, min_dist, max_dist = histogram_(pts_on_plane[0], object_verts, obj)

	# threshold and wedge to choose points that face the hand only
	pts_within, norms_within, robot_vert_local, object_verts_local, pts_within_world = threshold_(pts_on_plane[0], object_verts, object_norms, obj)
	pts_to_include = wedge(plane_normal, pts_within_world, norms_within, np.pi/6, obj)
