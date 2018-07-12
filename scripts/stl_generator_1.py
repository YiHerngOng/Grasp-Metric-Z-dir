#!/usr/bin/env python

from stlwriter import Binary_STL_Writer
import numpy as np
from openravepy import *


class stl_Gen():
    def __init__(self, obj):
        self.obj = obj

    def getSTLFeatures(self):
        links = self.obj.GetLinks()
        all_vertices = []
        all_faces = []
        ind = 0
        # I don't know what is happening here
        for link in links:
            vertices = link.GetCollisionData().vertices
            faces = link.GetCollisionData().indices
            if ind == 0:
                faces = np.add(faces,ind)
            else:
                faces = np.add(faces,ind+1)
            try:
                ind = faces[-1][-1]
            except:
                pass
        
            #print "link: ", link, "\nStarting index for this link: ", len(all_vertices)
            link_pose = poseFromMatrix(link.GetTransform())
            transform_vertices = poseTransformPoints(link_pose, vertices)
            all_vertices.extend(transform_vertices.tolist())
            all_faces.extend(faces.tolist())
        self.all_faces = all_faces
        self.all_vertices = all_vertices

    def writeSTL(self, save_filename):
        faces_points = []
        for vec in self.all_faces:
            faces_points.append([self.all_vertices[vec[0]],self.all_vertices[vec[1]],self.all_vertices[vec[2]]])
        
        # pdb.set_trace()
        with open(save_filename,'wb') as fp:
            writer = Binary_STL_Writer(fp)
            writer.add_faces(faces_points)
            writer.close()

    def generateSTL(self, save_filename):
        self.getSTLFeatures()
        self.writeSTL(save_filename)