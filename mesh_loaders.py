import pybullet as p
import pybullet_data
import math
import time
import os
import numpy as np

class Mesh:
    """Load a mesh into the simulation. Mass of the mesh is set to 0 so that it does not fall under gravity."""
    def __init__(
        
        self, path = "./models/Ring/Ring.obj", 
        scale = [0.5, 0.1, 0.1], 
        shift = [0, 0, 0],
        positionXYZ = [0, 0, 1], 
        orientationXYZ = [0, 0, 0],
        alpha = 1.0
        ):
        self.path = path
        self.scale = scale
        self.position = positionXYZ
        self.orientation = orientationXYZ
        self.shift = shift

        self.visualShapeId = p.createVisualShape(
            shapeType=p.GEOM_MESH, 
            fileName=self.path,
            rgbaColor=[1.0, 1.0, 1.0, alpha], 
            specularColor=[0.4, .4, 0],
            meshScale=self.scale, 
            visualFramePosition=self.shift
            )
        
        self.collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=self.path, 
            meshScale=self.scale
            )
        
        self.bodyId = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=self.collisionShapeId,
            baseVisualShapeIndex=self.visualShapeId,
            basePosition=self.position,
            baseOrientation=p.getQuaternionFromEuler(self.orientation)
            )
    
    def getVisualShapeId(self):
        return self.visualShapeId

    def getCollisionShapeId(self):
        return self.collisionShapeId
    
    def getBodyId(self):
        return self.bodyId