import pybullet as p
import pybullet_data
import math
import time
import os
import numpy as np

class Mesh:
    """Load a mesh into the simulation. Mass of the mesh is set to 0 so that it does not fall under gravity."""
    def __init__(
        
        self, path = "./assets/sphere/untitled.obj", 
        scale = [0.5, 0.1, 0.1], 
        shift = [0, 0, 0],
        positionXYZ = [0, 0, 1], 
        rgbColor = [0.0, 1.0, 0.0],
        orientationXYZ = [0, 0, 0],
        alpha = 0.1
        ):
        self.path = path
        self.scale = scale
        self.position = positionXYZ
        self.orientation = orientationXYZ
        self.shift = shift
        self.rgbColor = list(rgbColor)

        self.visualShapeId = p.createVisualShape(
            shapeType=p.GEOM_MESH, 
            fileName=self.path,
            rgbaColor=self.rgbColor+[alpha], 
            specularColor=[0.0, 0.0, 0.0],
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