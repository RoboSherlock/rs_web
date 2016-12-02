# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""

from pymongo import MongoClient
import cv2
import numpy as np
import base64

class RSMongoClient:
  
#    client = MongoClient()
#    db = client["Scenes_annotated"]

    def __init__(self,dbName):
        self.client = MongoClient()
        self.db = self.client[dbName]

    def getPersistentObjects(self):
        poCursor = self.db.persistent_objects.find()
        objImages = []
        for obj in poCursor:
            x=obj['rois']['roi_hires']['pos']['x']
            y=obj['rois']['roi_hires']['pos']['y']
            objheight = obj['rois']['roi_hires']['size']['height']
            objwidth = obj['rois']['roi_hires']['size']['width']
            imgID = self.db.cas.find({'_timestamp':obj['lastSeen']})[0]['color_image_hd']
            colorCursor= self.db.color_image_hd.find({'_id':imgID})
            if colorCursor.count()!=0:
                width = colorCursor[0]['cols']
                height = colorCursor[0]['rows']            
                imgData = colorCursor[0]['data']
                image = np.reshape(np.fromstring(imgData,np.uint8),(height,width,3))
                objImages.append(self.getBase64Img(image[y:y+objheight,x:x+objwidth]))
        return objImages
            
        
    def getBase64Img(self,img):
        [ret,png] = cv2.imencode('.png',img) 
        b64 = base64.b64encode(png.tostring())
        return 'data:image/png;base64,'+b64
    
    def getTimestamps(self):
        scene_cursor = self.db.cas.find()
        timestamps = []
        for sc in scene_cursor:
            timestamps.append(sc['_timestamp'])
        return timestamps

    def get_image_for_sceneID(self,sceneId,scaleFactor):    
      casDocument = self.db.cas.find({'_id':sceneId})
      if casDocument.count() !=0: 
        colorCursor= self.db.color_image_hd.find({'_id':casDocument[0]['color_image_hd']})
        if colorCursor.count()!=0:
          width = colorCursor[0]['cols']
          height = colorCursor[0]['rows']            
          imgData = colorCursor[0]['data']
          image = np.reshape(np.fromstring(imgData,np.uint8),(height,width,3))
          small = cv2.resize(image, (0,0), fx=scaleFactor, fy=scaleFactor) 
          return small


    def getSceneImages(self,timestamps):
      imgs = []
      for ts in timestamps:
        imgs.append(self.getBase64Img(self.get_image_for_sceneID(self.db.scene.find({'timestamp':ts})[0]['_parent'],0.22)))
      return imgs