# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""
from __future__ import division
from __future__ import print_function # In python 2.7

import sys

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
    
    def get_object_image(self, objEntry):
        x=objEntry['rois']['roi_hires']['pos']['x']
        y=objEntry['rois']['roi_hires']['pos']['y']
        objheight = objEntry['rois']['roi_hires']['size']['height']
        objwidth = objEntry['rois']['roi_hires']['size']['width']
        imgID = self.db.cas.find({'_timestamp':objEntry['lastSeen']})[0]['color_image_hd']
        colorCursor= self.db.color_image_hd.find({'_id':imgID})
        if colorCursor.count()!=0:
            width = colorCursor[0]['cols']
            height = colorCursor[0]['rows']            
            imgData = colorCursor[0]['data']
            image = np.reshape(np.fromstring(imgData,np.uint8),(height,width,3))
            objImg=image[y:y+objheight,x:x+objwidth]
            small = cv2.resize(objImg, (0,0), fx= 100 / objheight,fy=100 / objheight)
            height,width = small.shape[:2]
            if width > 200:
                small = cv2.resize(objImg, (0,0), fx= 200 / objwidth,fy=200 / objwidth)
            return small
    
    def get_object_hyp_image(self, objEntry,ts):
        x=objEntry['rois']['roi_hires']['pos']['x']
        y=objEntry['rois']['roi_hires']['pos']['y']
        objheight = objEntry['rois']['roi_hires']['size']['height']
        objwidth = objEntry['rois']['roi_hires']['size']['width']

        imgID = self.db.cas.find({'_timestamp':ts})[0]['color_image_hd']
        
        colorCursor= self.db.color_image_hd.find({'_id':imgID})
        if colorCursor.count()!=0:
            width = colorCursor[0]['cols']
            height = colorCursor[0]['rows']            
            imgData = colorCursor[0]['data']
            image = np.reshape(np.fromstring(imgData,np.uint8),(height,width,3))
            objImg=image[y:y+objheight,x:x+objwidth]
            small = cv2.resize(objImg, (0,0), fx= 100 / objheight,fy=100 / objheight)
            height,width = small.shape[:2]
            if width > 200:
                small = cv2.resize(objImg, (0,0), fx= 200 / objwidth,fy=200 / objwidth)
            return small
            
    def get_persistent_object_annotations(self,objEntry):
        annotations = objEntry['annotations']
        ann=[]
        for a in annotations:
            if  a['_type'] != 'rs.annotation.MLNAtoms' and \
                a['_type'] != 'rs.annotation.Segment': 
                ann.append(a)
        return ann
#        return objEntry['annotations']
        
    
    def process_annotations(self,annot):
        switcher = {
            'rs.annotation.Shape':self.handle_shape_annotation,
            'rs.annotation.SemanticColor:':self.handle_sem_color_annotation,
            'rs.annotation.SemanticSize':self.hanld_sem_size_annotation
        }
        func = switcher.get(annot,lambda:"nothing")
        return func(annot)
    
    def handle_shape_annotation(self,annot):
        res = {}        
         
        return res
        
    def handle_sem_color_annotation(self,annot):
        res= {}
        return res
    
    def handle_sem_size_annotation(self,annot):
        res ={}
        res['size'] = annot['size']
        res['confidence'] = annot['confidence']
        return res
        
    def getPersistentObjects(self):
        poCursor = self.db.persistent_objects.find()
        objects = []
        for objEntry in poCursor:
            obj = {}
            obj['image'] = self.getBase64Img(self.get_object_image(objEntry))
            obj['annotations'] = self.get_persistent_object_annotations(objEntry)
            objects.append(obj)

        return objects
    
    def getObjectHypsForScene(self,ts):
        sceneDoc = self.db.scene.find({'timestamp':ts})
        objHyps = []        
        if sceneDoc.count()!=0:
            hyps = sceneDoc[0]['identifiables']
            for hyp in hyps:
                 objHyp = {}
                 objHyp['image'] = self.getBase64Img(self.get_object_hyp_image(hyp,ts))
                 objHyp['annotations'] = self.get_persistent_object_annotations(hyp)
                 objHyps.append(objHyp)
        return objHyps
                
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
      
    def getSceneImage(self,ts):
        return self.getBase64Img(self.get_image_for_sceneID(self.db.scene.find({'timestamp':ts})[0]['_parent'],0.22))
        
  