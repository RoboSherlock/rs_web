# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""

from __future__ import division
from __future__ import print_function  # In python 2.7

from bson.objectid import ObjectId

import sys

from pymongo import MongoClient
import cv2
import numpy as np
import base64
import time


class RSMongoClient(object):

    def __init__(self, db_name):
        """

        :rtype: object
        """
        self.client = MongoClient()
        self.db = self.client[db_name]

    def get_object_image(self, obj_entry, ts):
        # start_time =time.time()
        x = obj_entry['rois']['roi_hires']['pos']['x']
        y = obj_entry['rois']['roi_hires']['pos']['y']
        objheight = obj_entry['rois']['roi_hires']['size']['height']
        objwidth = obj_entry['rois']['roi_hires']['size']['width']
        img_id = self.db.cas.find({'_timestamp': ts})[0]['color_image_hd']
        colorCursor = self.db.color_image_hd.find({'_id': img_id})
        if colorCursor.count() != 0:
            width = colorCursor[0]['cols']
            height = colorCursor[0]['rows']
            imgData = colorCursor[0]['data']
            image = np.reshape(np.fromstring(imgData, np.uint8), (height, width, 3))
            objImg = image[y:y + objheight, x:x + objwidth]
            small = cv2.resize(objImg, (0, 0), fx=100 / objheight, fy=100 / objheight)
            height, width = small.shape[:2]
            if width > 150:
                small = cv2.resize(objImg, (0, 0), fx=150 / objwidth, fy=150 / objwidth)
            # print("getting objHyps image: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
            return small

    def get_persistent_object_annotations(self, obj_entry):
        # start_time =time.time()
        annotations = obj_entry['annotations']
        ann = []
        for a in annotations:
            if a['_type'] != 'rs.annotation.MLNAtoms' and \
                            a['_type'] != 'rs.annotation.Segment' and \
                            a['_type'] != 'rs.annotation.Features':
                ann.append(self.adjust_annotation(a))
        # print("getting annotations for obj:  %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        return ann

    # adjust values in annotation for simpler js vis
    def adjust_annotation(self, a):
        if a['_type'] == 'rs.annotation.ColorHistogram':
            b = a
            width = a['hist']['cols']
            height = a['hist']['rows']
            imgData = a['hist']['data']

            b['values'] = np.fromstring(imgData, np.float32)
            b['bins'] = height * width
            return b
        elif a['_type'] == 'rs.pcl.PclFeature':
            b = a
            b['values'] = np.fromstring(a['feature'], np.float32)
            b['bins'] = b['values'].size
            return b
        else:
            return a

    def get_persistent_objects(self):
        poCursor = self.db.persistent_objects.find()
        objects = []
        for objEntry in poCursor:
            obj = {}
            obj['image'] = self.getBase64Img(self.get_object_image(objEntry, objEntry['lastSeen']))
            obj['annotations'] = self.get_persistent_object_annotations(objEntry)
            objects.append(obj)
        return objects

    def getObjectInstances(self, id):
        nrOfObjs = self.db.persistent_objects.count()
        if id > nrOfObjs:
            return
        poCursor = self.db.persistent_objects.find()
        clusterIDs = poCursor[id]['clusters']
        clusters = []
        for c in clusterIDs:
            document = self.db.scene.find({'identifiables._id': ObjectId(c)},
                                          {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})
            if document.count() != 0:
                cluster = {}
                ts = document[0]['timestamp']
                cluster['image'] = self.getBase64Img(self.get_object_image(document[0]['identifiables'][0], ts))
                cluster['annotations'] = self.get_persistent_object_annotations(document[0]['identifiables'][0])
                clusters.append(cluster)
        return clusters

    def getObjectHypsForScene(self, ts):
        # start_time = time.time()
        sceneDoc = self.db.scene.find({'timestamp': ts})
        # print("finding TS took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        objHyps = []
        if sceneDoc.count() != 0:
            hyps = sceneDoc[0]['identifiables']
            for hyp in hyps:
                objHyp = {}
                objHyp['image'] = self.getBase64Img(self.get_object_image(hyp, ts))
                objHyp['annotations'] = self.get_persistent_object_annotations(hyp)
                objHyps.append(objHyp)
        # print("getting objHyps for scene took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        return objHyps

    def getBase64Img(self, img):
        [ret, png] = cv2.imencode('.png', img)
        b64 = base64.b64encode(png.tostring())
        return 'data:image/png;base64,' + b64

    def getTimestamps(self):
        scene_cursor = self.db.cas.find()
        timestamps = []
        for sc in scene_cursor:
            timestamps.append(sc['_timestamp'])
        return timestamps

    def get_image_for_sceneID(self, sceneId, scaleFactor):
        casDocument = self.db.cas.find({'_id': sceneId})
        if casDocument.count() != 0:
            colorCursor = self.db.color_image_hd.find({'_id': casDocument[0]['color_image_hd']})
        if colorCursor.count() != 0:
            width = colorCursor[0]['cols']
            height = colorCursor[0]['rows']
            imgData = colorCursor[0]['data']
            image = np.reshape(np.fromstring(imgData, np.uint8), (height, width, 3))
            small = cv2.resize(image, (0, 0), fx=scaleFactor, fy=scaleFactor)
            return small

    def getSceneImages(self, timestamps):
        imgs = []
        for ts in timestamps:
            imgs.append(self.getBase64Img(
                self.get_image_for_sceneID(self.db.scene.find({'timestamp': ts})[0]['_parent'], 0.22)))
        return imgs

    def get_scene_image(self, ts):
        # start_time = time.time()
        img = self.get_image_for_sceneID(self.db.scene.find({'timestamp': ts})[0]['_parent'], 0.22)
        # print("getting image took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        # start_time = time.time()
        base64Img = self.getBase64Img(img)
        # print("converting to base64 took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        return base64Img
