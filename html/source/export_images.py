# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""


from bson.objectid import ObjectId

from pymongo import MongoClient
import cv2
import numpy as np
import base64
import time


class MongoWrapper():

    def __init__(self,collection_name):
        self.client = MongoClient()
        self.db = self.client[collection_name]
        self.active_collection = self.db.scene

    def get_object_image(self, obj_entry, ts):
        x = obj_entry['rois']['roi_hires']['pos']['x']
        y = obj_entry['rois']['roi_hires']['pos']['y']
        obj_height = obj_entry['rois']['roi_hires']['size']['height']
        obj_width = obj_entry['rois']['roi_hires']['size']['width']
        mask_data = obj_entry['rois']['mask']['data']
        mask_width = obj_entry['rois']['mask']['cols']
        mask_height = obj_entry['rois']['mask']['rows']

        img_id = self.db.cas.find({'_timestamp': ts})[0]['color_image_hd']
        depth_img_id = self.db.cas.find({'_timestamp': ts})[0]['depth_image_hd']

        annotations = obj_entry['annotations']
        gt = ''
        for annotation in annotations:
            if annotation['_type'] == 'rs.annotation.GroundTruth':
                gt = annotation['classificationGT']['classname']
                break

        color_cursor = self.db.color_image_hd.find({'_id': img_id})
        depth_cursor = self.db.depth_image_hd.find({'_id': depth_img_id})

        if color_cursor.count() != 0 and depth_cursor.count() != 0:
            width = color_cursor[0]['cols']
            height = color_cursor[0]['rows']
            img_data = color_cursor[0]['data']
            depth_data = depth_cursor[0]['data']
            image = np.reshape(np.fromstring(img_data, np.uint8), (height, width, 3))
            depth_image = np.reshape(np.fromstring(depth_data, np.uint16), (height, width, 1))
            mask_image = np.reshape(np.fromstring(mask_data, np.uint8),(mask_height, mask_width, 1))
            obj_image = image[y:y + obj_height, x:x + obj_width]
            obj_depth = depth_image[y:y + obj_height, x:x + obj_width]

            small_rgb = cv2.resize(obj_image, (0, 0), fx=0.5, fy=0.5)
            small_depth = cv2.resize(obj_depth, (0, 0), fx=0.5, fy=0.5)

            return {"rgb":small_rgb,"depth":small_depth,"mask":mask_image,"gt":gt}

    def save_object_hypotheses_for_scene(self, ts):
        scene_doc = self.db.scene.find({'timestamp': ts})
        if scene_doc.count() != 0:
            hyps = scene_doc[0]['identifiables']
            idx = 0;
            for hyp in hyps:
                images = self.get_object_image(hyp,ts)
                cv2.imwrite(images["gt"] + "_" + str(idx) + '_rgb_' + str(ts) + '_crop.png', images["rgb"])
                cv2.imwrite(images["gt"] + "_" + str(idx) + '_depth_' + str(ts) + '_depthcropped.png', images["depth"])
                cv2.imwrite(images["gt"] + "_" + str(idx) + '_mask_' + str(ts) + '_mask.png', images["mask"])
                idx += 1

    def get_timestamps(self):
        scene_cursor = self.db.cas.find()
        timestamps = []
        for sc in scene_cursor:
            timestamps.append(sc['_timestamp'])
        return timestamps

    def get_image_for_scene_id(self, scene_id, scale_factor):
        cas_document = self.db.cas.find({'_id': scene_id})
        color_cursor = None
        if cas_document.count() != 0:
            color_cursor = self.db.color_image_hd.find({'_id': cas_document[0]['color_image_hd']})
        if color_cursor.count() != 0:
            width = color_cursor[0]['cols']
            height = color_cursor[0]['rows']
            img_data = color_cursor[0]['data']
            image = np.reshape(np.fromstring(img_data, np.uint8), (height, width, 3))
            small = cv2.resize(image, (0, 0), fx=scale_factor, fy=scale_factor)
            return small

    def get_scene_rgb_image(self, ts):
        img = self.get_image_for_scene_id(self.db.scene.find({'timestamp': ts})[0]['_parent'], 1.0)
        return img


if __name__ =="__main__":

    mw=MongoWrapper("PnP09ObjSymbolicGTFixed")
    timestamps = mw.get_timestamps()

    for ts in timestamps:
        rgb_image = mw.get_scene_rgb_image(ts)
        cv2.imwrite('rgb_'+str(ts)+'.png',rgb_image)
        mw.save_object_hypotheses_for_scene(ts)
        # TODO don't break :D
#        break

