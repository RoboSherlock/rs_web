# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""

from __future__ import division
from __future__ import print_function  # In python 2.7

import sys

from bson.objectid import ObjectId

from pymongo import MongoClient
import cv2
import numpy as np
import base64
import time


class MongoWrapper(object):
    def __init__(self, dbname=None):
        if dbname is None:
            dbname = 'IJRRScenes'
        self.client = MongoClient()
        self.db = self.client[dbname]
        self.active_collection = None

    def get_hypos_for_obj(self, id):
        my_db = self.db.persistent_objects
        query = [{'$skip': int(id) - 1}, {'$limit': 1}, {'$project': {'clusters': 1, '_id': 0}}, {'$unwind': '$clusters'}]
        clusters = list(my_db.aggregate(query))
        return [x['clusters'] for x in clusters]

    def set_main_collection(self, _type):
        if _type == 'object':
            self.active_collection = self.db.persistent_objects
        else:
            # if we're looking for views or scenes
            print('Setting scene as active collection')
            self.active_collection = self.db.scene

    def call_query(self, query):
        cursor = self.active_collection.aggregate(query)
        print('\033[93mQuery resulted in %s results\033[0m' % cursor)
        return cursor

    def exist_collection(self, collection):
        return self.db[collection].count()

    def exist_persistent_obj(self):
        return self.exist_collection('persistent_objects')

    def get_object_image(self, obj_entry, ts, export=False):
        # start_time =time.time()
        x = obj_entry['rois']['roi_hires']['pos']['x']
        y = obj_entry['rois']['roi_hires']['pos']['y']
        obj_height = obj_entry['rois']['roi_hires']['size']['height']
        obj_width = obj_entry['rois']['roi_hires']['size']['width']
        img_id = self.db.cas.find({'_timestamp': ts})[0]['color_image_hd']
        color_cursor = self.db.color_image_hd.find({'_id': img_id})
        imgs_data = {}
        if color_cursor.count() != 0:
            width = color_cursor[0]['cols']
            height = color_cursor[0]['rows']
            img_data = color_cursor[0]['data']
            image = np.reshape(np.fromstring(img_data, np.uint8), (height, width, 3))
            obj_image = image[y:y + obj_height, x:x + obj_width]
            small = cv2.resize(obj_image, (0, 0), fx=100 / obj_height, fy=100 / obj_height)
            height, width = small.shape[:2]
            if width > 150:
                small = cv2.resize(obj_image, (0, 0), fx=150 / obj_width, fy=150 / obj_width)
            imgs_data['rgb'] = small
        if export is not False:
            depth_id = self.db.cas.find({'_timestamp': ts})[0]['depth_image_hd']
            depth_cursor = self.db.depth_image_hd.find({'_id': depth_id})
            if depth_cursor.count() != 0:
                width = depth_cursor[0]['cols']
                height = depth_cursor[0]['rows']
                img_data = depth_cursor[0]['data']
                image = np.reshape(np.fromstring(img_data, np.uint16), (height, width, 1))
                obj_image = image[y:y + obj_height, x:x + obj_width]
                small = cv2.resize(obj_image, (0, 0), fx=100 / obj_height, fy=100 / obj_height)
                height, width = small.shape[:2]
                if width > 150:
                    small = cv2.resize(obj_image, (0, 0), fx=150 / obj_width, fy=150 / obj_width)
                imgs_data['depth'] = small
                return imgs_data
        else:
            return imgs_data['rgb']

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
    @staticmethod
    def adjust_annotation(a):
        if a['_type'] == 'rs.annotation.ColorHistogram':
            b = a
            width = a['hist']['cols']
            height = a['hist']['rows']
            img_data = a['hist']['data']

            b['values'] = np.fromstring(img_data, np.float32)
            b['bins'] = height * width
            return b
        elif a['_type'] == 'rs.pcl.PclFeature':
            b = a
            b['values'] = np.fromstring(a['feature'], np.float32)
            b['bins'] = b['values'].size
            return b
        else:
            return a

    def get_all_persistent_objects(self):
        po_cursor = self.db.persistent_objects.find()
        return self.process_objects_cursor(po_cursor)

    def get_all_persistent_obj_cursor(self):
        return self.db.persistent_objects.find()

    def process_objects_cursor(self, ob_cursor):
        objects = []
        for objEntry in ob_cursor:
            ts = 0
            obj = {}
            try:
                ts = objEntry['lastSeen']
                obj = {'image': self.get_base64_img(self.get_object_image(objEntry, ts)),
                       'annotations': self.get_persistent_object_annotations(objEntry)}
            except:
                cas_cursor = self.db.cas.find({'_id': objEntry['_parent']})
                if cas_cursor.count() != 0:
                    ts = cas_cursor[0]['_timestamp']
                obj = {'image': self.get_base64_img(self.get_object_image(objEntry['identifiables'], ts)),
                   'annotations': self.get_persistent_object_annotations(objEntry['identifiables'])}
            objects.append(obj)
        return objects

    def process_my_hypos(self, hypo_cursor):
        hypos = []
        for curs in hypo_cursor:
            cas_cursor = self.db.cas.find({'_id': curs['_parent']})
            if cas_cursor.count() != 0:
                ts = cas_cursor[0]['_timestamp']
                obj = {'image': self.get_object_image(curs['identifiables'], ts, export=True),
                       'annotations': self.get_persistent_object_annotations(curs['identifiables'])}
                hypos.append(obj)
        return hypos

    def process_my_obj(self, obj_cursor):
        objs = []
        for curs in obj_cursor:
            ts = curs['lastSeen']
            obj = {'image': self.get_object_image(curs, ts),
                   'annotations': self.get_persistent_object_annotations(curs)}
            objs.append(obj)
        return objs

    def get_object_instances(self, object_id):
        nr_of_objs = self.db.persistent_objects.count()
        if object_id > nr_of_objs:
            return
        po_cursor = self.db.persistent_objects.find()
        cluster_ids = po_cursor[object_id]['clusters']
        clusters = []
        for c in cluster_ids:
            document = self.db.scene.find({'identifiables._id': ObjectId(c)},
                                          {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})
            if document.count() != 0:
                cluster = {}
                ts = document[0]['timestamp']
                cluster= {'image':self.get_base64_img(self.get_object_image(document[0]['identifiables'][0], ts)),
                          'annotations':self.get_persistent_object_annotations(document[0]['identifiables'][0])}
                clusters.append(cluster)
        return clusters

    def get_object_hypotheses_for_scene(self, ts):
        # start_time = time.time()
        scene_doc = self.db.scene.find({'timestamp': ts})
        # print("finding TS took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        obj_hyps = []
        if scene_doc.count() != 0:
            hyps = scene_doc[0]['identifiables']
            for hyp in hyps:
                obj_hyp = {'image': self.get_base64_img(self.get_object_image(hyp, ts)),
                        'annotations': self.get_persistent_object_annotations(hyp)}
                obj_hyps.append(obj_hyp)
        # print("getting obj_hyps for scene took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
        return obj_hyps

    def get_object_data(self, obj_entry):
        x = obj_entry['rois']['roi_hires']['pos']['x']
        y = obj_entry['rois']['roi_hires']['pos']['y']
        obj_height = obj_entry['rois']['roi_hires']['size']['height']
        obj_width = obj_entry['rois']['roi_hires']['size']['width']
        data = {'x': x, 'y': y, 'height': obj_height, 'width': obj_width, 'label': 'no_label'}
        annotations = obj_entry['annotations']
        for annotation in annotations:
            if annotation['_type'] == 'rs.annotation.GroundTruth':
                data['label'] = annotation['classificationGT']['classname']
        return data

    def get_object_data_for_scene(self, ts):
        scene_doc = self.db.scene.find({'timestamp': ts})
        obj_data = []
        if scene_doc.count != 0:
            hyps = scene_doc[0]['identifiables']
            i = 0
            for hyp in hyps:
                data = self.get_object_data(hyp)
                data['obj_no'] = i
                i = i + 1
                obj_data.append(data)

        return obj_data


    def get_base64_img(self, img):
        [ret, png] = cv2.imencode('.png', img)
        b64 = base64.b64encode(png.tostring())
        return 'data:image/png;base64,' + b64

    def get_timestamps(self):
        scene_cursor = self.db.cas.find()
        timestamps = []
        for sc in scene_cursor:
            timestamps.append(sc['_timestamp'])
        return timestamps

    def get_image_for_scene_id(self, scene_id, scale_factor):
        cas_document = self.db.cas.find({'_id': scene_id})
        color_cursor = None
        depth_cursor = None
        scene_img = {}
        if cas_document.count() != 0:
            color_cursor = self.db.color_image_hd.find({'_id': cas_document[0]['color_image_hd']})
            depth_cursor = self.db.depth_image_hd.find({'_id': cas_document[0]['depth_image_hd']})
        if color_cursor is not None and color_cursor.count() != 0:
            width = color_cursor[0]['cols']
            height = color_cursor[0]['rows']
            img_data = color_cursor[0]['data']
            image = np.reshape(np.fromstring(img_data, np.uint8), (height, width, 3))
            small = cv2.resize(image, (0, 0), fx=scale_factor, fy=scale_factor)
            scene_img['rgb'] = small
        else:
            scene_img['rgb'] = np.zeros((50, 50, 3), np.uint8)

        if depth_cursor is not None and depth_cursor.count() != 0:
            width = depth_cursor[0]['cols']
            height = depth_cursor[0]['rows']
            img_data = depth_cursor[0]['data']
            image = np.reshape(np.fromstring(img_data, np.uint16), (height, width, 1))
            small = cv2.resize(image, (0, 0), fx=scale_factor, fy=scale_factor)
            scene_img['depth'] = small
        else:
            scene_img['depth'] = np.zeros((50, 50, 3), np.uint8)

        return scene_img

    def get_scene_images(self, timestamps):
        images = []
        for ts in timestamps:
            images.append(self.get_base64_img(
                self.get_image_for_scene_id(self.db.scene.find({'timestamp': ts})[0]['_parent'], 0.22)))
        return images

    def get_scene_image(self, ts):
        # start_time = time.time()
        print(ts)
        img = self.get_image_for_scene_id(self.db.scene.find({'timestamp': ts})[0]['_parent'], 0.22)
        base64_img = self.get_base64_img(img['rgb'])

        return {'img_b64': base64_img, 'img': img['rgb'], 'depth': img['depth']}

    #add for set correcting groundTruth in database.....
    def setGTinDB(self, inputTS, imgNumber, objName):
        print("database Name", self.db)
        identifiable_index = imgNumber-1
        identifiable= self.db.scene.find({'timestamp':inputTS})[0]['identifiables'][identifiable_index]
        annotations = identifiable['annotations']

        gt_index = -1
        idx = 0
        for a in annotations:
            if a['_type'] == 'rs.annotation.GroundTruth':
                gt_index = idx
            idx = idx + 1
        if gt_index != -1:
            self.db.scene.update({"timestamp": inputTS},
                                 {"$set":{"identifiables"+"."+str(identifiable_index)+"."+
                                          "annotations"+"."+str(gt_index)+"."+
                                          "classificationGT"+"."+"classname": objName }})
        else:
            self.db.scene.update({'timestamp':inputTS},
                                 {'$push': {'identifiables'+"."+str(identifiable_index)+"."+
                                            'annotations': {'classificationGT': {"classname": objName,
                                                                                 'featurename': '',
                                                                                 'classifier': 'GroundTruth',
                                                                                 'model': ''},
                                                            '_type': 'rs.annotation.GroundTruth'}
                                            }})

    def countHypothesesWithAnnotations(self):
        global_count = 0
        nr_of_objs = self.db.persistent_objects.count()
        po_cursor = self.db.persistent_objects.find()
        dict = {}
        for i in range(0, nr_of_objs):
            cluster_ids = po_cursor[i]['clusters']

            count = 0
            obj_dict = {}
            for c in cluster_ids:
                document = self.db.scene.find({'identifiables._id': ObjectId(c)},
                                              {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})

                for annot in document[0]['identifiables'][0]['annotations']:
                    if annot['_type'] == "rs.annotation.Detection":
                        obj_dict[str(annot['name'])] = obj_dict.get(str(annot['name']),0) + 1
                        count=count+1
            global_count = global_count+count
            dict[i+1]=obj_dict
            print("Object{0} : '{1}' hypotheses : {2} detections ".format(i+1, len(cluster_ids),count))
        print(dict)
        return global_count

    def historyOfAnnotaionValue(self, obj_id):
        global_count = 0
        nr_of_objs = self.db.persistent_objects.count()
        po_cursor = self.db.persistent_objects.find()
        sc_cursor = self.db.scene.find()
        first_timestamp= sc_cursor[0]['timestamp']
        print(first_timestamp)
        cluster_ids = po_cursor[obj_id]['clusters']
        count = 0
        obj_dict = {}
        index = 1
        for c in cluster_ids:
            document = self.db.scene.find({'identifiables._id': ObjectId(c)},
                                          {'_id': 0, 'identifiables._id.$': 1, 'timestamp': 1})

            ts = document[0]['timestamp']
            elapsed = (ts - first_timestamp)/1000000000
            # print(elapsed)
            for annot in document[0]['identifiables'][0]['annotations']:
                if annot['_type'] == "rs.annotation.Detection":
                    obj_dict[str(annot['name'])] = obj_dict.get(str(annot['name']),0) + 1

                    elapsed_seen = (ts - first_timestamp)/1000000000
                    print("{0} ::: {1}".format(index, elapsed_seen))

                    count= count+1
            index += 1
        global_count = global_count+count

        print("Object{0} : '{1}' hypotheses : {2} detections ".format(obj_id+1, len(cluster_ids),count))
        print(obj_dict)
        return global_count


if __name__ == "__main__":

    mc = MongoWrapper();
    c = mc.historyOfAnnotaionValue(11)
    print (c)
