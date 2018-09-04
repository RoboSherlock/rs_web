from source.mongoclient import MongoWrapper
from flask import render_template
import os
import shutil
import json
import cv2
from bson import ObjectId
import json
import numpy as np
class Scene:

    def __init__(self, mongo_wrp):
        self.mongo_wrp = mongo_wrp
        self.no_of_scenes = 0
        self.index = 0
        self.timestamps = []
        self.scenes = []
        self.active = False
        self.export_data = []

    def reset(self):
        self.no_of_scenes = 0
        self.index = 0
        self.timestamps = []
        self.scenes = []
        self.active = False

    def first_call(self, filter=None):
        self.timestamps = self.mongo_wrp.get_timestamps()
        self.index = 0
        self.no_of_scenes = len(self.timestamps)
        one_step = 2
        if self.no_of_scenes < one_step:
            one_step = self.no_of_scenes

        for ts in self.timestamps[self.index:self.index+one_step]:
            img = self.mongo_wrp.get_scene_image(ts)
            scene = {'ts': ts, 'rgb': img['img_b64'], 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
            self.scenes.append(scene)
            export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'],
                            'objects': self.mongo_wrp.get_object_data_for_scene(ts)}

            self.export_data.append(export_scene)

        self.active = True
        self.index = self.index + one_step
        return render_template('scenes.html', scenes=self.scenes)

    def set_mongo_wrp(self, mongo_wrapper):
        self.mongo_wrp = mongo_wrapper

    def scroll_call(self):
        if self.active is True and self.index < self.no_of_scenes:
            one_step = 1
            if self.no_of_scenes - self.index < one_step:
                one_step = self.no_of_scenes - self.index
            scenes = []
            for ts in self.timestamps[self.index:self.index+one_step]:
                img = self.mongo_wrp.get_scene_image(ts)
                scene = {'ts': ts, 'rgb': img['img_b64'], 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
                self.scenes.append(scene)
                export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'],
                                'objects': self.mongo_wrp.get_object_data_for_scene(ts)}
                self.export_data.append(export_scene)
                scenes.append(scene)
            template = render_template('one_scene.html', scenes=scenes, index=self.index)
            self.index = self.index + one_step
            return template
        return 'NU'

    def export_all(self):
        for ts in self.timestamps[self.index:]:
            img = self.mongo_wrp.get_scene_image(ts)
            scene = {'ts': ts, 'rgb': img['img_b64'], 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
            self.scenes.append(scene)
            export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'],
                            'objects': self.mongo_wrp.get_object_data_for_scene(ts)}
            self.export_data.append(export_scene)

        try:
            shutil.rmtree('./scenes')
        except OSError:
            print("dir doesn't exists")

        os.mkdir('./scenes', 0755, )
        path_to_scenes = os.getcwd() + '/scenes'
        for scene in self.export_data:
            ts = scene['ts']
            rgb = scene['rgb']
            depth = scene['depth']
            abs_dir = path_to_scenes + '/scene_' + str(ts)
            rgb_name = abs_dir + '/rgb_' + str(ts) + '.png'
            depth_name = abs_dir + '/depth_' + str(ts) + '.png'
            os.mkdir(abs_dir, 0777)
            cv2.imwrite(rgb_name, rgb)
            cv2.imwrite(depth_name,depth)
            objects = scene['objects']
            objects_name = abs_dir + '/objects_' + str(ts)
            with open(objects_name, 'w') as obj_file:
                json.dump(objects, obj_file, cls=MyJSONEncoder)
            shutil.make_archive('scenes', 'zip', path_to_scenes)


class Hypothesis:
    def __init__(self, mongo_wrapper):
        self.mongo_wrapper = mongo_wrapper
        self.no_of_hypos = 0
        self.index = 0
        self.cursor = None
        self.hypos = []
        self.active = False
        self.export_data = []
        self.step = 5
        self.query = "[{'$project': {'_parent': 1, 'identifiables': 1, '_id': 0}},{'$unwind': '$identifiables'}"
        self.parsers = {'shape': self.parse_shape, 'size': self.parse_size}
        self.limit = 0

    def reset(self):
        self.no_of_hypos = 0
        self.index = 0
        self.hypos = []
        self.active = False
        self.export_data = []
        self.cursor = None
        self.limit = 0

        self.query = [{'$project': {'_parent': 1, 'identifiables': 1, '_id': 0}}, {'$unwind': '$identifiables'}]
        self.parsers = {'shape': self.parse_shape, 'size': self.parse_size}

    def set_mongo_wrp(self, mongo_wrapper):
        self.mongo_wrapper = mongo_wrapper

    def first_call(self, query):
        self.parse_string(query)

        pipeline = self.query[:]

        pipeline.append({'$limit': self.step})
        self.mongo_wrapper.set_main_collection('hypotheses')
        self.cursor = self.mongo_wrapper.call_query(pipeline)
        self.hypos = self.mongo_wrapper.process_objects_cursor(self.cursor)
        template = render_template('objects.html', objects=self.hypos, index=self.index)
        self.index = self.step
        return template

    def scroll_call(self):
        pipeline = self.query[:]
        pipeline.append({'$skip': self.index})
        pipeline.append({'$limit': self.step})
        self.mongo_wrapper.set_main_collection('hypotheses')
        self.cursor = self.mongo_wrapper.call_query(pipeline)
        hypos = self.mongo_wrapper.process_objects_cursor(self.cursor)
        if len(hypos) > 0:
            self.hypos.extend(hypos)
            template = render_template('scroll_objects.html', objects=hypos, index=self.index)
            self.index += self.step
            return template
        else:
            return "NU"

    def parse_string(self, query):
        data_q = json.loads(query)['hypothesis']
        match_dict = {}
        if len(data_q) > 0:
            if 'timestamp' in data_q.keys():
                self.query.insert(0,self.parse_timestamp(data_q['timestamp']))
                del data_q['timestamp']
            if len(data_q) > 0:
                and_dict = {'$and': []}
                match_dict['$match'] = and_dict
                for key, val in data_q.items():
                    match_dict['$match']['$and'].append(self.parsers[key](val))
                self.query.append(match_dict)

    def parse_shape(self, shape):
        return {'identifiables.annotations': {'$elemMatch': {'shape': shape['val'], 'confidence': {'$gt':
                                                                                shape['conf']},
                                                                                '_type': 'rs.annotation.Shape'}}}

    def parse_size(self, size):
        return {'identifiables.annotations': {'$elemMatch': {'size': size['val'] , 'confidence': {'$gt':
                                                                                size['conf']}, '_type': 'rs.annotation.SemanticSize'}}}

    def parse_timestamp(self, timestamp):

        if timestamp['gt'] != '' and timestamp['lt']:
            return {'$match': {'$and': [{'timestamp': {'$gt': np.int64(timestamp['gt'])}}, {'timestamp': {'$lt': np.int64(timestamp['lt'])}}]}}
        elif timestamp['gt'] != '':
            return {'$match': {'timestamp': {'$gt': np.int64(timestamp['gt'])}}}
        elif timestamp['lt'] != '':
            return {'$match': {'timestamp': {'$gt': np.int64(timestamp['gt'])}}}



class Filter:
    def __init__(self):
        pass

    def apply_filter(self):
        pass


class MyJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, ObjectId):
            return str(o)
        return json.JSONEncoder.default(self, o)
