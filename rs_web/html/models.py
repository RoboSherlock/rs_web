from source.mongoclient import MongoWrapper
from flask import render_template, send_from_directory
import os
import shutil
import json
import cv2
from bson import ObjectId
import json
import numpy as np


def parse_timestamp(timestamp):
    if timestamp['gt'] != '' and timestamp['lt']:
        return {'$match': {'$and': [{'timestamp': {'$gt': np.int64(timestamp['gt'])}},
                                    {'timestamp': {'$lt': np.int64(timestamp['lt'])}}]}}
    elif timestamp['gt'] != '':
        return {'$match': {'timestamp': {'$gt': np.int64(timestamp['gt'])}}}
    elif timestamp['lt'] != '':
        return {'$match': {'timestamp': {'$gt': np.int64(timestamp['gt'])}}}


def intersect3(timestamps):
    inters = set(timestamps[0][:])
    for i in range(1, len(timestamps)):
        inters = inters.intersection(set(timestamps[i]))
    return list(inters)


class Scene:

    def __init__(self, mongo_wrp):
        self.mongo_wrp = mongo_wrp
        self.no_of_scenes = 0
        self.index = 0
        self.timestamps = []
        self.scenes = []
        self.active = False
        self.export_data = []
        self.parsers = {}
        self.obj_size = 0
        self.abs_path = ''

    def reset(self):
        self.no_of_scenes = 0
        self.index = 0
        self.timestamps = []
        self.scenes = []
        self.active = False
        self.export_data = []
        self.obj_size = 0
        self.parsers = {}

    def get_timestamps(self, query):
        data_q = json.loads(query)['scene']
        if len(data_q) > 0:
            pipeline = []
            if 'timestamp' in data_q.keys():
                pipeline.append(parse_timestamp(data_q['timestamp']))
                del data_q['timestamp']
            if len(data_q) > 0:
                pipeline.extend([{'$project': {'timestamp': 1, 'identifiables': 1}}, {'$unwind': '$identifiables'}])
                list_of_queries = []
                for id in data_q['obj']['ids']:
                    pip = pipeline[:]
                    objects = [ObjectId(x) for x in self.mongo_wrp.get_hypos_for_obj(id)]
                    pip.append({'$match': {'identifiables._id': {'$in': objects}}})
                    pip.append({'$project': {'timestamp': 1, '_id': 0}})
                    list_of_queries.append(pip)
                if len(list_of_queries) > 0:
                    return self.call_queries(list_of_queries)
                pipeline.append({'$project': {'timestamp': 1, '_id': 0}})
                return self.mongo_wrp.call_query(pipeline)
            elif len(data_q) == 0:

                pipeline.append({'$project': {'timestamp': 1}})
                self.mongo_wrp.set_main_collection('hypothesis')
                return [ts['timestamp'] for ts in self.mongo_wrp.call_query(pipeline)]
        else:
            self.mongo_wrp.set_main_collection('hypothesis')
            return [ts['timestamp'] for ts in self.mongo_wrp.call_query([{'$project': {'timestamp': 1}}])]

    def parse_objects(self, objs):
        objects = []
        for id in objs['ids']:
            objects.extend([ObjectId(x) for x in self.mongo_wrp.get_hypos_for_obj(id)])
        if len(objects) > 0:
            return {'$match': {'identifiables._id': {'$in': objects}}}

    def call_queries(self, queries):
        self.mongo_wrp.set_main_collection('hypothesis')
        timestmps = []
        for query in queries:
            timestmps.append([ts['timestamp'] for ts in list(self.mongo_wrp.call_query(query))])

        return intersect3(timestmps)

    def first_call(self, query):
        query1 = query
        self.timestamps = self.get_timestamps(query1)
        self.index = 0
        self.no_of_scenes = len(self.timestamps)
        one_step = 0
        if self.no_of_scenes == 0:
            return render_template('emptyPage.html')
        for ts in self.timestamps[self.index:]:
            img = self.mongo_wrp.get_scene_image(ts)
            scene = {'ts': ts, 'rgb': img['img_b64'], 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
            self.scenes.append(scene)
            export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'],
                            'objects': self.mongo_wrp.get_object_data_for_scene(ts)}

            self.export_data.append(export_scene)
            self.obj_size += len(scene['objects']) + 1
            one_step += 1
            if self.obj_size > 7:
                break
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

    def prepare_export(self):
        exp_scens = self.export_data[:]
        for ts in self.timestamps[self.index:]:
            img = self.mongo_wrp.get_scene_image(ts)
            scene = {'ts': ts, 'rgb': img['img_b64'], 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
            self.scenes.append(scene)
            export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'],
                            'objects': self.mongo_wrp.get_object_data_for_scene(ts)}
            exp_scens.append(export_scene)

        try:
            shutil.rmtree('./scenes')
        except OSError:
            print("dir doesn't exists")

        os.mkdir('./scenes', 0755, )
        path_to_scenes = os.getcwd() + '/scenes'
        self.abs_path = path_to_scenes
        for scene in exp_scens:
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

    def export_all(self):
        return send_from_directory(os.getcwd(), 'scenes.zip', mimetype="application/zip")


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
        self.query = [{'$project': {'_parent': 1, 'identifiables': 1, '_id': 1}}, {'$unwind': '$identifiables'}]
        self.parsers = {'shape': self.parse_shape, 'size': self.parse_size, 'obj': self.parse_objects, 'class': self.parse_classification}
        self.limit = 0
        self.path = ''

    def reset(self):
        self.no_of_hypos = 0
        self.index = 0
        self.hypos = []
        self.active = False
        self.export_data = []
        self.cursor = None
        self.limit = 0
        self.query = [{'$project': {'_parent': 1, 'identifiables': 1, '_id': 1}}, {'$unwind': '$identifiables'}]

        self.parsers = {'shape': self.parse_shape, 'size': self.parse_size, 'obj': self.parse_objects, 'class': self.parse_classification}

    def set_mongo_wrp(self, mongo_wrapper):
        self.mongo_wrapper = mongo_wrapper

    def first_call(self, query):
        self.parse_string(query)
        self.mongo_wrapper.set_main_collection('hypotheses')
        self.hypos = []
        no_of_annotations = 0
        while no_of_annotations < 30 and no_of_annotations > -1:
            pipeline = self.query[:]
            pipeline.append({'$skip': self.index})
            pipeline.append({'$limit': 2})
            self.cursor = self.mongo_wrapper.call_query(pipeline)
            hypos = self.mongo_wrapper.process_objects_cursor(self.cursor)
            for hypo in hypos:
                no_of_annotations += len(hypo['annotations'])
            no_of_annotations -= 1
            self.hypos.extend(hypos)
            self.index += len(hypos)
        if len(self.hypos) == 0:
            return render_template('emptyPage.html')
        template = render_template('objects.html', objects=self.hypos, index=0)
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
                self.query.insert(0, parse_timestamp(data_q['timestamp']))
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

    def parse_objects(self, obj):
        objs = [ObjectId(x) for x in self.mongo_wrapper.get_hypos_for_obj(obj['id'])]
        if len(objs) > 0:
            return {'identifiables._id': {'$in': objs}}

    def parse_classification(self, clas):
        return {'identifiables.annotations': {'$elemMatch': {'classname': clas['val'], '_type': 'rs.annotation.Classification'}}}


    def prepare_export(self):
        pipeline = self.query[:]
        self.mongo_wrapper.set_main_collection('hypothesis')
        self.cursor = self.mongo_wrapper.call_query(pipeline)
        export_data(self.cursor, 'hypothesis', self.mongo_wrapper)

    def export_all(self):
        return send_from_directory(os.getcwd(), 'hypothesis.zip', mimetype="application/zip")


class Object:

    def __init__(self, mongo_wrapper):
        self.objects = []
        self.index = 0
        self.mongo_wrapper = mongo_wrapper
        self.abs_path = ''

    def reset(self):
        self.objects = []
        self.index = 0

    def set_mongo_wrp(self, mongo_wrapper):
        self.mongo_wrapper = mongo_wrapper

    def first_call(self):
        self.mongo_wrapper.set_main_collection('object')

        no_of_annot = 0
        while no_of_annot < 30 and no_of_annot >= 0:
            pipeline = [{'$skip': self.index}, {'$limit': 5}]
            cursor = self.mongo_wrapper.call_query(pipeline)
            objects = self.mongo_wrapper.process_objects_cursor(cursor)
            for obj in objects:
                no_of_annot += len(obj['annotations'])
            self.objects.extend(objects)
            self.index += len(objects)
            no_of_annot -= 1

        if len(self.objects) == 0:
            return render_template('emptyPage.html')
        return render_template('objects.html', objects=self.objects, index=0)

    def scroll_call(self):
        self.mongo_wrapper.set_main_collection('object')
        pipeline = [{'$skip': self.index}, {'$limit': 5}]
        cursor = self.mongo_wrapper.call_query(pipeline)
        objects = self.mongo_wrapper.process_objects_cursor(cursor)
        if len(objects) > 0:
            self.objects = objects
            template = render_template('scroll_objects.html', objects=objects, index=self.index)
            self.index += len(objects)
            return template
        else:
            return "NU"

    def prepare_export(self):
        query = [{'$project': {'_parent': 1, 'identifiables': 1, '_id': 1}}, {'$unwind': '$identifiables'}]
        self.mongo_wrapper.set_main_collection('hypothesis')
        cursor = self.mongo_wrapper.call_query(query)
        self.abs_path = export_data(cursor, 'objects', self.mongo_wrapper)


    def prepare_obj_anot(self, annot):
        return 'annotations'

    def export_all(self):
        return send_from_directory(os.getcwd(), 'objects.zip', mimetype="application/zip")

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


def create_directory(directory, path=None):
    abs_path = os.getcwd()
    if path is not None:
        abs_path = path

    abs_path += "/" + directory
    try:
        shutil.rmtree(abs_path)
    except OSError:
        print("dir doesn't exists")

    os.mkdir(abs_path, 0775, )
    return abs_path


def export_data(cursor, directory, mongo_wrapper):
    hypos = mongo_wrapper.process_my_hypos(cursor)
    export_data = {}
    for hypo in hypos:
        categ = prepare_hypos(hypo['annotations'])
        if categ not in export_data.keys():
            export_data[categ] = []
            export_data[categ].append(hypo['image'])
        else:
            export_data[categ].append(hypo['image'])

    path_to_hypos = create_directory(directory)
    keys = export_data.keys()
    index = 0
    for vals in export_data.itervalues():
        index2 = 0
        path_to_acc_hypo = path_to_hypos + '/' + keys[index]
        os.mkdir(path_to_acc_hypo, 0777)
        the_key = keys[index]
        for val in vals:
            rgb = val['rgb']
            depth = val['depth']
            rgb_dir = path_to_acc_hypo + '/' + the_key + '_' + str(index2) + '_crop' + '.png'
            cv2.imwrite(rgb_dir, rgb)
            depth_dir = path_to_acc_hypo + '/' + the_key + '_' + str(index2) + '_depthcrop' + '.png'
            cv2.imwrite(depth_dir, depth)
            index2 += 1
        index += 1
    shutil.make_archive(path_to_hypos, 'zip', path_to_hypos)


def prepare_hypos(idents):
    for ident in idents:
        if ident['_type'] == 'rs.annotation.Detection':
            return ident['name']
        if ident['_type'] == 'rs.annotation.Classification':
            return ident['classname']
    return 'noname'