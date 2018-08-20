from source.mongoclient import MongoWrapper
from flask import render_template
import os
import shutil
import json
import cv2
from bson import ObjectId


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
        if self.active is True:
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
