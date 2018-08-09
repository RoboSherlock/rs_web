from source.mongoclient import MongoWrapper
from flask import render_template


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
            export_scene = {'ts': ts, 'rgb': img['img'], 'depth': img['depth'], 'objects': ''}
            self.export_data.append(export_scene)

        self.active = True
        self.index = self.index + one_step
        return render_template('scenes.html', scenes=self.scenes)

    def scroll_call(self):
        if self.active is True:
            one_step = 2
            if self.no_of_scenes - self.index < one_step:
                one_step = self.no_of_scenes - self.index
            scenes = []
            for ts in self.timestamps[self.index:self.index+one_step]:
                scene = {'ts': ts, 'rgb': self.mongo_wrp.get_scene_image(ts), 'objects': self.mongo_wrp.get_object_hypotheses_for_scene(ts)}
                self.scenes.append(scene)
                scenes.append(scene)
            template = render_template('one_scene.html', scenes=scenes, index=self.index)
            self.index = self.index + one_step
            return template
        return ''


class Filter:
    def __init__(self):
        pass

    def apply_filter(self):
        pass