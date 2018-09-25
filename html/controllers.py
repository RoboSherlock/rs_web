
from models import Scene, Hypothesis, Object
from source.mongoclient import MongoWrapper

class UserController:

    def __init__(self, mongo_name=None):
        self.mongo_wrapper = MongoWrapper(mongo_name)
        self.mono_db_names = self.mongo_wrapper.get_db_names()
        self.scenes_handler = Scene(self.mongo_wrapper)
        self.hypothesis_handler = Hypothesis(self.mongo_wrapper)
        self.objects_handler = Object(self.mongo_wrapper)
        self.active_data_type = self.scenes_handler

    def get_db_names(self):
        return self.mono_db_names

    def get_no_objs(self):
        no = self.mongo_wrapper.exist_persistent_obj()
        return no

    def set_active_db(self, db_name):
        self.mongo_wrapper = MongoWrapper(dbname=db_name)
        self.scenes_handler.set_mongo_wrp(self.mongo_wrapper)
        self.hypothesis_handler.set_mongo_wrp(self.mongo_wrapper)
        self.objects_handler.set_mongo_wrp(self.mongo_wrapper)

    def first_call(self, call_type, query=''):
        if call_type is "hypos":
            self.hypothesis_handler.reset()
            self.active_data_type = self.hypothesis_handler
            return self.hypothesis_handler.first_call(query)
        elif call_type is "scenes":
            self.scenes_handler.reset()
            self.active_data_type = self.scenes_handler
            return self.scenes_handler.first_call(query)
        elif call_type is "objects":
            self.objects_handler.reset()
            self.active_data_type = self.objects_handler
            return self.objects_handler.first_call()

    def prepare_export(self):
        self.active_data_type.prepare_export()
        return "OK"

    def get_timestamps(self):
        return self.mongo_wrapper.get_timestamps()

    def scroll_call(self, data_type):
        if data_type == "scenes_tab":
            return self.scenes_handler.scroll_call()
        elif data_type == "hypothesis_tab":
            return self.hypothesis_handler.scroll_call()
        elif data_type == "objects_tab":
            return self.objects_handler.scroll_call()
        return "OK"

    def export_all(self):
        return self.active_data_type.export_all()