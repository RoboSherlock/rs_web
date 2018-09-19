
from models import Scene, Hypothesis, Object
from source.mongoclient import MongoWrapper

class UserController:

    def __init__(self, mongo_name=None):
        self.mongo_wapper = MongoWrapper()
        self.mono_db_names = []
        if mongo_name is None:
            self.mono_db_names = self.mongo_wapper.get_db_names()
        self.scenes_handler = Scene(self.mongo_wapper)
        self.hypothesis_handler = Hypothesis(self.mongo_wapper)
        self.objects_handler = Object(self.mongo_wapper)
        self.active_data_type = self.scenes_handler

    def set_active_db(self, db_name):
        mongo_wrapper = MongoWrapper(dbname=db_name)
        self.scenes_handler.set_mongo_wrp(mongo_wrapper)
        self.hypothesis_handler.set_mongo_wrp(mongo_wrapper)
        self.objects_handler.set_mongo_wrp(mongo_wrapper)

    def first_call(self, call_type, query):
        if call_type is "hypos":
            return self.hypothesis_handler.first_call(query)
        elif call_type is "scenes":
            return self.scenes_handler.first_call(query)
        elif call_type is "objects":
            return self.objects_handler.first_call()

    def prepare_export(self, export_type):
        if export_type == "export_scenes":
            self.scenes_handler.prepare_export()
            self.active_data_type = self.scenes_handler
        elif export_type == "export_hypothesis":
            self.hypothesis_handler.prepare_export()
            self.active_data_type = self.hypothesis_handler
        elif export_type == "export_objects":
            self.objects_handler.prepare_export()
            self.active_data_type = self.objects_handler
        return "OK"

    def get_timestamps(self):
        return self.mongo_wapper.get_timestamps()

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