
from models import Scene, Hypothesis, Object
from source.mongoclient import MongoWrapper

class UserController:

    def __init__(self, mongo_wapper):
        self.scenes_handler = Scene(mongo_wapper)
        self.hypothesis_handler = Hypothesis(mongo_wapper)
        self.objects_handler = Object(mongo_wapper)

    def set_active_db(self, db_name):
        mongo_wrapper = MongoWrapper(dbname=db_name)
        self.scenes_handler.set_mongo_wrp(mongo_wrapper)
        self.hypothesis_handler.set_mongo_wrp(mongo_wrapper)
        self.objects_handler.set_mongo_wrp(mongo_wrapper)

    