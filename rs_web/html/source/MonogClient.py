# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 12:50:56 2016

@author: ferenc
"""

from pymongo import MongoClient

collections = {'rgb:color_imgage_hd','depth:depth_imgage_hd','scene:scene','objects:persistent_objects'}


client = MongoClient()
db = client["Scenes_annotated"]


for collectionName in db.collection_names():
    print collectionName

scene_cursor = db.scene.find()
for sc in scene_cursor:
    for key, value in sc.iteritems():
        print key
#    for s in sc['identifiables']:
#    for s in sc:
#        print s
#    identifCursor = sc.identifiables.find()