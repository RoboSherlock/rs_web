from flask import Flask
from flask import render_template

from pymongo import MongoClient

import bson
from bson import Binary

import numpy as np
import cv2 
import cv


app = Flask(__name__)
#collections = {'rgb:color_imgage_hd','depth:depth_imgage_hd','scene:scene','objects:persistent_objects'}


dbName = "Scenes_annotated"
#dbName = "PnP9Obj"
client = MongoClient()
db = client[dbName]


for collectionName in db.collection_names():
    print collectionName

keyOut=''
scene_cursor = db.scene.find()
timestamps = []
for sc in scene_cursor:
    timestamps.append(sc['timestamp'])
    for key, value in sc.iteritems():
        if key == 'annotations':
            keyOut=key
print timestamps


def get_image_for_scene(sceneId):    
    casDocument = db.cas.find({'_id':sceneId})
    if casDocument.count() !=0: 
        colorCursor= db.color_image_hd.find({'_id':casDocument[0]['color_image_hd']})
        if colorCursor.count()!=0:
            width = colorCursor[0]['cols']
            height = colorCursor[0]['rows']            
            print 'Size of image is: '+str(width)+'x'+str(height)+ '\n'
            print 'Type is: ' + str(colorCursor[0]['mat_type'])
            imgData = colorCursor[0]['data']

            nparr = np.fromstring(imgData,np.uint8)
            nppic= np.reshape(nparr,(960,1280,3))
            
            cv2.imshow('display',nppic)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

get_image_for_scene(db.scene.find({'timestamp':timestamps[0]})[0]['_parent'])

@app.route('/rs_test.html')
def hello(dbname=None,rows=[]):
    return render_template('objStore.html', dbname=dbName,rows=timestamps)

#if __name__ == '__main__':
#    app.run(use_reloader=True, debug=False)

