from flask import Flask, render_template,current_app,request
from flask_paginate import Pagination, get_page_args

from pymongo import MongoClient

import base64
import numpy as np
import cv2

from math import ceil

app = Flask(__name__)
app.config.from_pyfile('app.cfg')


dbName = "Scenes_annotated"
#dbName = "PnP9Obj"
#dbName = "kitchen"
client = MongoClient()
db = client[dbName]


for collectionName in db.collection_names():
    print collectionName

keyOut=''
scene_cursor = db.cas.find()
timestamps = []
for sc in scene_cursor:
    timestamps.append(sc['_timestamp'])
    for key, value in sc.iteritems():
        if key == 'annotations':
            keyOut=key

def getBase64Img(img):
  [ret,png] = cv2.imencode('.png',img) 
  b64 = base64.b64encode(png.tostring())
  return 'data:image/png;base64,'+b64

def get_image_for_sceneID(sceneId,scaleFactor):    
  casDocument = db.cas.find({'_id':sceneId})
  if casDocument.count() !=0: 
    colorCursor= db.color_image_hd.find({'_id':casDocument[0]['color_image_hd']})
    if colorCursor.count()!=0:
      width = colorCursor[0]['cols']
      height = colorCursor[0]['rows']            
      imgData = colorCursor[0]['data']
      image = np.reshape(np.fromstring(imgData,np.uint8),(height,width,3))
      small = cv2.resize(image, (0,0), fx=scaleFactor, fy=scaleFactor) 
      return small


def getImages(timestamps):
  imgs = []
  for ts in timestamps:
    imgs.append(getBase64Img(get_image_for_sceneID(db.scene.find({'timestamp':ts})[0]['_parent'],0.22)))
  return imgs


#@app.route('/rs_test.html')
#def hello(dbname=None,rows=[],images =[]):
#  imgs = getImages(1,10)
#  return render_template('bkup.html', dbname=dbName,rows=timestamps,images=imgs)


@app.route('/scenes', methods= ['GET','POST'])
def index():
    total = len(timestamps)
    page, per_page, offset = get_page_args()
    
    idxB = (page-1)*per_page
    idxE = page*per_page  
    ts = timestamps[idxB:idxE]
    imgs = getImages(ts)
    pagination = get_pagination(page=page,
                                per_page=per_page,
                                total=total,
                                record_name='scenes',
                                format_total=True,
                                format_number=True,
                                )
    return render_template('objStore.html', timestamps=ts,
                           images=imgs,
                           page=page,
                           per_page=per_page,
                           pagination=pagination,
                           )

@app.route('/objects', methods=['GET', 'POST'])
def handle_objects():
    # do something to send email
    if request.method == 'POST':
        return render_template('bkup.html')
    print 'button pressed'
    

def get_pagination(**kwargs):
    kwargs.setdefault('record_name', 'records')
    return Pagination(css_framework=get_css_framework(),
                      link_size=get_link_size(),
                      show_single_page=show_single_page_or_not(),
                      **kwargs
                      )
                      
def get_css_framework():
    return current_app.config.get('CSS_FRAMEWORK', 'bootstrap3')


def get_link_size():
    return current_app.config.get('LINK_SIZE', 'sm')


def show_single_page_or_not():
    return current_app.config.get('SHOW_SINGLE_PAGE', False)

if __name__ == '__main__':
    app.run(use_reloader=True, debug=True, host="0.0.0.0", threaded=True)

