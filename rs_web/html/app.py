from __future__ import print_function # In python 2.7

from flask import Flask, render_template,current_app,request
from flask_paginate import Pagination, get_page_args

from pymongo import MongoClient
import sys
import base64
import numpy as np
import cv2

from source import RSMongoClient as RSMC

app = Flask(__name__)
app.config.from_pyfile('app.cfg')

mc = RSMC.RSMongoClient('Scenes_annotated')


#@app.route('/rs_test.html')
#def hello(dbname=None,rows=[],images =[]):
#  imgs = getImages(1,10)
#  return render_template('bkup.html', dbname=dbName,rows=timestamps,images=imgs)

mc.getPersistentObjects()

@app.route('/', methods= ['GET','POST'])
@app.route('/scenes',methods= ['GET','POST'])
def index():
    if request.method == 'POST':
       if request.form['submit'] == 'Objects':
         return handle_objects()
    timestamps = mc.getTimestamps()
    total = len(timestamps)
    page, per_page, offset = get_page_args()
    
    idxB = (page-1)*per_page
    idxE = page*per_page  
    ts = timestamps[idxB:idxE]
    imgs = mc.getSceneImages(ts)
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
    objs = mc.getPersistentObjects()
    
    page, per_page, offset = get_page_args()    
    idxB = (page-1)*per_page
    idxE = page*per_page  
    total = len(objs)
    obj=objs[idxB:idxE]
    pagination = get_pagination(page=page,
                                per_page=per_page,
                                total=total,
                                record_name='scenes',
                                format_total=True,
                                format_number=True,
                                )
    if request.method == 'POST':
        if request.form['submit'] == 'Objects':
            print( 'ITT ',file=sys.stderr ) 
            return render_template('bkup.html',objects=objs)    
        elif request.form['submit'] == 'Scenes':
            print( 'ITT is',file=sys.stderr ) 
            return index()
        else:
            print( 'Passzolunk',file=sys.stderr ) 
            pass # unknown
    elif request.method == 'GET':
        return render_template('bkup.html', objects=objs)
  
#    print( request.form, file=sys.stderr)
    
#        return render_template('bkup.html',objects=objs)
    

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

