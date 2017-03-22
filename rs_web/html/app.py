from __future__ import print_function # In python 2.7

from flask import Flask, render_template,current_app,request,jsonify
from flask_paginate import Pagination, get_page_args

import sys
import re
import json
import time

from source.mongoclient import RSMongoClient
from source.parser import QueryHandler

app = Flask(__name__)
app.config.from_pyfile('app.cfg')

mc = RSMongoClient('Scenes_annotated')
qh = QueryHandler()

@app.route('/', methods= ['GET','POST'])
@app.route('/query',methods= ['GET','POST'])
def index():
    query_in=None
    if (request.method == 'POST'):
        query_in = request.data
        print("Query is : ",query_in,file = sys.stderr)
        param = re.search(r"\(([0-9])\)",query_in)
        if param:            
            return findObjectInstances(int(param.group(1)))
        elif query_in == 'objects':
            return handle_objects()
        elif query_in == 'scenes':
            return handle_scenes()
    qh.parse_query(query_in)
    print("Method was not POST",file = sys.stderr) 
    print(request.data,file = sys.stderr)
    print("Rendering base.html",file=sys.stderr)
    return render_template('base.html')

@app.route('/prolog_query',methods= ['GET','POST'])
def query_wrapper():
    query_in = None
    if (request.method == 'POST'):
        query_in = request.data
        print("query is: "+query_in,file=sys.stderr)
        if query_in == 'objects':
            return handle_objects()
        elif query_in == 'scenes':
            return handle_scenes()
        else:
            qh.parse_query(query_in)


@app.route('/_get_queries', methods = ['GET'])
def serveStaticFile():
    config = json.loads(open('testQueries.json').read())
    return jsonify(config)
 

def handle_objects():
    objs = mc.get_persistent_objects()
    print("handle_objects.",file=sys.stderr)
    return render_template('objects.html', objects=objs)


def handle_scenes():
    print("handle_scenes.",file=sys.stderr)    
    timestamps = mc.getTimestamps()
#    total = len(timestamps)
#    page, per_page, offset = get_page_args()
#    
#    idxB = (page-1)*per_page
#    idxE = page*per_page  
    scenes = []
    

    start_time = time.time()

    for ts in timestamps:#[idxB:idxE]:
        scene = {}
        scene['ts']= ts
        scene['rgb'] = mc.get_scene_image(ts)
        scene['objects'] = mc.getObjectHypsForScene(ts)
        scenes.append(scene)
    print("getting data took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
#    pagination = get_pagination(page=page,
#                                per_page=per_page,
#                                total=total,
#                                record_name='scenes',
#                                format_total=True,
#                                format_number=True,
#                                )
    start_time =time.time()
    templ = render_template('scenes.html',scenes=scenes)
    print("rendering took: %s seconds ---" % (time.time() - start_time),file=sys.stderr)
    return templ
#    return render_template('scenes.html',
#                               scenes=scenes)
#                               page=page,
#                               per_page=per_page,
#                               pagination=pagination)
  
def findObjectInstances(objID):
    objs = mc.get_object_instances(objID)
    print("handle_object_instances",file=sys.stderr)
    return render_template('objects.html', objects=objs)
    

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

