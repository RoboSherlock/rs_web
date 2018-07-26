from __future__ import print_function  # In python 2.7

from flask import Flask, render_template, current_app, request, jsonify, redirect
from flask_paginate import Pagination, get_page_args

from gevent.pywsgi import WSGIServer
import sys
import re
import json
import time
import numpy as np

from source.mongoclient import MongoWrapper
from source.parser import QueryHandler
from pyparsing import ParseException

app = Flask(__name__)
app.config.from_pyfile('app.cfg')
# http_server = WSGIServer(('', 5555), app)

mc = MongoWrapper()
qh = QueryHandler()
querys_list = []

@app.route('/', methods=['GET', 'POST'])
# @app.route('/query', methods=['GET', 'POST'])
def index():
    print("Rendering object_store.html", file=sys.stderr)
    return render_template('rs_live.html')


@app.route('/store', methods=['GET', 'POST'])
def object_store_methode():
    print("Rendering store.html", file=sys.stderr)
    return render_template('object_store.html')

@app.route('/query', methods=['GET', 'POST'])
def query_methode():
    print("Rendering query.html", file=sys.stderr)
    return render_template('rs_live.html')

@app.route('/robosherlock/add_new_query', methods=['POST'])
def adding_new_query():
    query = request.json
    querys_list.append(query['query'])
    print(querys_list)
    return 'OK'


@app.route('/robosherlock/get_history_query', methods=['POST'])
def get_history():
    data = request.json
    index = int(data['index'])
    response = "{\"item\":\""
    lung_lista = len(querys_list) - 1
    if index <= -1:
        index = -1
    elif index > lung_lista:
        response = response + querys_list[0]
        index = lung_lista
    else:
        response = response + querys_list[lung_lista - index]
    response = response + "\",\"index\":" + str(index) + "}"
    return response


@app.route('/prolog_query', methods=['GET', 'POST'])
def query_wrapper():

   if request.method == 'POST':
        query_in = request.data
        print("query is: " + query_in, file=sys.stderr)
        if query_in == 'objects':
            return handle_objects()
        elif query_in == 'scenes(Sc,[]).':
            return handle_scenes()
        elif query_in == 'scenes(Sc,[ts>1482401694215166627, ts<1482401807402294324]).':
            return handle_scenes(1482401694215166627,1482401807402294324)
        elif query_in == 'hypotheses(Hyp, [detection:[confidence>0.5, source:DeCafClassifier], type:\'Cutlery\']).':
            print( 'DO SOME MAGIC',file=sys.stderr)
            o = qh.exec_query(query_in,1)
            return render_template("objects.html", objects=o)
        elif query_in == 'hypotheses(Hyp1, [detection:[confidence>0.5, source:DeCafClassifier]]),scenes(Sc1,[ts>1482401694215166627, ts<1482401807402294324]),hypothesesInScenes(Hyp2,Sc),intersect(Hyp1, Hyp2, R).':
            o = qh.exec_query(query_in,2)
            return render_template("objects.html", objects=o)
        else:
            try:
                objects = qh.exec_query(query_in)
                if 0 != len(objects):
                    return render_template("objects.html", objects=objects)
                else:
                    return render_template("emptyPage.html")
            except ParseException:
                return render_template("emptyPage.html")

@app.route('/_get_queries', methods=['GET'])
def serve_static_file():
    config = json.loads(open('static/queries/testQueries.json').read())
    return jsonify(config)


def handle_objects():
    objects = mc.get_all_persistent_objects()
    print("handle_objects.", file=sys.stderr)
    return render_template('objects.html', objects=objects)


def handle_scenes(ts1 = None, ts2 = None):
    print("handle_scenes.", file=sys.stderr)
    timestamps = mc.get_timestamps()
    # total = len(timestamps)
    # page, per_page, offset = get_page_args()
    # idxB = (page-1)*per_page
    # idxE = page*per_page


    scenes = []
    start_time = time.time()
    idx_begin = 0
    idx_end = len(timestamps)
    if ts1 != None and ts2!=None:
        idx_begin = timestamps.index(ts1)
        idx_end = timestamps.index(ts2)
        # idx_begin = 9
        # idx_end = 19

    for ts in timestamps[idx_begin:idx_end]:  # [idxB:idxE]:
        scene = {'ts': ts, 'rgb': mc.get_scene_image(ts), 'objects': mc.get_object_hypotheses_for_scene(ts)}
        scenes.append(scene)
    print("getting data took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
    # pagination = get_pagination(page=page,
    #                             per_page=per_page,
    #                             total=total,
    #                             record_name='scenes',
    #                             format_total=True,
    #                             format_number=True,
    #                             )
    start_time = time.time()
    template = render_template('scenes.html', scenes=scenes)
    print("rendering took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
    return template
    # return render_template('scenes.html',
    #                         scenes=scenes,
    #                         page=page,
    #                         per_page=per_page,
    #                         pagination=pagination)

#set new root to correct groundTruth annotations
@app.route('/newpost', methods=['GET', 'POST'])
def databaseQ():
    clname = request.form.get('classN')
    timestm = request.form.get('timeS')
    imgnum = request.form.get('imgN')
    print('inputed value is:', clname, timestm, imgnum)
    if not clname and not timestm and not imgnum:
        print("All the GT tool's inputs are empty")
    else:
        mc.setGTinDB(int(timestm), int(imgnum), clname)
        print("Database is updated with GT tool's informations")
    return redirect("Location:http://127.0.0.1")

def find_object_instances(obj_id):
    objects = mc.get_object_instances(obj_id)
    print("handle_object_instances", file=sys.stderr)
    return render_template('objects.html', objects=objects)


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
    # app.run(use_reloader=True, debug=True, host="0.0.0.0", threaded=True, port=5555)
    http_server = WSGIServer(('', 5555), app)
    http_server.serve_forever()
