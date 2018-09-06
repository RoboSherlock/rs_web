#!/usr/bin/env python

from __future__ import print_function  # In python 2.7

from mercurial.context import memctx

from forms import ScenesForm, HypothesisForm
from flask import Flask, render_template, current_app, request, jsonify, redirect, render_template_string, send_from_directory
from flask_paginate import Pagination, get_page_args

from gevent.pywsgi import WSGIServer
import sys
import re
import json
import time
import numpy as np
import os
from pymongo import MongoClient
from source.mongoclient import MongoWrapper
from source.parser import QueryHandler
from pyparsing import ParseException
import shutil
from models import Scene, Hypothesis, Object
app = Flask(__name__)
app.config.from_pyfile('app.cfg')
database_names = MongoClient().database_names()
print(database_names)
mc = MongoWrapper(dbname='PnP09ObjSymbolicGTFixed')
scene_handler = Scene(mc)
hypothesis_handler = Hypothesis(mc)
object_handler = Object(mc)
qh = QueryHandler(mc)
queries_list = []

loading_type = 0    # 1 - first attempt, 2 - loading scene, 3 - loading on scroll
export_type = 0     # 1 - scenes, 2 - hypothesis, 3 - objects
export_entities = []


@app.route('/', methods=['GET', 'POST'])
@app.route('/query', methods=['GET', 'POST'])
def index():
    print("Rendering rs_live.html", file=sys.stderr)
    return render_template('rs_live.html')


@app.route('/store', methods=['GET', 'POST'])
def object_store_methode():
    global scene_handler
    scene_handler.reset()
    global mc
    timestamps = mc.get_timestamps()
    return render_template('object_store_devel.html', db_names=database_names, timestamps=timestamps,
                           no_of_obj=mc.exist_persistent_obj())


@app.route('/robosherlock/add_new_query', methods=['POST'])
def adding_new_query():
    query = request.json
    queries_list.append(query['query'])
    print(queries_list)
    return 'OK'


@app.route("/set_active_DB", methods=['POST'])
def set_active_db():
    name = request.json
    global mc
    mc = MongoWrapper(dbname=name['activeDB'])
    global qh
    qh.set_mongo_wrapper(mc)
    global scene_handler
    scene_handler.set_mongo_wrp(mc)
    scene_handler.reset()
    global hypothesis_handler
    hypothesis_handler.reset()
    hypothesis_handler.set_mongo_wrp(mc)
    return 'OK'


@app.route('/robosherlock/get_history_query', methods=['POST'])
def get_history():
    data = request.json
    index = int(data['index'])
    response = "{\"item\":\""
    lung_lista = len(queries_list) - 1
    if index <= -1:
        index = -1
    elif index > lung_lista:
        response = response + queries_list[0]
        index = lung_lista
    else:
        response = response + queries_list[lung_lista - index]
    response = response + "\",\"index\":" + str(index) + "}"
    return response


@app.route("/get_timestamps", methods=['GET', 'POST'])
def get_timestamps():
    data = request.data
    global mc
    mc = MongoWrapper(data)
    timestamps = mc.get_timestamps()
    return render_template('timestamps.html', timestamps=timestamps)


@app.route('/get_more_data', methods=['GET', 'POST'])
def get_more_data():
    data = request.data
    if data == "scenes_tab":
        global scene_handler
        return scene_handler.scroll_call()
    elif data == "hypothesis_tab":
        global hypothesis_handler
        return hypothesis_handler.scroll_call()
    elif data == "objects_tab":
        global object_handler
        return object_handler.scroll_call()
    return render_template('emptyPage.html')



@app.route('/prolog_query', methods=['GET', 'POST'])
def query_wrapper():

   if request.method == 'POST':
        data = request.data
        query_in = data
        print("query is: " + query_in, file=sys.stderr)
        if query_in == 'objects':
            return handle_objects()
        elif query_in == 'scenes(Sc,[]).':
            return handle_scenes_devel()
        elif query_in == 'scenes(Sc,[ts>1472563260017208200, ts<1472563330256651508]).':
            return handle_scenes(1472563260017208200, 1472563330256651508)
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


def handle_hypothesis_export():
    pass


def handle_objects_export():
    pass


@app.route('/get_objects', methods=['POST', 'GET'])
def handle_objects():
    global object_handler
    object_handler.reset()
    return object_handler.first_call()


@app.route('/get_scenes', methods=['POST', 'GET'])
def handle_scenes_devel():
    data = request.data
    global scene_handler
    scene_handler.reset()
    template = scene_handler.first_call(data)
    global export_type
    export_type = 1
    return template


@app.route('/get_hypothesis', methods=['POST', 'GET'])
def handle_hypothesis_devel():
    data = request.data
    global hypothesis_handler
    hypothesis_handler.reset()
    return hypothesis_handler.first_call(data)


def handle_scenes(ts1 = None, ts2 = None):
    print("handle_scenes.", file=sys.stderr)
    timestamps = mc.get_timestamps()
    start_time = time.time()
    idx_begin = 0
    idx_end = len(timestamps)
    if ts1 != None and ts2!=None:
        idx_begin = timestamps.index(ts1)
        idx_end = timestamps.index(ts2)
    scenes = []
    for ts in timestamps[idx_begin:idx_end]:  # [idxB:idxE]:
        scene = {'ts': ts, 'rgb': mc.get_scene_image(ts), 'objects': mc.get_object_hypotheses_for_scene(ts)}
        scenes.append(scene)
        export_entities.append(scene)

    global export_type
    export_type = 1
    print("getting data took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
    start_time = time.time()
    template = render_template('scenes.html', scenes=scenes)
    print("rendering took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
    return template


def handle_scenes_export():
    global scene_handler
    global scene_handler
    scene_handler.export_all()
    return send_from_directory('./', 'scenes.zip', mimetype="application/zip")


@app.route('/export_data', methods=['POST'])
def export_data():
    if export_type == 1:
        return handle_scenes_export()
    elif export_type == 2:
        return handle_hypothesis_export()
    elif export_type == 3:
        return handle_objects_export()

    return 'NO'


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
