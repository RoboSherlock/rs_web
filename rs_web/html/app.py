#!/usr/bin/env python
from __future__ import print_function  # In python 2.7
from flask import Flask, render_template, request, jsonify
from gevent.pywsgi import WSGIServer
import sys
import json
import time
from source.mongoclient import MongoWrapper
from pyparsing import ParseException
from controllers import UserController
from source.parser import QueryHandler

app = Flask(__name__)
app.config.from_pyfile('app.cfg')
queries_list = []

#mc = MongoWrapper(dbname='PnP09ObjSymbolicGTFixed')
#qh = QueryHandler(mc)
#export_entities = []
user_controller = UserController()


@app.route('/', methods=['GET', 'POST'])
@app.route('/query', methods=['GET', 'POST'])
def index():
    print("Rendering rs_live.html", file=sys.stderr)
    return render_template('rs_live.html')


@app.route('/store', methods=['GET', 'POST'])
def object_store():
    names = user_controller.get_db_names()
    return render_template('object_store_devel.html', db_names=names,
                           timestamps=user_controller.get_timestamps(),
                           no_of_obj=user_controller.get_no_objs())


@app.route('/robosherlock/add_new_query', methods=['POST'])
def adding_new_query():
    query = request.json
    queries_list.append(str(query['query']+"."))
    print(queries_list)
    return 'OK'



@app.route('/prepare_export', methods=['POST'])
def set_export_data():
    user_controller.prepare_export()
    return "OK"


@app.route("/change_db", methods=['GET', 'POST'])
def get_timestamps():
    db_name = request.data
    user_controller.set_active_db(db_name)
    return render_template('timestamps.html', timestamps=user_controller.get_timestamps())


@app.route('/get_more_data', methods=['GET', 'POST'])
def get_more_data():
    data_type = request.data
    return user_controller.scroll_call(data_type)


@app.route('/get_objects', methods=['POST', 'GET'])
def handle_objects():
    return user_controller.first_call('objects')


@app.route('/get_scenes', methods=['POST', 'GET'])
def handle_scenes_devel():
    query = request.data
    return user_controller.first_call('scenes', query)


@app.route('/get_hypothesis', methods=['POST', 'GET'])
def handle_hypothesis_devel():
    query = request.data
    return user_controller.first_call('hypos', query)


@app.route('/export_data/<data>', methods=['POST', 'GET'])
def export_data(data):
    user_controller.prepare_export()
    return user_controller.export_all()


#def handle_scenes(ts1 = None, ts2 = None):
#    print("handle_scenes.", file=sys.stderr)
#    timestamps = mc.get_timestamps()
#    start_time = time.time()
#    idx_begin = 0
#    idx_end = len(timestamps)
#    if ts1 != None and ts2!=None:
#        idx_begin = timestamps.index(ts1)
#        idx_end = timestamps.index(ts2)
#    scenes = []
#    for ts in timestamps[idx_begin:idx_end]:  # [idxB:idxE]:
#        scene = {'ts': ts, 'rgb': mc.get_scene_image(ts), 'objects': mc.get_object_hypotheses_for_scene(ts)}
#        scenes.append(scene)
#        export_entities.append(scene)
#    print("getting data took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
#    start_time = time.time()
#    template = render_template('scenes.html', scenes=scenes)
#    print("rendering took: %s seconds ---" % (time.time() - start_time), file=sys.stderr)
#    return template
#
#
#@app.route('/prolog_query', methods=['GET', 'POST'])
#def query_wrapper():
#
#   if request.method == 'POST':
#        data = request.data
#        query_in = data
#        print("query is: " + query_in, file=sys.stderr)
#        if query_in == 'objects':
#            return handle_objects()
#        elif query_in == 'scenes(Sc,[]).':
#            return handle_scenes_devel()
#        elif query_in == 'scenes(Sc,[ts>1472563260017208200, ts<1472563330256651508]).':
#            return handle_scenes(1472563260017208200, 1472563330256651508)
#        elif query_in == 'hypotheses(Hyp, [detection:[confidence>0.5, source:DeCafClassifier], type:\'Cutlery\']).':
#            print( 'DO SOME MAGIC',file=sys.stderr)
#            o = qh.exec_query(query_in,1)
#            return render_template("objects.html", objects=o)
#        elif query_in == 'hypotheses(Hyp1, [detection:[confidence>0.5, source:DeCafClassifier]]),scenes(Sc1,[ts>1482401694215166627, ts<1482401807402294324]),hypothesesInScenes(Hyp2,Sc),intersect(Hyp1, Hyp2, R).':
#            o = qh.exec_query(query_in,2)
#            return render_template("objects.html", objects=o)
#        else:
#            try:
#                objects = qh.exec_query(query_in)
#                if 0 != len(objects):
#                    return render_template("objects.html", objects=objects)
#                else:
#                    return render_template("emptyPage.html")
#            except ParseException:
#                return render_template("emptyPage.html")


@app.route('/_get_queries', methods=['GET'])
def serve_static_file():
    config = json.loads(open('static/queries/testQueries.json').read())
    return jsonify(config)


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


if __name__ == '__main__':
    http_server = WSGIServer(('', 5555), app)
    http_server.serve_forever()
