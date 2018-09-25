#!/usr/bin/env python
from __future__ import print_function  # In python 2.7
from flask import Flask, render_template, request, jsonify
from gevent.pywsgi import WSGIServer
import sys
import json
from controllers import UserController

app = Flask(__name__)
app.config.from_pyfile('app.cfg')
queries_list = []
user_controller = UserController()


@app.route('/', methods=['GET', 'POST'])
@app.route('/query', methods=['GET', 'POST'])
def index():
    print("Rendering rs_live.html", file=sys.stderr)
    return render_template('rs_live.html')


@app.route('/store', methods=['GET', 'POST'])
def object_store():
    global user_controller
    user_controller = UserController()
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
    if len(sys.argv) == 2:
        http_server = WSGIServer(('', int(sys.argv[1])), app)
    else:
        http_server = WSGIServer(('', 5555), app)
    print("the server is starting ...................")
    http_server.serve_forever()

