from flask import Flask, request, send_from_directory
import json
from simulator import *
from algorithm import step_algorithm

app = Flask(__name__)
app.arena = ArenaMap()


@app.route("/")
def page_index():
    return send_from_directory("static", "simulator.html")


@app.route("/mapgen.js")
def page_mapgen_js():
    return send_from_directory("static", "mapgen.js")


@app.route("/jquery.js")
def page_jquery_js():
    return send_from_directory("static", "jquery.js")


@app.route("/mapgen.css")
def page_mapgen_css():
    return send_from_directory("static", "mapgen.css")


@app.route("/get", methods=['get', 'post'])
def page_get():
    req_data = request.json
    if req_data is not None:
        print req_data
        if req_data['method'] == "set_obs":
            obs_str = req_data['obs_grid']
            obs_grid_new = decode_grid(obs_str)
            app.arena.obs_grid = obs_grid_new
        elif req_data['method'] == "step":
            step_algorithm(app.arena)
        elif req_data['method'] == "reset":
            obs_preserved = app.arena.obs_grid
            app.arena.__init__()
            app.arena.obs_grid = obs_preserved
        elif req_data['method'] == "set_max_time":
            max_time = req_data['max_time']
            if max_time is None:
                max_time = 5 * 60
            app.arena.time_max = max_time
        elif req_data['method'] == "set_max_percentage":
            max_percentage = req_data['max_percentage']
            print max_percentage
            if max_percentage is None:
                max_percentage = 100
            app.arena.max_exploration_percentage = max_percentage

    return json.dumps(
        {
            "map": app.arena.get_encoded_map(),
            "obs": app.arena.get_encoded_obs(),
            "w_number_grid": app.arena.w_number_grid,
            "w_detail_grid": app.arena.w_detail_grid,
            "timing":{
                "explore": app.arena.get_time_explore(),
                "shortest": app.arena.get_time_shortest()
            },
            "robot": {
                'x': app.arena.get_robot_coord()[0],
                'y': app.arena.get_robot_coord()[1],
                'degree': app.arena.get_robot_degree()
            },
            "state": app.arena.robot_state
        })


@app.route("/command")
def page_command():
    return "Hello World!"


if __name__ == "__main__":
    app.debug = True
    app.run("0.0.0.0", 8000)
