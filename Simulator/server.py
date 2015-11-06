from flask import Flask, request, send_from_directory
import json
from simulator import *
from algorithm import step_algorithm

app = Flask(__name__)
app.arena_map = ArenaMap()


@app.route("/")
def page_index():
    return send_from_directory(".", "index.html")


@app.route("/mapgen.js")
def page_mapgen_js():
    return send_from_directory(".", "mapgen.js")


@app.route("/jquery.js")
def page_jquery_js():
    return send_from_directory(".", "jquery.js")


@app.route("/mapgen.css")
def page_mapgen_css():
    return send_from_directory(".", "mapgen.css")


@app.route("/get", methods=['get', 'post'])
def page_get():
    req_data = request.json
    if req_data is not None:
        print req_data
        if req_data['method'] == "set_obs":
            obs_str = req_data['obs_grid']
            obs_grid_new = decode_grid(obs_str)
            app.arena_map.obs_grid = obs_grid_new
        elif req_data['method'] == "step":
            step_algorithm(app.arena_map)
        elif req_data['method'] == "reset":
            app.arena_map.__init__()

    return json.dumps(
        {
            "map": app.arena_map.get_encoded_map(),
            "obs": app.arena_map.get_encoded_obs(),
            "w_number_grid": app.arena_map.w_number_grid,
            "w_detail_grid": app.arena_map.w_detail_grid,
            "robot": {
                'x': app.arena_map.get_robot_coord()[0],
                'y': app.arena_map.get_robot_coord()[1],
                'degree': app.arena_map.get_robot_degree()
            },
            "state": app.arena_map.robot_state
        })


@app.route("/command")
def page_command():
    return "Hello World!"


if __name__ == "__main__":
    app.debug = True
    app.run("0.0.0.0", 5001)
