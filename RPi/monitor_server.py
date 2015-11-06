from flask import Flask, send_from_directory, request
import socket
import json
from constants import *

app = Flask(__name__)
app.sock = None
""":type : socket.socket"""


testing = False


@app.route("/")
def page_index():
    return send_from_directory("static", "monitor.html")


@app.route("/mapgen.js")
def page_mapgen_js():
    return send_from_directory("static", "mapgen.js")


@app.route("/jquery.js")
def page_jquery_js():
    return send_from_directory("static", "jquery.js")


@app.route("/mapgen.css")
def page_mapgen_css():
    return send_from_directory("static", "mapgen.css")


@app.route("/connect12347", methods=['get', 'post'])
def page_connect12347():
    if testing:
        return json.dumps("Connected")
    app.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    app.sock.connect(('192.168.3.3',12347))
    return json.dumps("Connected")


@app.route("/connect12348", methods=['get', 'post'])
def page_connect12348():
    if testing:
        return json.dumps("Connected")
    app.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    app.sock.connect(('192.168.3.3',12348))
    return json.dumps("Connected")



@app.route("/disconnect", methods=['get', 'post'])
def page_disconnect():
    if app.sock is not None:
        app.sock.close()
        app.sock = None
    return json.dumps("Disconnected")


def send_command(command):
    app.sock.send(chr(command))


@app.route("/balance", methods=['get', 'post'])
def page_balance():
    send_command(MCM_INIT_BALANCE)
    return json.dumps("Okay")


@app.route("/align", methods=['get', 'post'])
def page_align():
    send_command(MCM_INIT_ALIGN)
    return json.dumps("Okay")


@app.route("/explore", methods=['get', 'post'])
def page_explore():
    send_command(MCM_START_EXPLORE)
    return json.dumps("Okay")


@app.route("/shortest", methods=['get', 'post'])
def page_shortest():
    send_command(MCM_START_SHORTEST)
    return json.dumps("Okay")


@app.route("/command", methods=['get', 'post'])
def page_command():
    req_data = request.json
    print req_data
    if req_data is not None:
        command = req_data['command']
        send_command(command)
    return json.dumps("Okay")


@app.route("/get_status", methods=['get', 'post'])
def page_get_status():
    if testing:
        return json.dumps({
            "map": "000000800000800080000000008000000000800000000080000000208000000020000002002000800000000000000000000000000200000020000200002000020000200002000000000000",
            "robot":{
                "x": 2.5,
                "y": 2.5,
                "degree": 270.0
            },
            "state": "fake_state"
        })
    app.sock.send(chr(MCM_REQUEST))
    return app.sock.recv(2048)

if __name__ == "__main__":
    # app.debug = True
    app.run("0.0.0.0", 5000)
