"use strict";

const C = 0;
const U = 1;
const O = 2;

const WIDTH = 17;
const HEIGHT = 22; 

const JOE_MAP_STR = "000000800000800080000000008000000000800000000080000000208000000020000002002000800000000000000000000000000200000020000200002000020000200002000000000000"
/*
000000800000800080000000008000000000800000000080000000208000000020000002002000800000000000000000000000000200000020000200002000020000200002000000000000
*/

const LEFT = 0, RIGHT=1, TOP = 2, BOTTOM=3;

var MAP_INIT_STR = "";
var OBS_INIT_STR = "";
for(let i = 0; i<150; i++){
    MAP_INIT_STR += "5";
    OBS_INIT_STR += "0";
}

//(function() {
    // exploration map
    var map_grid = [];
    // target map, having only C and O
    var obs_grid = [];
    // w_number_grid, used for displaying W
    var w_number_grid = []
    // w_detail_grid, used for displaying W's details in tooltip
    var w_detail_grid = []
    var robot = {x:2.5, y:2.5, degree: 0.0};
    var state = "Unavailable";

    var createDummyGrid = function(value){
        var grid = [];
        for(var x = 0; x<WIDTH; x++){
            var column = [];
            for (var y = 0; y<HEIGHT; y++){
                column.push(value);
            }
            grid.push(column);
        }
        return grid;
    }

    var resetGrids = function() {
        map_grid = decodeGrid(MAP_INIT_STR);
        obs_grid = decodeGrid(OBS_INIT_STR);
    };

    var codeToClass = function(code) {
        switch(code){
            case C: return "cellC";
            case U: return "cellU";
            case O: return "cellO";
        }
        return "cellBad";
    }

    var degreeToDirection=function(degree){
        if (degree <= 45 || degree > 315)
            return RIGHT;
        if (degree <= 135 && degree >45)
            return TOP;
        if (degree <= 225 && degree > 135)
            return LEFT;
        if (degree <= 315 && degree > 225)
            return BOTTOM;
        throw "Bad degree " + degree;
    }

    var getRobotHead = function(x,y,heading){
        switch(heading){
            case TOP:   return [x, y+1];
            case LEFT:  return [x-1, y];
            case RIGHT: return [x+1, y];
            case BOTTOM:return [x, y-1];
        }
        throw "Bad heading: " + heading;
    }

    var displayGrids = function() {
        // robot location
        var robot_x = Math.floor(robot.x);
        var robot_y = Math.floor(robot.y);
        var head = getRobotHead(robot_x, robot_y, degreeToDirection(robot.degree));
        var head_x = head[0];
        var head_y = head[1];
        // display grids
        var map_view = document.getElementById("map_view");
        var obs_view = document.getElementById("obs_view");

        var do_obs = mode === "simulator";

        map_view.innerHTML = "";
        if(do_obs)
            obs_view.innerHTML = "";
        for (var y = HEIGHT-1; y >= 0; y --) {
            for (var x = 0; x < WIDTH; x ++) {
                var map_cell = document.createElement("div");
                if(do_obs)
                    var obs_cell = document.createElement("div");
                map_cell.className = codeToClass(map_grid[x][y]);
                map_cell.x = x;
                map_cell.y = y;

                if( x===robot_x && y===robot_y )
                    map_cell.className += " cellRobotCenter";

                if( x===head_x  && y===head_y )
                    map_cell.className += " cellRobotHead";

                if(w_number_grid.length && w_number_grid[x][y]!=null){
                    var dist = w_number_grid[x][y];
                    map_cell.innerHTML = dist > 999? "I" : dist;
                    var detail = w_detail_grid[x][y];
                    map_cell.title = JSON.stringify(detail);
                }

                if(do_obs){
                    obs_cell.className = codeToClass(obs_grid[x][y]);
                    obs_cell.x = x;
                    obs_cell.y = y;
                    obs_cell.onclick = function() {
                        obs_grid[this.x][this.y] = (obs_grid[this.x][this.y] === C)? O : C;
                        this.className = codeToClass(obs_grid[this.x][this.y]);
                        displayGrids();
                    };
                }
                map_view.appendChild(map_cell);
                if(do_obs)
                    obs_view.appendChild(obs_cell);
            }
        }

        // update encoded string textarea
        if(do_obs)
            $("#encode-str").val(encodeGrid(obs_grid));

        // update robot state
        $("#robot_state").html(state);

        // update map descriptor
        var desc = "";
        var code = 3;
        var codeCount = 2;
        // known-unknown encoding
        for(var y = 0; y < 20; y++){
            for(var x = 0; x < 15; x++){
                code *= 2;
                // 0 for unknown, 1 for known
                if(map_grid[x+1][y+1] != U)
                    code += 1;
                codeCount++;
                if(codeCount == 4){
                    desc += encode_map[code];
                    codeCount = 0;
                    code = 0;
                }
            }
        }
        code *= 4;
        code += 3;
        desc += encode_map[code];
        // C-O encoding
        var desc2 = "";
        code = 0;
        codeCount = 0;
        for(var y = 0; y < 20; y++){
            for(var x = 0; x < 15; x++){
                var val = map_grid[x+1][y+1];
                if(val == U)
                    continue;
                code *= 2;
                if(val == O)
                    code += 1;
                codeCount++;
                if(codeCount == 4){
                    desc2 += encode_map[code];
                    codeCount = 0;
                    code = 0;
                }
            }
        }
        if(codeCount > 0){
            code <<= 4 - codeCount;
            desc2 += encode_map[code];
        }
        if((desc2.length % 2) != 0)
            desc2 += '0';

        $("#map-desc").val(desc + "\n" + desc2);
    };

    var encode_map = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'];
    var decode_map = {};
    for(let i = 0; i<16; i++) decode_map[encode_map[i]] = i;

    var encodeGrid = function(grid){
        var encoded_str = "";
        for (var i = 0; i<150; i++){
            var code = 0;
            for (var j = 0; j<2; j++){
                var index = i * 2 + j;
                var x = Math.floor(index / 20) + 1;
                var y = index % 20 + 1;
                var inner_code = grid[x][y];
                if (j === 1)
                    inner_code *= 4
                code += inner_code
            }
            encoded_str += encode_map[code]
        }
        return encoded_str;
    }

    var decodeGrid = function(encoded_str){
        var grid = createDummyGrid(O);
        for(var i = 0; i<150; i++){
            var code = decode_map[encoded_str[i]]
            for (var j = 0; j<2; j++){
                var index = i * 2 + j
                var x = Math.floor( index / 20 )
                var y = index % 20
                grid[x + 1][y + 1] = code % 4
                code = Math.floor ( code / 4 )
            }
        }
        return grid;
    }

    var step_enabled = false;
    var step_interval = 0;

    var simulator_timer = null;
    var simulator_time_begin = null;
    var simulator_arrived = false;

    var processServerResponse = function(data){
        console.log(data);
        map_grid = decodeGrid(data.map);
        if(data.obs){ // only available in simulator mode
            obs_grid = decodeGrid(data.obs);
            w_number_grid = data.w_number_grid;
            w_detail_grid = data.w_detail_grid;
        }
        var timing = data.timing;
        $("#run_time").html(timing.explore.toFixed(1) + "||" + timing.shortest.toFixed(1));
        robot = data.robot;
        state = data.state;
        displayGrids();

        if(step_enabled){
            console.log("calling stepServer soon"+step_enabled+step_interval);
            setTimeout(stepServer, step_interval);
        }
    }

    var setObsOnServer = function(){
        syncWithServer("set_obs", processServerResponse);
    }

    var pullFromServer = function(){
        syncWithServer("pull", processServerResponse);
    }

    var stepServer = function(){
        syncWithServer("step", processServerResponse);
    }

    var resetServer = function(){
        syncWithServer("reset", processServerResponse);
    }

    var set_max_time = function(){
        syncWithServer("set_max_time", processServerResponse);
    }

    var set_max_percentage = function(){
        syncWithServer("set_max_percentage", processServerResponse);
    }

    var syncWithServer = function (method, onSuccess) {
        /*
         * method can be "set_obs", "pull", "step"
         */
        $.ajax({
            url: "/get",
            type: "post", 
            dataType: "json", 
            contentType : "application/json",
            data: JSON.stringify({
                method: method,
                map_grid: encodeGrid(map_grid),
                obs_grid: encodeGrid(obs_grid),
                max_time: parseInt($('#max-time').val()),
                max_percentage: parseInt($('#max-percentage').val())
            }),
            success:onSuccess,
            error:function(){
                console.log("AJAX failure");
                alert("AJAX failure");
            }
        });
    }

    var refreshTimer = null;

    var connectMonitor7 = function(){
        talkWithMonitor("/connect12347");
        if(refreshTimer != null)
            clearInterval(refreshTimer);
        setInterval(getStatusFromMonitor, 500);
    }

    var connectMonitor8 = function(){
        talkWithMonitor("/connect12348");
        if(refreshTimer != null)
            clearInterval(refreshTimer);
        setInterval(getStatusFromMonitor, 500);
    }

    var getStatusFromMonitor = function(){
        talkWithMonitor("/get_status", processServerResponse);
    }

    var balanceMonitor = function(){
        talkWithMonitor("/balance");
    }

    var alignMonitor = function(){
        talkWithMonitor("/align");
    }

    var exploreMonitor = function(){
        talkWithMonitor("/explore");
    }

    var shortestMonitor = function(){
        talkWithMonitor("/shortest");
    }

    var commandMonitor = function(command){
        talkWithMonitor("/command", null, {"command": command});
    }

    var talkWithMonitor = function (url, onSuccess, data) {
        if(data == undefined || data == null){
            data = {}
        }
        data = JSON.stringify(data);
        $.ajax({
            url: url,
            type: "post", 
            dataType: "json", 
            contentType : "application/json",
            success:onSuccess,
            data: data,
            error:function(){
                alert("Monitor AJAX failure");
            }
        });
    }

    var onLoad = function () {
        resetGrids();
        displayGrids();

        if(mode === "simulator"){
            console.log("Simulator mode");
            // pullFromServer();

            var loadButton =   $("#load");
            var useButton  =   $('#use');
            var commitButton = $("#commit");
            var stepButton =   $("#step");
            var resetButton =   $("#reset");
            var setMaxTimeBtn = $("#set-max-time");
            var setMaxPercBtn = $("#set-max-percentage");
            loadButton.click(function () {
                var encoded_str = $('#encode-str').val();
                obs_grid = decodeGrid(encoded_str);
                displayGrids();
                console.log("load complete");
            });
            useButton.click(function(){
                $('#encode-str').val(JOE_MAP_STR);
            });
            commitButton.click(setObsOnServer);
            stepButton.click(function(){
                stepServer();
            });
            $(document).on("keypress", function(e) { 
                if (e.which === 115){   // s key
                    if (mode === "simulator")
                        stepServer();
                    else
                        getStatusFromMonitor();
                }
            });
            resetButton.click(function(){
                resetServer();
            });

            setMaxTimeBtn.click(set_max_time);
            setMaxPercBtn.click(set_max_percentage);

            var stepPerSecond = $("#step-per-second");
            stepPerSecond.change(function(){
                step_enabled = false;
                var steps = parseInt(stepPerSecond.val());
                step_interval = 1000 / steps;
                console.log(step_interval);
                if(isNaN(step_interval))
                    step_enabled = false;
                else{
                    step_enabled = true;
                    stepServer();
                }
                console.log(step_enabled);
            });
        }
        else if(mode === "monitor"){
            console.log("Monitor mode");

            var connect7Button = $("#c12347");
            var connect8Button = $("#c12348");
            var refreshButton =   $("#refresh");

            var balanceButton =   $("#balance");
            var alignButton =   $("#align");
            var exploreButton =   $("#explore");
            var shortestButton =   $("#shortest");

            var sendButton =   $("#send");

            connect7Button.click(connectMonitor7);
            connect8Button.click(connectMonitor8);
            refreshButton.click(getStatusFromMonitor);

            balanceButton.click(balanceMonitor);
            alignButton.click(alignMonitor);
            exploreButton.click(exploreMonitor);
            shortestButton.click(shortestMonitor);

            sendButton.click(function(){
                var command = parseInt($('#hex_command').val());
                commandMonitor(command);
            });
        }
    };

    window.onload = onLoad;
//}).call(window);
