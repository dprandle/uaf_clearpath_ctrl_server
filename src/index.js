const express = require('express');
const path = require('path');
const wss_lib = require('ws');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const geom_msgs = rosnodejs.require('geometry_msgs').msg;

const net = require('net');

const app = express();
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;
const http_server_port = 8080;

let pub = {};

// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
    .then((ros_node) => {
        // let sub = ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
        // (data) => { // define callback execution
        //     console.log("Got updated map - size is " + data.info.width + " by " + data.info.height);
        //     send_data_on_sockets(data.info.width);
        // });
        pub = rosnodejs.nh.advertise('/cmd_vel', 'geometry_msgs/Twist');
    });


// Serve to browsers via websockets
wss.on("connection", web_sckt => {
    console.log("New client connected for web sockets");

    // sending message
    web_sckt.on("message", data => {
        const hdr = parse_command_header(data);
        if (hdr.command === 0) {
            const vel_cmd = parse_vel_cmd(data);
            const msg = new geom_msgs.Twist;
            msg.linear.x = vel_cmd.linear;
            msg.angular.z = vel_cmd.angular;
            pub.publish(msg);
        }
    });

    // handling what to do when clients disconnects from server
    web_sckt.on("close", () => {
        console.log("the client has disconnected");
    });

    // handling client connection error
    web_sckt.onerror = function () {
        console.log("Some Error occurred")
    }

    wsockets.push(web_sckt);
});


function parse_command_header(buf) {
    return {
        command: buf.readInt8(0),
        future: buf.readInt8(1),
        future1: buf.readInt8(2),
        future2: buf.readInt8(3)
    };
}

function parse_vel_cmd(buf) {
    return {
        linear: buf.readFloatLE(4),
        angular: buf.readFloatLE(8)
    };
}


// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", dt_skt => {
    console.log("Got connection from " + dt_skt.remoteAddress + " on port " + dt_skt.remotePort);
    // sending message
    dt_skt.on("data", data => {
        const hdr = parse_command_header(data);
        if (hdr.command === 0) {
            const vel_cmd = parse_vel_cmd(data);
            const msg = new geom_msgs.Twist;
            msg.linear.x = vel_cmd.linear;
            msg.angular.z = vel_cmd.angular;
            pub.publish(msg);
        }
    });

    dt_skt.on("end", data => {
        console.log(`client ended`);
    });

    dt_skt.on("error", data => {
        console.log(`Client has sent us: ${data}`)
    });

    // handling what to do when clients disconnects from server
    dt_skt.on("close", () => {
        console.log("the client has disconnected");
    });

    //dt_skt.pipe(dt_skt);
    dt_sockets.push(dt_skt);

});

// Serve up the emscripten generated main page
app.get('/', function (req, res) {
    res.sendFile(path.join(__dirname, 'emscripten', 'jackal_ctrl.html'));
});

// The html shell above refernces other files in the same dir - make all files available (must use full path relative doesn't work)
app.use(express.static(path.join(__dirname, 'emscripten')));

// List on the normal http server - and when an upgrade request comes through (for websockets protocol), then route the
// request to the above websocket handler
const main_server = app.listen(http_server_port);
main_server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, socket => {
        wss.emit('connection', socket, request);
    });
});

// Listen for non websocket connections
non_browser_server.listen(desktop_server_port);