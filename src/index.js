const express = require('express');
const path = require('path');
const wss_lib = require('ws');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const net = require('node:net');

const app = express();
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });
const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 3000;
const http_server_port = 8080;

// Go through all sockets for web and non web and send out data
function send_data_on_sockets(data)
{
    for (let i = 0; i < wsockets.length; ++i)
        wsockets[i].send(data);
    for (let i = 0; i < dt_sockets.length; ++i)
        dt_sockets[i].send(data);
}

function setup_socket(sckt)
{
    console.log("new client connected");

    // sending message
    sckt.on("message", data => {
        console.log(`Client has sent us: ${data}`)
    });

    // handling what to do when clients disconnects from server
    sckt.on("close", () => {
        console.log("the client has disconnected");
    });

    // handling client connection error
    sckt.onerror = function () {
        console.log("Some Error occurred")
    }
}


// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
.then((ros_node) => {  
    let sub = ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
    (data) => { // define callback execution
        console.log("Got updated map - size is " + data.info.width + " by " + data.info.height);
        send_data_on_sockets(data.info.width);
    });
});


// Serve to browsers via websockets
wss.on("connection", web_sckt => {
    setup_socket(web_sckt, wsockets);
    wsockets.push(web_sckt);
});

// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", dt_skt => {
    setup_socket(dt_skt);
    dt_sockets.push(dt_skt);
});

// Serve up the emscripten generated main page
app.get('/', function(req, res){
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