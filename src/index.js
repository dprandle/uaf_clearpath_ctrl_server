var express = require('express');
var app = express();
const path = require('path');
const WebSocketServer = require('ws');
const wss = new WebSocketServer.Server({ port: 3000 })

var gsockets = [];

// Creating connection using websocket
wss.on("connection", ws => {
    console.log("new client connected");
    // sending message
    ws.on("message", data => {
        console.log(`Client has sent us: ${data}`)
    });
    // handling what to do when clients disconnects from server
    ws.on("close", () => {
        console.log("the client has connected");
    });
    // handling client connection error
    ws.onerror = function () {
        console.log("Some Error occurred")
    }
});

app.get('/', function(req, res){
   res.sendFile(path.join(__dirname, 'emscripten', 'jackal_ctrl.html'));
});

app.use(express.static(path.join(__dirname, 'emscripten')));
app.listen(8080);