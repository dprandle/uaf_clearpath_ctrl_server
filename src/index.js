const express = require('express');
const path = require('path');
const wss_lib = require('ws');
const rosnodejs = require('rosnodejs');
const geom_msgs = rosnodejs.require('geometry_msgs').msg;
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;

const net = require('net');

const app = express();
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;
const http_server_port = 8080;
const PACKET_STR_ID_LEN = 16;

let pub = {};
let scan_sent_count = 0;

function send_packet_to_clients(packet) {
    for (let i = 0; i < wsockets.length; ++i)
        wsockets[i].send(packet);
    for (let i = 0; i < dt_sockets.length; ++i)
        dt_sockets[i].write(packet);
}

const scan_packet_header = {
    type: "SCAN_PCKT_ID\0\0\0\0"
};

const vel_cmd_header = {
    type: "VEL_CMD_PCKT_ID\0"
};

function write_packet_header(packet_header, packet) {
    for (let i = 0; i < scan_packet_header.type.length; ++i)
        packet.writeInt8(scan_packet_header.type.charCodeAt(i), i);
}

function create_packet_from_scan(scan) {
    // 20 for the 5 floats listed below, and 4 bytes per scan range float
    const packet_size = PACKET_STR_ID_LEN + 20 + scan.ranges.length * 4;
    const packet = new Buffer.alloc(packet_size);

    write_packet_header(scan_packet_header, packet);

    packet.writeFloatLE(scan.angle_min, PACKET_STR_ID_LEN);
    packet.writeFloatLE(scan.angle_max, PACKET_STR_ID_LEN + 4);
    packet.writeFloatLE(scan.angle_increment, PACKET_STR_ID_LEN + 8);
    packet.writeFloatLE(scan.range_min, PACKET_STR_ID_LEN + 12);
    packet.writeFloatLE(scan.range_max, PACKET_STR_ID_LEN + 16);
    for (let i = 0; i < scan.ranges.length; ++i)
        packet.writeFloatLE(scan.ranges[i], PACKET_STR_ID_LEN + 20 + 4 * i);
    return packet;
}

// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
    .then((ros_node) => {
        let sub = ros_node.subscribe("/front/scan", "sensor_msgs/LaserScan",
            (scan) => {
                const packet = create_packet_from_scan(scan);
                send_packet_to_clients(packet);
                ++scan_sent_count;
            });
        pub = rosnodejs.nh.advertise('/cmd_vel', 'geometry_msgs/Twist');
    });


// Serve to browsers via websockets
wss.on("connection", web_sckt => {
    console.log("New client connected for web sockets");

    // sending message
    web_sckt.on("message", data => {
        const hdr = parse_command_header(data);
        if (hdr === vel_cmd_header.type) {
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
    return buf.toString('utf8', 0, PACKET_STR_ID_LEN);
}

function parse_vel_cmd(buf) {
    return {
        linear: buf.readFloatLE(PACKET_STR_ID_LEN),
        angular: buf.readFloatLE(PACKET_STR_ID_LEN + 4)
    };
}


// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", (dt_skt) => {
    console.log("Got connection from " + dt_skt.remoteAddress + " on port " + dt_skt.remotePort);
    // sending message
    dt_skt.on("data", data => {
        const hdr = parse_command_header(data);
        if (hdr === vel_cmd_header.type) {
            const vel_cmd = parse_vel_cmd(data);
            const msg = new geom_msgs.Twist;
            msg.linear.x = vel_cmd.linear * 1.4;
            msg.angular.z = vel_cmd.angular * 1.4;
            pub.publish(msg);
        }
    });

    dt_skt.on("error", err => {
        console.log("Connection with non browser client " + dt_skt.remoteAddress + " encountered error: " + err);
    });

    // handling what to do when clients disconnects from server
    dt_skt.on("close", () => {
        console.log("Connection to non browser " + dt_skt.remoteAddress + " was closed");
    });

    dt_sockets.push(dt_skt);

});

non_browser_server.on("error", (err) => {
    console.log("Non browser server on " + non_browser_server.address() + " encountered error: " + err);
});

non_browser_server.on("listening", () => {
    console.log("Non browser server listening on " + non_browser_server.address());
})

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