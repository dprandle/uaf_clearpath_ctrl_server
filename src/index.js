const express = require('express');
const path = require('path');
const wss_lib = require('ws');
const rosnodejs = require('rosnodejs');
const geom_msgs = rosnodejs.require('geometry_msgs').msg;
// const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
// const nav_msgs = rosnodejs.require('nav_msgs').msg;

const net = require('net');

const app = express();
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;
const http_server_port = 8080;
const PACKET_STR_ID_LEN = 16;

let debug_print = true;
function dlog(params) {debug_print&&console.log(params);}

let warning_print = true;
function wlog(params) {warning_print&&console.log(params);}

let pub = {};
const occ_grid = [];

function send_packet_to_clients(packet) {
    for (let i = 0; i < wsockets.length; ++i)
        wsockets[i].send(packet);
    for (let i = 0; i < dt_sockets.length; ++i)
        dt_sockets[i].write(packet);
}

const scan_packet_header = {
    type: "SCAN_PCKT_ID\0\0\0\0"
};

const occ_grid_header = {
    type: "OC_GRID_PCKT_ID\0"
};

const vel_cmd_header = {
    type: "VEL_CMD_PCKT_ID\0"
};

function write_packet_header(packet_header, packet) {
    for (let i = 0; i < packet_header.type.length; ++i)
        packet.writeInt8(packet_header.type.charCodeAt(i), i);
}

function update_occ_grid_and_get_delta(cm_data)
{
    dlog(`Occ grid cur size:${cm_data.length} prev size:${occ_grid.length}`);
    let changes = [];
    changes.length = 0;
    for (let i = 0; i < cm_data.length; ++i)
    {
        if (!(i in occ_grid))
        {
            if (occ_grid.length != i)
            {
                wlog("The occ grid length is not what we expect");
            }
            occ_grid.push(cm_data[i]);
            changes.push({ind: i, val: cm_data[i]})
        }
        else if (cm_data[i] !== occ_grid[i])
        {
            occ_grid[i] = cm_data[i];
            changes.push({ind: i, val: cm_data[i]})
        }
    }
    occ_grid.length = cm_data.length;
    dlog(`Updated occ grid size:${occ_grid.length}`);
    dlog(`Got ${changes.length} changes to be sent over the wire!`);
    return changes;
}


function create_packet_from_costmap(cm) {
    
    // We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
    // since the last update - we can assume -1 (unknown) for all entries at start
    const frame_changes = update_occ_grid_and_get_delta(cm.data)

    if (frame_changes.length < 1)
    {
        dlog("Skipping sending packet - no changes found");
    }

    // Packet size is header plus map meta data (3 4 byte values) plus the origin pos and orientation
    // (total of 7 doubles which are each 8 bytes), a single 4 byte value for the length of the changes array,
    // and then the changes array where each entry is a 4 byte value - 3 MSB for the index and 1 LSB for the likelyhood value
    const packet_size = PACKET_STR_ID_LEN + 4*3 + 8*7 + 4 + frame_changes.length * 4;

    const packet = new Buffer.alloc(packet_size);
    let offset = PACKET_STR_ID_LEN;

    // Map meta data
    write_packet_header(occ_grid_header, packet);
    offset = packet.writeFloatLE(cm.info.resolution, offset);
    offset = packet.writeUInt32LE(cm.info.width, offset);
    offset = packet.writeUInt32LE(cm.info.height, offset);

    // Origin position
    offset = packet.writeDoubleLE(cm.info.origin.position.x, offset);
    offset = packet.writeDoubleLE(cm.info.origin.position.y, offset);
    offset = packet.writeDoubleLE(cm.info.origin.position.z, offset);

    // Origin orientation
    offset = packet.writeDoubleLE(cm.info.origin.orientation.x, offset);
    offset = packet.writeDoubleLE(cm.info.origin.orientation.y, offset);
    offset = packet.writeDoubleLE(cm.info.origin.orientation.z, offset);
    offset = packet.writeDoubleLE(cm.info.origin.orientation.w, offset);

    if (frame_changes.length > 16777216)
        wlog("Changes array size is larger than allowed max");
    
    // Length of changes array
    offset = packet.writeUInt32LE(frame_changes.length, offset);

    for (let i = 0; i < frame_changes.length; ++i)
    {
        // Shift index to the left by one byte and use the LSByte for the likelihood value
        // Since this is javascript, gotta manually check for -1 and convert to 255 to keep it within a byte
        let byte_val = frame_changes[i].val;
        if (byte_val == -1)
            byte_val = 255;
        
        let val = (frame_changes[i].ind << 8) + byte_val;
        offset = packet.writeInt32LE(val, offset);
    }
}

function create_packet_from_scan(scan) {
    // 20 for the 5 floats listed below, and 4 bytes per scan range float
    const packet_size = PACKET_STR_ID_LEN + 20 + scan.ranges.length * 4;
    const packet = new Buffer.alloc(packet_size);
    let offset = PACKET_STR_ID_LEN;

    write_packet_header(scan_packet_header, packet);

    offset = packet.writeFloatLE(scan.angle_min, offset);
    offset = packet.writeFloatLE(scan.angle_max, offset);
    offset = packet.writeFloatLE(scan.angle_increment, offset);
    offset = packet.writeFloatLE(scan.range_min, offset);
    offset = packet.writeFloatLE(scan.range_max, offset);
    for (let i = 0; i < scan.ranges.length; ++i)
        offset = packet.writeFloatLE(scan.ranges[i], offset);
    return packet;
}

// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
    .then((ros_node) => {
        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
            (cm) => {
                const packet = create_packet_from_costmap(cm);
                send_packet_to_clients(packet);
            });
        
        // Subscribe to the laser scan messages - send a packet with the scan info
        ros_node.subscribe("/front/scan", "sensor_msgs/LaserScan",
            (scan) => {
                const packet = create_packet_from_scan(scan);
                send_packet_to_clients(packet);
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