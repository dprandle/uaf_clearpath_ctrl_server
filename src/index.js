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
let warning_print = true;
let info_print = true;
function dlog(params) { debug_print && console.log(params); }
function wlog(params) { warning_print && console.log(params); }
function ilog(params) { info_print && console.log(params); }

let pub = {};
const prev_frame_occ_grid = [];
const connections_needing_full_occgrid = [];

function get_element_index(value, array) {
    return array.findIndex((element) => { return value === element });
}

function remove_element_at_index(ind, array)
{
    const last_ind = array.length - 1;
    if (ind < 0)
        return;

    if (ind !== last_ind)
        array[ind] = array[last_ind];
    array.pop();
}

function remove_socket_from_array(sckt, array) {
    remove_element_at_index(get_element_index(sckt, array), array);
}

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

function update_occ_grid_and_get_delta(cur_occ_grid, prev_occ_grid) {
    dlog(`Occ grid cur size:${cur_occ_grid.length} prev size:${prev_occ_grid.length}`);
    let changes = [];
    changes.length = 0;
    for (let i = 0; i < cur_occ_grid.length; ++i) {
        if (!(i in prev_occ_grid)) {
            if (prev_occ_grid.length != i) {
                wlog("The occ grid length is not what we expect");
            }
            prev_occ_grid.push(cur_occ_grid[i]);
            changes.push({ ind: i, val: cur_occ_grid[i] })
        }
        else if (cur_occ_grid[i] !== prev_occ_grid[i]) {
            prev_occ_grid[i] = cur_occ_grid[i];
            changes.push({ ind: i, val: cur_occ_grid[i] })
        }
    }
    prev_occ_grid.length = cur_occ_grid.length;
    dlog(`Got ${changes.length} changes this frame!`);
    return changes;
}

function create_packet_from_occgrid(occ_grid_msg, frame_changes) {
    // Packet size is header plus map meta data (3 4 byte values) plus the origin pos and orientation
    // (total of 7 doubles which are each 8 bytes), a single 4 byte value for the length of the changes array,
    // and then the changes array where each entry is a 4 byte value - 3 MSB for the index and 1 LSB for the likelyhood value
    const packet_size = PACKET_STR_ID_LEN + 4 * 3 + 8 * 7 + 4 + frame_changes.length * 4;

    const packet = new Buffer.alloc(packet_size);
    let offset = PACKET_STR_ID_LEN;

    // Map meta data
    write_packet_header(occ_grid_header, packet);

    offset = packet.writeFloatLE(occ_grid_msg.info.resolution, offset);
    offset = packet.writeUInt32LE(occ_grid_msg.info.width, offset);
    offset = packet.writeUInt32LE(occ_grid_msg.info.height, offset);
    
    // Origin position
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.position.x, offset);
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.position.y, offset);
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.position.z, offset);

    // Origin orientation
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.orientation.x, offset);
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.orientation.y, offset);
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.orientation.z, offset);
    offset = packet.writeDoubleLE(occ_grid_msg.info.origin.orientation.w, offset);

    if (frame_changes.length > 16777216)
        wlog("Changes array size is larger than allowed max");
    
        // Length of changes array
    dlog(`Sending frame_changes length ${frame_changes.length}`);
    offset = packet.writeUInt32LE(frame_changes.length, offset);

    dlog(`Sending message info ${JSON.stringify(occ_grid_msg.info)}`);

    for (let i = 0; i < frame_changes.length; ++i) {
        // Shift index to the left by one byte and use the LSByte for the likelihood value
        // Since this is javascript, gotta manually check for -1 and convert to 255 to keep it within a byte
        let byte_val = frame_changes[i].val;
        if (byte_val == -1)
            byte_val = 255;

        let val = (frame_changes[i].ind << 8) + byte_val;
        offset = packet.writeUInt32LE(val, offset);
    }
    return packet;
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

function parse_command_header(buf) {
    return buf.toString('utf8', 0, PACKET_STR_ID_LEN);
}

function parse_vel_cmd(buf) {
    return {
        linear: buf.readFloatLE(PACKET_STR_ID_LEN),
        angular: buf.readFloatLE(PACKET_STR_ID_LEN + 4)
    };
}

function send_occ_grid_to_all_clients(socket_arr, normal_frame_packet, full_frame_packet, func_to_use_for_send)
{
    for (let i = 0; i < socket_arr.length; ++i) {
        let packet_to_send = normal_frame_packet;

        // If the socket is a new connection
        let ind = get_element_index(socket_arr[i], connections_needing_full_occgrid);
        if (ind >= 0)
        {
            dlog(`New connection sending full frame)`);
            packet_to_send = full_frame_packet;
            remove_element_at_index(ind, connections_needing_full_occgrid);
        }
        dlog(`Sending occ grid packet (${packet_to_send.length} bytes)`);
        func_to_use_for_send(socket_arr[i], packet_to_send);
    }
}

function send_occ_grid_to_clients(occ_grid_msg) {
    const total_connections = wsockets.length + dt_sockets.length;
    dlog(`Total connection count ${total_connections} and new connection count ${connections_needing_full_occgrid.length}`);

    // If there is a newly established connection/s, we need to make a packet for them that has the whole costmap
    let full_frame_packet = null;
    if (connections_needing_full_occgrid.length > 0)
    {
        const frame_all_changes = update_occ_grid_and_get_delta(occ_grid_msg.data, []);
        full_frame_packet = create_packet_from_occgrid(occ_grid_msg, frame_all_changes);
    }

    // We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
    // since the last update (for new connections - the above is needed as we can't just send the delta)
    const frame_changes = update_occ_grid_and_get_delta(occ_grid_msg.data, prev_frame_occ_grid);
    
    // Only create the packet if it will be sent to someone
    let normal_frame_packet = null;    
    //dlog(`Normal frame packet ${normal_frame_packet}`);
    if (total_connections > connections_needing_full_occgrid.length)
    {
        normal_frame_packet = create_packet_from_occgrid(occ_grid_msg, frame_changes);
    }

    send_occ_grid_to_all_clients(wsockets, normal_frame_packet, full_frame_packet, (sckt, pckt) => {sckt.send(pckt);});
    send_occ_grid_to_all_clients(dt_sockets, normal_frame_packet, full_frame_packet, (sckt, pckt) => {sckt.write(pckt);});
}

// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
    .then((ros_node) => {
        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
            (occ_grid_msg) => {
                send_occ_grid_to_clients(occ_grid_msg);
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
    web_sckt.on("close", (code, reason) => {
        remove_socket_from_array(web_sckt, wsockets);

        // Just in case a connection was added/removed super quickly
        remove_socket_from_array(web_sckt, connections_needing_full_occgrid);
        ilog(`Connection to ws client closed with code ${code} for reason ${reason} - ${wsockets.length} ws clients remain connected`);
    });

    // handling client connection error
    web_sckt.on("error", (error) => {
        wlog(`An error on a web socket has occured: ${error}`);
    });

    wsockets.push(web_sckt);
    connections_needing_full_occgrid.push(web_sckt);
    ilog(`New client connected for web sockets - ${wsockets.length} web sockets connected`);

});

// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", (dt_skt) => {
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
        ilog(`Connection to non ws client ${dt_skt.remoteAddress} encountered error: ${err}`);
    });

    // handling what to do when clients disconnects from server
    dt_skt.on("close", () => {
        remove_socket_from_array(dt_skt, dt_sockets);

        // Just in case a connection was added/removed super quickly
        remove_socket_from_array(dt_skt, connections_needing_full_occgrid);

        ilog(`Connection to non ws client ${dt_skt.remoteAddress}:${dt_skt.remotePort} was closed - ${dt_sockets.length} non ws clients remain connected`);
    });

    dt_sockets.push(dt_skt);
    connections_needing_full_occgrid.push(dt_skt);
    ilog(`Client connected from ${dt_skt.remoteAddress}:${dt_skt.remotePort} - ${dt_sockets.length} non ws clients connected`);

});

non_browser_server.on("error", (err) => {
    wlog(`Non browser server on ${non_browser_server.address()} encountered error: ${err}`);
});

non_browser_server.on("listening", () => {
    ilog(`Non browser server listening on ${non_browser_server.address()}`);
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
