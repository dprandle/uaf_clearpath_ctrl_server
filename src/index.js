const express = require('express');
const path = require('path');
const wss_lib = require('ws');
const rosnodejs = require('rosnodejs');
const geom_msgs = rosnodejs.require('geometry_msgs').msg;
// const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
// const nav_msgs = rosnodejs.require('nav_msgs').msg;

const net = require('net');
const { Buffer } = require('buffer');

const app = express();
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;
const http_server_port = 8080;

const PACKET_STR_ID_LEN = 16;
const TFORM_NODE_NAME_LEN = 32;
const KB_SIZE = 1024;
const MB_SIZE = 1024 * KB_SIZE;

let debug_print = true;
let warning_print = true;
let info_print = true;
function dlog(params) { debug_print && console.log(params); }
function wlog(params) { warning_print && console.log(params); }
function ilog(params) { info_print && console.log(params); }

let pub = {};
let static_tforms = {};
let frame_tforms = {};
let prev_frame_occ_grid_msg = [];

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
    type: "SCAN_PCKT_ID"
};

const occ_grid_header = {
    type: "OC_GRID_PCKT_ID"
};

const tform_pcket_header = {
    type: "TFORM_PCKT_ID"
};

const vel_cmd_header = {
    type: "VEL_CMD_PCKT_ID"
};


function write_packet_string(str, len, packet, offset) {
    for (let i = 0; i < len; ++i)
    {
        if (i < str.length)
            packet.writeInt8(str.charCodeAt(i), offset+i);
        else
            packet.writeInt8(0, offset+i);
    }
    return offset+len;
}

function write_packet_header(packet_header, packet, offset) {
    return write_packet_string(packet_header.type, PACKET_STR_ID_LEN, packet, offset);
}

function get_occ_grid_delta(cur_occ_grid, prev_occ_grid) {
    dlog(`Occ grid cur size:${cur_occ_grid.length} prev size:${prev_occ_grid.length}`);
    let changes = [];
    changes.length = 0;
    for (let i = 0; i < cur_occ_grid.length; ++i) {

        if (!(i in prev_occ_grid) && (cur_occ_grid[i] !== -1)) {
            changes.push({ ind: i, val: cur_occ_grid[i] })
        }
        else if ((i in prev_occ_grid) && cur_occ_grid[i] !== prev_occ_grid[i]) {
            changes.push({ ind: i, val: cur_occ_grid[i] })
        }
    }
    return changes;
}

function get_occgrid_packet_size(frame_changes)
{
    // Packet size is header plus map meta data (3 4 byte values) plus the origin pos and orientation
    // (total of 7 doubles which are each 8 bytes), a single 4 byte value for the length of the changes array,
    // and then the changes array where each entry is a 4 byte value - 3 MSB for the index and 1 LSB for the likelyhood value
    return PACKET_STR_ID_LEN + 4 * 3 + 8 * 7 + 4 + frame_changes.length * 4;
}

function add_occgrid_to_packet(occ_grid_msg, frame_changes, packet, offset) {

    // Map meta data
    offset = write_packet_header(occ_grid_header, packet, offset);

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
    
    // Length of changes array
    dlog(`Sending frame_changes length ${frame_changes.length}`);
    offset = packet.writeUInt32LE(frame_changes.length, offset);

    for (let i = 0; i < frame_changes.length; ++i) {

        console.assert((frame_changes[i].val < 16777216), "Changes array size is larger than allowed max");
            // Shift index to the left by one byte and use the LSByte for the likelihood value
        // Since this is javascript, gotta manually check for -1 and convert to 255 to keep it within a byte
        let byte_val = frame_changes[i].val;
        if (byte_val == -1)
            byte_val = 255;

        let val = (frame_changes[i].ind << 8) + byte_val;
        offset = packet.writeUInt32LE(val, offset);
    }
    return offset;
}

function add_scan_to_packet(scan, packet, offset) {
    offset = write_packet_header(scan_packet_header, packet, offset);
    offset = packet.writeFloatLE(scan.angle_min, offset);
    offset = packet.writeFloatLE(scan.angle_max, offset);
    offset = packet.writeFloatLE(scan.angle_increment, offset);
    offset = packet.writeFloatLE(scan.range_min, offset);
    offset = packet.writeFloatLE(scan.range_max, offset);
    for (let i = 0; i < scan.ranges.length; ++i)
        offset = packet.writeFloatLE(scan.ranges[i], offset);
    return offset;
}

function parse_command_header(buf) {
    // Remove the null bytes from the end for comparison to our header stringsS
    return buf.toString('utf8', 0, PACKET_STR_ID_LEN).replace(/\0/g, '');  
}

function parse_vel_cmd(buf) {
    return {
        linear: buf.readFloatLE(PACKET_STR_ID_LEN),
        angular: buf.readFloatLE(PACKET_STR_ID_LEN + 4)
    };
}

function send_occ_grid_to_clients(cur_occ_grid_msg, prev_occ_grid_msg) {
    
    // Used to see if we need to send the packet at all
    const total_connections = wsockets.length + dt_sockets.length;

    // If we haven't received any messages yet there will be no data member in prev_occ_grid_msg
    let occ_data = [];
    if ('data' in prev_occ_grid_msg)
        occ_data = prev_occ_grid_msg.data;
    
    // We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
    // since the last update    
    const frame_changes = get_occ_grid_delta(cur_occ_grid_msg.data, occ_data);
    
    // Only create the packet and send it if there are clients connected
    if (total_connections > 0)
    {
        const packet_size = get_occgrid_packet_size(frame_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(cur_occ_grid_msg, frame_changes, packet, 0);
        dlog(`Sending occ grid packet (${packet.length}B or ${packet.length / MB_SIZE}MB) to ${total_connections} clients`);
        send_packet_to_clients(packet);
    }
}

/// When a new connection is established
function send_prev_occ_grid_to_new_client(occ_grid_msg, socket, func_to_use) {
    if ('data' in occ_grid_msg)
    {
        const frame_all_changes = get_occ_grid_delta(occ_grid_msg.data, []);
        const packet_size = get_occgrid_packet_size(frame_all_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(occ_grid_msg, frame_all_changes, packet, 0);
        dlog(`Sending occ grid packet (${packet.length} bytes) to newly connected client`);
        func_to_use(socket, packet);
    }
    else
    {
        dlog("No occ grid message stored from ROS to send to newly connected client");
    }
}

function add_transform_to_packet(tf, packet, offset)
{    
    offset = write_packet_header(tform_pcket_header, packet, offset);
    offset = write_packet_string(tf.header.frame_id, TFORM_NODE_NAME_LEN, packet, offset);
    offset = write_packet_string(tf.child_frame_id, TFORM_NODE_NAME_LEN, packet, offset);

    // Transform position
    offset = packet.writeDoubleLE(tf.transform.translation.x, offset);
    offset = packet.writeDoubleLE(tf.transform.translation.y, offset);
    offset = packet.writeDoubleLE(tf.transform.translation.z, offset);

    // Transform orientation
    offset = packet.writeDoubleLE(tf.transform.rotation.x, offset);
    offset = packet.writeDoubleLE(tf.transform.rotation.y, offset);
    offset = packet.writeDoubleLE(tf.transform.rotation.z, offset);
    offset = packet.writeDoubleLE(tf.transform.rotation.w, offset);
    return offset;
}

function convert_tforms_key_val(tf_message, converted)
{
    for (let i = 0; i < tf_message.transforms.length; ++i){
        converted[tf_message.transforms[i].child_frame_id] = tf_message.transforms[i];
    }
}

function add_transforms_to_packet(tforms, packet, offset)
{
    for (const [key, value] of Object.entries(tforms)) {
        offset = add_transform_to_packet(value, packet, offset);
    }
    return offset;
}

function send_static_tforms_to_new_client(tforms, socket, func_to_use)
{
    const item_count = Object.keys(tforms).length;
    if (item_count !== 0)
    {
        let packet_size = get_transforms_packet_size(tforms);
        let offset = 0;
        const packet = new Buffer.alloc(packet_size);
        add_transforms_to_packet(tforms, packet, offset);
        func_to_use(socket, packet);
        dlog(`Sent ${item_count} static transforms (${packet.length} bytes) to newly connected client`);
    }
    else
    {
        dlog("No static transforms stored from ROS to send to newly connected client");
    }
}

function get_scan_packet_size(scan_msg)
{
    return PACKET_STR_ID_LEN + 20 + scan_msg.ranges.length * 4;
}

function get_transform_packet_size()
{
    return PACKET_STR_ID_LEN + 32 + 32 + 8*3 + 8*4;
}

function get_transforms_packet_size(tforms)
{
    return get_transform_packet_size() * Object.keys(tforms).length;
}

// Create command server ROS node wich will relay our socket communications to control and monitor the robot
rosnodejs.initNode("/command_server")
    .then((ros_node) => {
        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
            (occ_grid_msg) => {
                send_occ_grid_to_clients(occ_grid_msg, prev_frame_occ_grid_msg);
                prev_frame_occ_grid_msg = occ_grid_msg;
            });

        // Subscribe to the transform topic - send packets for each transform
        ros_node.subscribe("/tf", "tf2_msgs/TFMessage",
        (tf_message) => {
            convert_tforms_key_val(tf_message, frame_tforms);
        });

        ros_node.subscribe("/tf_static", "tf2_msgs/TFMessage",
        (tf_message) => {
            convert_tforms_key_val(tf_message, static_tforms);
        });

        // Subscribe to the laser scan messages - send a packet with the scan info
        ros_node.subscribe("/front/scan", "sensor_msgs/LaserScan",
            (scan) => {
                let packet_size = get_transforms_packet_size(frame_tforms) + get_scan_packet_size(scan);
                const packet = new Buffer.alloc(packet_size);
                let offset = add_scan_to_packet(scan, packet, 0);
                offset = add_transforms_to_packet(frame_tforms, packet, offset);
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
        ilog(`Connection to ws client closed with code ${code} for reason ${reason} - ${wsockets.length} ws clients remain connected`);
    });

    // handling client connection error
    web_sckt.on("error", (error) => {
        wlog(`An error on a web socket has occured: ${error}`);
    });

    wsockets.push(web_sckt);
    send_prev_occ_grid_to_new_client(prev_frame_occ_grid_msg, web_sckt, (sckt, pckt) => {sckt.send(pckt);});
    send_static_tforms_to_new_client(static_tforms, web_sckt, (sckt, pckt) => {sckt.send(pckt);});
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
        ilog(`Connection to non ws client ${dt_skt.remoteAddress}:${dt_skt.remotePort} was closed - ${dt_sockets.length} non ws clients remain connected`);
    });

    dt_sockets.push(dt_skt);
    send_prev_occ_grid_to_new_client(prev_frame_occ_grid_msg, dt_skt, (sckt, pckt) => {sckt.write(pckt);});
    send_static_tforms_to_new_client(static_tforms, dt_skt, (sckt, pckt) => {sckt.write(pckt);});
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
