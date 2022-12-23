const express = require('express');
const path = require('path');
const rosnodejs = require('rosnodejs');
const geom_msgs = rosnodejs.require('geometry_msgs').msg;

const wss_lib = require('ws');
const http = require('http');
const { spawn } = require("child_process");

const net = require('net');
const { Buffer } = require('buffer');

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;
const http_server_port = 8080;

const app = express();
const http_server = http.createServer(app);
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const PACKET_STR_ID_LEN = 32;
const TFORM_NODE_NAME_LEN = 32;
const KB_SIZE = 1024;
const MB_SIZE = 1024 * KB_SIZE;

let debug_print = true;
let warning_print = true;
let info_print = true;
function dlog(params) { debug_print && console.log(params); }
function wlog(params) { warning_print && console.log(params); }
function ilog(params) { info_print && console.log(params); }

let cmd_vel_pub = {};
let cmd_goal_pub = {};
let clear_map_client = {};
let cancel_goal_pub = {};

let static_tforms = {};
let frame_tforms = {};
let prev_frame_map_msg = {};
let prev_frame_glob_cm_msg = {};
let current_goal_status_msg = {};

function get_element_index(value, array) {
    return array.findIndex((element) => { return value === element });
}

function remove_element_at_index(ind, array) {
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
        wsockets[i].send(packet, { binary: true });
    for (let i = 0; i < dt_sockets.length; ++i)
        dt_sockets[i].write(packet);
}

const scan_packet_header = {
    type: "SCAN_PCKT_ID"
};

const map_header = {
    type: "MAP_PCKT_ID"
};

const glob_cm_header = {
    type: "GLOB_CM_PCKT_ID"
};

const tform_pcket_header = {
    type: "TFORM_PCKT_ID"
};

const navp_pcket_header = {
    type: "NAVP_PCKT_ID"
};

const goal_status_pckt_header = {
    type: "GOAL_STAT_PCKT_ID"
};

const vel_cmd_header = {
    type: "VEL_CMD_PCKT_ID"
};

const goal_cmd_header = {
    type: "GOAL_CMD_PCKT_ID"
};

const stop_cmd_header = {
    type: "STOP_CMD_PCKT_ID"
};

const clear_maps_cmd_header = {
    type: "CLEAR_MAPS_PCKT_ID"
};

const set_params_cmd_header = {
    type: "SET_PARAMS_CMD_PCKT_ID"
};

const set_params_response_cmd_header = {
    type: "SET_PARAMS_RESP_CMD_PCKT_ID"
};

const get_params_response_cmd_header = {
    type: "GET_PARAMS_RESP_CMD_PCKT_ID"
};

const get_params_cmd_header = {
    type: "GET_PARAMS_CMD_PCKT_ID"
};


function write_packet_string(str, len, packet, offset) {
    for (let i = 0; i < len; ++i) {
        if (i < str.length)
            packet.writeInt8(str.charCodeAt(i), offset + i);
        else
            packet.writeInt8(0, offset + i);
    }
    return offset + len;
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

function get_changes_and_apply_update_to_occgrid(update_msg, prev_occ_grid) {

    // We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
    // since the last update
    let changes = [];
    for (let y = 0; y < update_msg.height; ++y) {
        for (let x = 0; x < update_msg.width; ++x) {
            let full_map_x = update_msg.x + x;
            let full_map_y = update_msg.y + y;
            if ('data' in prev_occ_grid && 'info' in prev_occ_grid && full_map_x < prev_occ_grid.info.width && full_map_y < prev_occ_grid.info.height) {
                let update_ind = y * update_msg.width + x;
                let full_map_update_ind = full_map_y * prev_occ_grid.info.width + full_map_x;
                if (prev_occ_grid.data[full_map_update_ind] !== update_msg.data[update_ind]) {
                    prev_occ_grid.data[full_map_update_ind] = update_msg.data[update_ind];
                    changes.push({ ind: full_map_update_ind, val: update_msg.data[update_ind] });
                }
            }
        }
    }
    return changes;
}

function get_occgrid_packet_size(frame_changes) {
    // Packet size is header plus map meta data (3 4 byte values) plus the origin pos and orientation plus a reset flag
    // (total of 7 doubles which are each 8 bytes), a single 4 byte value for the length of the changes array,
    // and then the changes array where each entry is a 4 byte value - 3 MSB for the index and 1 LSB for the likelyhood value
    return PACKET_STR_ID_LEN + 4 * 3 + 8 * 7 + 1 + 4 + frame_changes.length * 4;
}

function add_occgrid_to_packet(occ_grid_msg, frame_changes, header, reset_map, packet, offset) {

    // Map meta data
    offset = write_packet_header(header, packet, offset);

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

    offset = packet.writeInt8(reset_map, offset);

    // Length of changes array
    //dlog(`Sending frame_changes length ${frame_changes.length}`);
    offset = packet.writeUInt32LE(frame_changes.length, offset);

    for (let i = 0; i < frame_changes.length; ++i) {

        console.assert((frame_changes[i].val < 16777216), "Changes array size is larger than allowed max");

        // Shift index to the left by one byte and use the LSByte for the likelihood value
        // Since this is javascript, gotta manually check for -1 and convert to 255 to keep it within a byte
        let byte_val = frame_changes[i].val;
        if (byte_val == -1)
            byte_val = 255;

        let val = (frame_changes[i].ind << 8) | byte_val;
        offset = packet.writeInt32LE(val, offset);
    }
    return offset;
}

function add_text_block_to_packet(text, packet, offset, header) {
    offset = write_packet_header(header, packet, offset);
    offset = packet.writeUInt32LE(text.length, offset);
    offset = packet.write(text, offset, text.length);
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

function add_navp_to_packet(navmsg, packet, offset) {
    offset = write_packet_header(navp_pcket_header, packet, offset);
    offset = packet.writeUInt32LE(navmsg.poses.length, offset);
    for (let i = 0; i < navmsg.poses.length; ++i) {
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.position.x, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.position.y, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.position.z, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.orientation.x, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.orientation.y, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.orientation.z, offset);
        offset = packet.writeDoubleLE(navmsg.poses[i].pose.orientation.w, offset);
    }
    return offset;
}

function add_goal_stat_to_packet(goal_stat_msg, packet, offset) {
    let status = -1;

    if ('cur_goal_stamp' in goal_stat_msg) {
        for (let i = 0; i < goal_stat_msg.status_list.length; ++i) {
            if (goal_stat_msg.cur_goal_stamp.secs === goal_stat_msg.status_list[i].goal_id.stamp.secs && goal_stat_msg.cur_goal_stamp.nsecs === goal_stat_msg.status_list[i].goal_id.stamp.nsecs)
                status = goal_stat_msg.status_list[i].status;
        }
    }

    offset = write_packet_header(goal_status_pckt_header, packet, offset);
    offset = packet.writeInt32LE(status, offset);
    let position = { x: 0, y: 0, z: 0 };
    let orientation = { x: 0, y: 0, z: 0, w: 1 };

    if ('cur_goal' in goal_stat_msg) {
        position = goal_stat_msg.cur_goal.target_pose.pose.position;
        orientation = goal_stat_msg.cur_goal.target_pose.pose.orientation;
    }

    // Pose position
    offset = packet.writeDoubleLE(position.x, offset);
    offset = packet.writeDoubleLE(position.y, offset);
    offset = packet.writeDoubleLE(position.z, offset);

    // Pose orientation
    offset = packet.writeDoubleLE(orientation.x, offset);
    offset = packet.writeDoubleLE(orientation.y, offset);
    offset = packet.writeDoubleLE(orientation.z, offset);
    offset = packet.writeDoubleLE(orientation.w, offset);

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

function recursively_apply_commands(cur_obj, parent_str_key, cmd_obj) {
    for (const [key, value] of Object.entries(cur_obj)) {
        const this_key = parent_str_key + key;
        if (typeof value === 'object' && value !== null)
            recursively_apply_commands(value, this_key + "/", cmd_obj);
        else
            cmd_obj[this_key] = value;
    }
}

function parse_param_cmd(buf) {
    let strlen = buf.readUInt32LE(PACKET_STR_ID_LEN);
    let str = buf.toString('utf8', PACKET_STR_ID_LEN + 4, PACKET_STR_ID_LEN + 4 + strlen);
    let ret = {};
    let jsobj = {};
    try {
        jsobj = JSON.parse(str);
    } catch (e) {
        let text = `${str} is invalid JSON: ${e}`;
        send_console_text_to_clients(text);
        ilog(text);
    }
    for (const [key, value] of Object.entries(jsobj)) {
        const cmd_obj = {}
        recursively_apply_commands(value, "", cmd_obj);
        ret[key] = cmd_obj;
    }
    return ret;
}

function parse_goal_cmd(buf) {
    let dbl_size = 8;
    let pos = {
        x: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 0),
        y: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 1),
        z: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 2)
    };
    let orientation = {
        x: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 3),
        y: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 4),
        z: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 5),
        w: buf.readDoubleLE(PACKET_STR_ID_LEN + dbl_size * 6),
    };
    return {
        pos: pos,
        orient: orientation
    };
}

function send_occ_grid_from_update_to_clients(update_msg, prev_frame_og_msg, header) {

    // Used to see if we need to send the packet at all
    const total_connections = wsockets.length + dt_sockets.length;

    if (!('data' in prev_frame_og_msg)) {
        ilog("Cannot send costmap update - haven't got original costmap yet");
        return;
    }

    const frame_changes = get_changes_and_apply_update_to_occgrid(update_msg, prev_frame_og_msg);

    // Only create the packet and send it if there are clients connected
    if (total_connections > 0) {
        const packet_size = get_occgrid_packet_size(frame_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(prev_frame_og_msg, frame_changes, header, 0, packet, 0);
        //dlog(`Sending occ grid packet ${JSON.stringify(header)} (${packet.length}B or ${packet.length / MB_SIZE}MB) to ${total_connections} clients`);
        send_packet_to_clients(packet);
    }
}

function send_occ_grid_to_clients(cur_occ_grid_msg, prev_occ_grid_msg, header) {

    // Used to see if we need to send the packet at all
    const total_connections = wsockets.length + dt_sockets.length;

    // If we haven't received any messages yet there will be no data member in prev_occ_grid_msg
    let reset_map = 1;
    let occ_data = [];
    if ('data' in prev_occ_grid_msg && 'info' in prev_occ_grid_msg && prev_occ_grid_msg.info.width === cur_occ_grid_msg.info.width && prev_occ_grid_msg.info.height === cur_occ_grid_msg.info.height && prev_occ_grid_msg.info.resolution === cur_occ_grid_msg.info.resolution) {
        occ_data = prev_occ_grid_msg.data;
        reset_map = 0;
    }

    // We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
    // since the last update    
    const frame_changes = get_occ_grid_delta(cur_occ_grid_msg.data, occ_data);

    // Only create the packet and send it if there are clients connected
    if (total_connections > 0) {
        const packet_size = get_occgrid_packet_size(frame_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(cur_occ_grid_msg, frame_changes, header, reset_map, packet, 0);
        dlog(`Sending occ grid packet ${JSON.stringify(header)} (${packet.length}B or ${packet.length / MB_SIZE}MB) to ${total_connections} clients`);
        send_packet_to_clients(packet);
    }
}

/// When a new connection is established
function send_prev_occ_grid_to_new_client(occ_grid_msg, header, socket, func_to_use) {
    if ('data' in occ_grid_msg) {
        const frame_all_changes = get_occ_grid_delta(occ_grid_msg.data, []);
        const packet_size = get_occgrid_packet_size(frame_all_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(occ_grid_msg, frame_all_changes, header, 0, packet, 0);
        dlog(`Sending occ grid packet ${JSON.stringify(header)} (${packet.length} bytes) to newly connected client`);
        func_to_use(socket, packet);
    }
    else {
        dlog("No occ grid message stored from ROS to send to newly connected client");
    }
}

function add_transform_to_packet(tf, packet, offset) {
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

function convert_tforms_key_val(tf_message, converted) {
    for (let i = 0; i < tf_message.transforms.length; ++i) {
        converted[tf_message.transforms[i].child_frame_id] = tf_message.transforms[i];
    }
}

function add_transforms_to_packet(tforms, packet, offset) {
    for (const [key, value] of Object.entries(tforms)) {
        offset = add_transform_to_packet(value, packet, offset);
    }
    return offset;
}

function send_static_tforms_to_new_client(tforms, socket, func_to_use) {
    const item_count = Object.keys(tforms).length;
    if (item_count !== 0) {
        let packet_size = get_transforms_packet_size(tforms);
        let offset = 0;
        const packet = new Buffer.alloc(packet_size);
        add_transforms_to_packet(tforms, packet, offset);
        func_to_use(socket, packet);
        dlog(`Sent ${item_count} static transforms (${packet.length} bytes) to newly connected client`);
    }
    else {
        dlog("No static transforms stored from ROS to send to newly connected client");
    }
}

function get_navp_packet_size(navmsg) {
    // 4 bytes for poses array size, then each pose is 7 doubles of 8 bytes each (3 for position and 4 for orientation)
    return PACKET_STR_ID_LEN + 4 + navmsg.poses.length * 7 * 8;
}

function get_goal_status_packet_size() {
    // Packet id str and then 4 bytes for the goal status, then 7 8-byte doubles for the pose
    return PACKET_STR_ID_LEN + 4 + 7 * 8;
}


function get_scan_packet_size(scan_msg) {
    return PACKET_STR_ID_LEN + 20 + scan_msg.ranges.length * 4;
}

function get_transform_packet_size() {
    return PACKET_STR_ID_LEN + 32 + 32 + 8 * 3 + 8 * 4;
}

function get_transforms_packet_size(tforms) {
    return get_transform_packet_size() * Object.keys(tforms).length;
}

function get_text_block_packet_size(text_to_send) {
    return PACKET_STR_ID_LEN + 4 + text_to_send.length;
}

function send_tforms(tforms) {
    const packet = new Buffer.alloc(get_transforms_packet_size(tforms));
    add_transforms_to_packet(tforms, packet, 0);
    send_packet_to_clients(packet);
}

function send_text_packet_to_clients(text, header)
{
    let offset = 0;
    let packet_size = get_text_block_packet_size(text);
    const packet = new Buffer.alloc(packet_size);
    add_text_block_to_packet(text, packet, offset, header);
    send_packet_to_clients(packet);
}

function send_console_text_to_clients(text) {
    send_text_packet_to_clients(text, set_params_response_cmd_header);
}

function send_param_get_text_to_clients(text) {
    send_text_packet_to_clients(text, get_params_response_cmd_header);
}

let cur_param_proc = null;
const param_stack = [];

function run_child_process(name, args, send_err = false, send_output = false, send_success_confirm = false, success_confirm_text = '') {
    const proc = spawn(name, args);
    if (send_err) {
        proc.stderr.on("data", data => {
            let txt = data.toString();
            send_console_text_to_clients(txt);
            ilog(`${name} stderr (sending to clients): ${txt}`);
        });
    }
    if (send_output) {
        proc.stdout.on("data", data => {
            let txt = data.toString();
            send_console_text_to_clients(txt);
            ilog(`${name} stdout (sending to clients): ${txt}`);
        });
    }
    proc.on('error', (error) => {
        ilog(`${name} error: ${error.message}`);
    });

    proc.on("close", code => {
        if (send_success_confirm) {
            let fmsg = ' succeeded';
            if (code !== 0)
                fmsg = ` failed with code ${code}`;
            if ('timeout' in proc)
                fmsg = ' failed - timeout';

            let text = success_confirm_text + fmsg;
            if (success_confirm_text.length === 0)
                text = `Command ${name} with args ${JSON.stringify(args)} ${fmsg}`;

            send_console_text_to_clients(text);
        }
        if (proc === cur_param_proc)
        {
            cur_param_proc = null;
        }
        //ilog(`${name} child process exited with code ${code}`);
    });
    return proc;
}

let gmapping_proc = {};
let jackal_nav_proc = {};

//
function run_gmapping() {
    return run_child_process("rosrun", ["gmapping", "slam_gmapping", "scan:=front/scan", "_delta:=0.05", "_xmin:=-1", "_xmax:=1", "_ymin:=-1", "_ymax:=1"]);
}

function run_jackal_navigation() {
    return run_child_process("roslaunch", ["jackal_navigation", "move_base.launch"]);
}

function update_param_check(cur_param_proc, pstack)
{
    if (!cur_param_proc && pstack.length !== 0)
    {
        const pobj = pstack.shift();
        let msg = `Setting node ${pobj.node} parameter ${pobj.param_name} to ${JSON.stringify(pobj.param_val)}`;
        cur_param_proc = run_child_process("rosrun", ["dynamic_reconfigure", "dynparam", "set", pobj.node, pobj.param_name, pobj.param_val], true, true, true, msg);
        setTimeout(proc_timeout, 20000, cur_param_proc);
        ilog(msg);
    }
}

setInterval(send_tforms, 30, update_param_check);

// Create command server ROS node wich will relay our socket communications to control and monitor the robotrosnodejs
rosnodejs.initNode("/command_server")
    .then((ros_node) => {

        gmapping_proc = run_gmapping();
        jackal_nav_proc = run_jackal_navigation();

        setInterval(send_tforms, 30, frame_tforms);
        setInterval(update_param_check, 30, cur_param_proc, param_stack);

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
            (occ_grid_msg) => {
                send_occ_grid_to_clients(occ_grid_msg, prev_frame_map_msg, map_header);
                prev_frame_map_msg = occ_grid_msg;
            });

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/move_base/global_costmap/costmap", "nav_msgs/OccupancyGrid",
            (occ_grid_msg) => {
                send_occ_grid_to_clients(occ_grid_msg, prev_frame_glob_cm_msg, glob_cm_header);
                prev_frame_glob_cm_msg = occ_grid_msg;
                ilog("Got Costmap full update");
            });

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/move_base/global_costmap/costmap_updates", "map_msgs/OccupancyGridUpdate",
            (occ_grid_msg) => {
                send_occ_grid_from_update_to_clients(occ_grid_msg, prev_frame_glob_cm_msg, glob_cm_header);
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

        ros_node.subscribe("/move_base/NavfnROS/plan", "nav_msgs/Path",
            (navmsg) => {
                const packet = new Buffer.alloc(get_navp_packet_size(navmsg));
                add_navp_to_packet(navmsg, packet, 0);
                send_packet_to_clients(packet);
            });

        // Subscribe to the laser scan messages - send a packet with the scan info
        ros_node.subscribe("/front/scan", "sensor_msgs/LaserScan",
            (scan) => {
                const packet = new Buffer.alloc(get_scan_packet_size(scan));
                add_scan_to_packet(scan, packet, 0);
                send_packet_to_clients(packet);
            });

        ros_node.subscribe("/move_base/status", "actionlib_msgs/GoalStatusArray",
            (status_msg) => {
                current_goal_status_msg.status_list = status_msg.status_list;
                const packet = new Buffer.alloc(get_goal_status_packet_size());
                add_goal_stat_to_packet(current_goal_status_msg, packet, 0);
                send_packet_to_clients(packet);
            });

        ros_node.subscribe("move_base/goal", "move_base_msgs/MoveBaseActionGoal",
            (goal_msg) => {
                current_goal_status_msg.cur_goal = goal_msg.goal;
                current_goal_status_msg.cur_goal_stamp = goal_msg.header.stamp;
            });

        cmd_vel_pub = rosnodejs.nh.advertise('/cmd_vel', 'geometry_msgs/Twist');
        cmd_goal_pub = rosnodejs.nh.advertise('/move_base_simple/goal', 'geometry_msgs/PoseStamped');
        cancel_goal_pub = rosnodejs.nh.advertise('/move_base/cancel', 'actionlib_msgs/GoalID');
        clear_map_client = rosnodejs.nh.serviceClient('/move_base/clear_costmaps', 'std_srvs/Empty');
    });

function proc_timeout(proc) {
    if (proc !== undefined) {
        proc['timeout'] = true;
        proc.kill('SIGINT');
    }
}

function parse_incoming_data(data) {
    const hdr = parse_command_header(data);
    if (hdr === vel_cmd_header.type) {
        const vel_cmd = parse_vel_cmd(data);
        const msg = new geom_msgs.Twist;
        msg.linear.x = vel_cmd.linear * 1.4;
        msg.angular.z = vel_cmd.angular * 1.4;
        cmd_vel_pub.publish(msg);
    }
    else if (hdr === goal_cmd_header.type) {
        const goal_cmd = parse_goal_cmd(data);
        const msg = new geom_msgs.PoseStamped;
        msg.pose.position = goal_cmd.pos;
        msg.pose.orientation = goal_cmd.orient;
        msg.header.frame_id = 'map';
        cmd_goal_pub.publish(msg);
    }
    else if (hdr === stop_cmd_header.type) {
        ilog(`Got stop command`);
        if (!('status_list' in current_goal_status_msg)) {
            ilog("No status list built received yet from ROS");
            return;
        }
        for (let i = 0; i < current_goal_status_msg.status_list.length; ++i) {
            if (current_goal_status_msg.status_list[i].status <= 1) {
                cancel_goal_pub.publish(current_goal_status_msg.status_list[i].goal_id);
            }
        }
    }
    else if (hdr == get_params_cmd_header.type) {
        const proc = spawn("rosrun", ["dynamic_reconfigure", "dynparam", "list"]);

        proc.stderr.on("data", data => {
            const txt = data.toString();
            ilog(`dynamic_reconfigure stderr (sending to clients): ${txt}`);
        });

        proc.stdout.on("data", data => {
            const txt = data.toString();
            arr = txt.split('\n');
            const main_obj = {};

            let request_count = 0;
            for (let i = 0; i < arr.length; ++i) {
                if (arr[i].length !== 0) {
                    ++request_count;
                    ilog(`Spawning child process for node ${arr[i]} (${request_count} requests)`);
                    const pget_proc = spawn("rosrun", ["dynamic_reconfigure", "dynparam", "get", arr[i]]);
                    const obj_key = arr[i].slice(1);

                    pget_proc.stderr.on("data", data => {
                        const txt = data.toString();
                        ilog(`dynamic_reconfigure get ${arr[i]} stderr (sending to clients): ${txt}`);
                    });

                    pget_proc.on('error', (error) => {
                        ilog(`dynamic_reconfigure get ${arr[i]} error: ${error.message}`);
                    });
            
                    pget_proc.on("close", code => {
                        --request_count;
                        if (request_count == 0)
                        {
                            const txt_to_send = JSON.stringify(main_obj, null, 2);
                            ilog(`Got final params for ${Object.keys(main_obj).length} nodes - sending to clients`);
                            send_param_get_text_to_clients(txt_to_send);
                        }
                        ilog(`dynamic_reconfigure get ${arr[i]} child process exited with code ${code} (${request_count} requests remain)`);
                    });

                    pget_proc.stdout.on("data", data => {
                        const txt = data.toString();
                        const txt_swapped = txt.replace(/'/g, '"').replace(/True/g, 'true').replace(/False/g, 'false');
                        main_obj[obj_key] = JSON.parse(txt_swapped);
                        delete main_obj[obj_key]['groups'];
                    });
                }
            }
        });

        proc.on('error', (error) => {
            ilog(`dynamic_reconfigure error: ${error.message}`);
        });

        proc.on("close", code => {
            ilog(`dynamic_reconfigure child process exited with code ${code}`);
        });

        setTimeout(proc_timeout, 3000, proc);
    }
    else if (hdr === clear_maps_cmd_header.type) {
        ilog(`Got clear maps command`);
        gmapping_proc.kill('SIGINT');
        jackal_nav_proc.kill('SIGINT');
        prev_frame_glob_cm_msg = {};
        prev_frame_map_msg = {};
        clear_map_client.call();
        gmapping_proc = run_gmapping();
        jackal_nav_proc = run_jackal_navigation();
        send_console_text_to_clients("Restarted jackal_navigation and gmapping");
    }
    else if (hdr === set_params_cmd_header.type) {
        let cmd_obj = parse_param_cmd(data);
        for (const [node, params] of Object.entries(cmd_obj)) {
            for (const [param_name, param_val] of Object.entries(params)) {
                param_stack.push({'node': node, 'param_name': param_name, 'param_val': param_val});
            }
        }
    }
}


// Serve to browsers via websockets
wss.on("connection", web_sckt => {
    // sending message
    web_sckt.on("message", data => {
        parse_incoming_data(data);
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
    send_prev_occ_grid_to_new_client(prev_frame_glob_cm_msg, glob_cm_header, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    send_prev_occ_grid_to_new_client(prev_frame_map_msg, map_header, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    send_static_tforms_to_new_client(static_tforms, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    ilog(`New client connected for web sockets - ${wsockets.length} web sockets connected`);
});


// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", (dt_skt) => {
    // sending message
    dt_skt.on("data", data => {
        parse_incoming_data(data);
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
    send_prev_occ_grid_to_new_client(prev_frame_glob_cm_msg, glob_cm_header, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
    send_prev_occ_grid_to_new_client(prev_frame_map_msg, map_header, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
    send_static_tforms_to_new_client(static_tforms, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
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


http_server.on('upgrade', (request, socket, head) => {
    wss.handleUpgrade(request, socket, head, socket => {
        wss.emit('connection', socket, request);
    });
});

http_server.listen(http_server_port);

// Listen for non websocket connections
non_browser_server.listen(desktop_server_port);
