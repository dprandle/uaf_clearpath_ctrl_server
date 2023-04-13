const express = require('express');
const path = require('path');
const rosnodejs = require('rosnodejs');
const geom_msgs = rosnodejs.require('geometry_msgs').msg;

const wss_lib = require('ws');
const os = require('os');
const http = require('http');
const { spawn } = require("child_process");

const net = require('net');
const { Buffer } = require('buffer');

const app = express();
const http_server = http.createServer(app);
const non_browser_server = net.createServer();
const wss = new wss_lib.Server({ noServer: true });

const wsockets = [];
const dt_sockets = [];
const desktop_server_port = 4000;

const http_server_port = (process.env.IS_SIMULATION === "1")?8080:80;

const PACKET_STR_ID_LEN = 32;
const TFORM_NODE_NAME_LEN = 32;
const KB_SIZE = 1024;
const MB_SIZE = 1024 * KB_SIZE;
const JACKAL_HOSTNAME = "cpr-uaf01";
const HUSKY_HOSTNAME = "cpr-uaf02-husky";
const ROBOT_HOSTNAME = os.hostname();

const IS_JACKAL = ((ROBOT_HOSTNAME === JACKAL_HOSTNAME) || (process.env.IS_JACKAL === "1"));
let NAV_PACKAGE_NAME = (IS_JACKAL)?"uaf_jackal_navigation":"uaf_husky_navigation";
const SCAN_TOPIC_NAME = ((IS_JACKAL) || (process.env.IS_SIMULATION === "1"))?"front/scan":"scan";

let jackal_cam_sub = null;
let image_requestors = [];
const PER_IMAGE_MS_DELAY = 100;
const PER_CONNECT_MS_DELAY = 100;
const IMAGE_PERIOD_CONST_MS = 150;

let cmd_vel_pub = {};
let cmd_goal_pub = {};
let cancel_goal_pub = {};

let cur_loc_navp_sub = "/move_base/TrajectoryPlannerROS/local_plan";
let cur_glob_navp_sub = "/move_base/NavfnROS/plan";

let static_tforms = {};
let frame_tforms = {};
let prev_frame_map_msg = {};
let prev_frame_glob_cm_msg = {};
let prev_frame_loc_cm_msg = {};
let current_goal_status_msg = {};

const misc_stats = {
    conn_count: 0,
    missed_packets: 0,
    cur_bw_mbps: 0,
    avg_bw_mbps: 0
};

const MISC_STAT_PERIOD_MS = 1000;
const MISC_STAT_AVG_CNT = 10;
let total_cur_data_bytes = 0;
let TS_START = Date.now();

const scan_packet_header = {
    type: "SCAN_PCKT_ID"
};

const map_header = {
    type: "MAP_PCKT_ID"
};

const loc_cm_header = {
    type: "LOC_CM_PCKT_ID"
};

const glob_cm_header = {
    type: "GLOB_CM_PCKT_ID"
};

const tform_pcket_header = {
    type: "TFORM_PCKT_ID"
};

const loc_navp_pcket_header = {
    type: "LOC_NAVP_PCKT_ID"
};

const glob_navp_pcket_header = {
    type: "GLOB_NAVP_PCKT_ID"
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

const comp_img_pckt_id = {
    type: "COMP_IMG_PCKT_ID"
}

const enable_img_cmd_id = {
    type: "ENABLE_IMG_CMD_ID"
}

const disable_img_cmd_id = {
    type: "DISABLE_IMG_CMD_ID"
}

const misc_stats_pckt_id = {
    type: "MISC_STATS_PCKT_ID"
}

const debug_print = true;
const warning_print = true;
const info_print = true;
const cm_print = false;
const cm2_print = false;
const cam_print = false;
const misc_stats_print = false;

function dlog(params) { debug_print && console.log(`${Date.now() - TS_START}: ${params}`); }
function wlog(params) { warning_print && console.log(`${Date.now() - TS_START}: ${params}`); }
function ilog(params) { info_print && console.log(`${Date.now() - TS_START}: ${params}`); }
function cm_log(params) { cm_print && console.log(`${Date.now() - TS_START},${params}`); }
function cm2_log(params) { cm2_print && console.log(`${Date.now() - TS_START},${params}`); }

// const IS_JACKAL = ((ROBOT_HOSTNAME === JACKAL_HOSTNAME) || process.env.IS_JACKAL)
// const NAV_PACKAGE_NAME = (IS_JACKAL)?"uaf_jackal_navigation":"uaf_husky_navigation";
// const SCAN_TOPIC_NAME = (IS_JACKAL || process.env.IS_SIMULATION)?"/front/scan":"/scan";
dlog(`IS_JACKAL: ${IS_JACKAL} ${process.env.IS_JACKAL}  NAV_PACKAGE_NAME: ${NAV_PACKAGE_NAME}  SCAN_TOPIC_NAME:${SCAN_TOPIC_NAME}`);

function cam_log() {
    cam_print && console.log(`${Date.now() - TS_START},${misc_stats.conn_count},${image_requestors.length},${misc_stats.cur_bw_mbps},${misc_stats.avg_bw_mbps}`);
}

function misc_stats_log() {
    misc_stats_print && console.log(`${Date.now() - TS_START},${misc_stats.conn_count},${misc_stats.missed_packets},${misc_stats.cur_bw_mbps},${misc_stats.avg_bw_mbps}`);
}

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

function send_packet_to_clients(packet, override_rate_limit = false) {
    for (let i = 0; i < wsockets.length; ++i) {
        if (wsockets[i].bufferedAmount == 0 || override_rate_limit) {
            wsockets[i].send(packet, { binary: true });
            total_cur_data_bytes += packet.length;
        } else {
            misc_stats.missed_packets += 1;
            ilog("GHEREARESRASDFASDF");
        }
    }
    for (let i = 0; i < dt_sockets.length; ++i) {
        if (dt_sockets[i].ready_for_more_data) {
            dt_sockets[i].ready_for_more_data = false;
            dt_sockets[i].write(packet, '', () => {
                dt_sockets[i].ready_for_more_data = true;
                total_cur_data_bytes += packet.length;
            });
        } else {
            ilog("Desktop GHEREARESRASDFASDF");
            misc_stats.missed_packets += 1;
        }
    }
}

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

function connected_client_count() {
    return wsockets.length + dt_sockets.length;
}

function ready_socket_count() {
    let ret = 0;
    for (let i = 0; i < wsockets.length; ++i) {
        if (wsockets[i].bufferedAmount == 0) {
            ++ret;
        }
    }
    for (let i = 0; i < dt_sockets.length; ++i) {
        if (dt_sockets[i].ready_for_more_data)
            ++ret;
    }
    return ret;
}

function at_least_one_client_ready() {
    const ready_cnt = ready_socket_count();
    if (ready_cnt < 1) {
        if (connected_client_count() > 0) {
            ilog(`Add ${connected_client_count()} to missed`);
        }
        misc_stats.missed_packets += connected_client_count();
    }
    return ready_cnt > 0;
}

function all_sockets_ready() {
    const ready_cnt = ready_socket_count();
    const conn_cnt = connected_client_count();
    misc_stats.missed_packets += (conn_cnt - ready_cnt);
    return (conn_cnt === ready_cnt);
}

function sum_elements(array)
{
    let ret = 0;
    for (let i = 0; i < array.length; ++i)
        ret += array[i];
    return ret;
}

// We don't need to send the entire costmap every update - instead just send the size of the costmap and changes
// since the last update
function get_occ_grid_delta(cur_occ_grid, prev_occ_grid) {
    if (cur_occ_grid.length !== prev_occ_grid.length) {
        ilog(`Occ grid resized from ${prev_occ_grid.length} to ${cur_occ_grid.length}`);
    }
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

// Sometimes ROS sends costmap updates for us - in this case we need to apply the updates to our stored costmap
function get_changes_and_apply_update_to_occgrid(update_msg, prev_occ_grid) {
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

function get_compressed_image_header_packet_size(comp_image_msg) {
    /// Header bytes plus one byte for format, four bytes for data size, and however many bytes are in the data
    return PACKET_STR_ID_LEN + 1 + 4;// + comp_image_msg.data.length;
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

function add_compressed_image_header_to_packet(compressed_image_msg, packet, offset) {
    offset = write_packet_header(comp_img_pckt_id, packet, offset);
    offset = packet.writeUInt8(1, offset);
    offset = packet.writeUInt32LE(compressed_image_msg.data.length, offset);
    return offset;
}

function add_navp_to_packet(navmsg, header, packet, offset) {
    offset = write_packet_header(header, packet, offset);
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
            if (goal_stat_msg.cur_goal_stamp.secs === goal_stat_msg.status_list[i].goal_id.stamp.secs)
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
    if (strlen > 0 && str[0] !== '{') {
        send_console_text_to_clients("msg: " + str);
        return;
    }
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
    // We have to override the rate limit because we are receiving costmap updates from ROS - we gotta apply these
    // to our data, and then we gotta send those changes when we do
    if (!('data' in prev_frame_og_msg)) {
        ilog("Cannot send costmap update - haven't got original costmap yet");
        return;
    }

    const frame_changes = get_changes_and_apply_update_to_occgrid(update_msg, prev_frame_og_msg);
    cm2_log(`,,${frame_changes.length*4}`);
    // Only create the packet and send it if there are clients connected
    if (connected_client_count() > 0) {
        const packet_size = get_occgrid_packet_size(frame_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(prev_frame_og_msg, frame_changes, header, 0, packet, 0);
        send_packet_to_clients(packet, true);
    }
}

function send_occ_grid_to_clients(cur_occ_grid_msg, prev_occ_grid_msg, header, override_rate_limit=false) {
    
    if (!override_rate_limit && !all_sockets_ready()) {
        return;
    }
    
    // Used to see if we need to send the packet at all
    const total_connections = connected_client_count();

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

    if (header == map_header) {
        cm2_log(`${frame_changes.length*4},,`);
    } else {
        cm2_log(`,${frame_changes.length*4},`);
    }

    // Only create the packet and send it if there are clients connected
    if (total_connections > 0) {
        const packet_size = get_occgrid_packet_size(frame_changes);
        const packet = new Buffer.alloc(packet_size);
        add_occgrid_to_packet(cur_occ_grid_msg, frame_changes, header, reset_map, packet, 0);
        dlog(`Sending occ grid packet ${JSON.stringify(header)} (${packet.length}B or ${packet.length / MB_SIZE}MB) to ${total_connections} clients`);
        send_packet_to_clients(packet, override_rate_limit);
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

function add_misc_stats_to_packet(ms, packet, offset) {
    offset = write_packet_header(misc_stats_pckt_id, packet, offset);
    offset = packet.writeUInt8(ms.conn_count, offset);
    offset = packet.writeFloatLE(ms.cur_bw_mbps, offset);
    offset = packet.writeFloatLE(ms.avg_bw_mbps, offset);
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

function get_misc_stats_packet_size() {
    return PACKET_STR_ID_LEN + 1 + 4 + 4;
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
    if (!at_least_one_client_ready()) {
        return;
    }
    const packet = new Buffer.alloc(get_transforms_packet_size(tforms));
    add_transforms_to_packet(tforms, packet, 0);
    send_packet_to_clients(packet);
}

function send_text_packet_to_clients(text, header) {
    if (!at_least_one_client_ready()) {
        return;
    }
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
    proc.stderr.on("data", data => {
        if (send_err) {
            let txt = data.toString();
            send_console_text_to_clients(txt);
            ilog(`${name} stderr (sending to clients): ${txt}`);
        }
    });

    proc.stdout.on("data", data => {
        if (send_output) {
            let txt = data.toString();
            send_console_text_to_clients(txt);
            ilog(`${name} stdout (sending to clients): ${txt}`);
        }
    });
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

        if (proc === cur_param_proc) {
            ilog("Setting cur param proc to null");
            cur_param_proc = null;
        }

        ilog(`Args length: ${args.length}`);
        if (args.length >= 6) {
            ilog(`Args[4] = ${args[4]}`);
            if (args[4] === "base_local_planner") {
                const new_topic = "/move_base/" + args[5].split("/").pop() + "/local_plan";
                const msg = `Unsubscribing from ${cur_loc_navp_sub} and subscribing to ${new_topic}`;
                send_console_text_to_clients(msg);
                ilog(msg);

                rosnodejs.nh.unsubscribe(cur_loc_navp_sub);
                cur_loc_navp_sub = new_topic;
                rosnodejs.nh.subscribe(cur_loc_navp_sub, "nav_msgs/Path", on_local_navp_msg);
            }
            else if (args[4] === "base_global_planner") {
                const new_topic = "/move_base/" + args[5].split("/").pop() + "/plan";
                const msg = `Unsubscribing from ${cur_glob_navp_sub} and subscribing to ${new_topic}`;
                send_console_text_to_clients(msg);
                ilog(msg);

                rosnodejs.nh.unsubscribe(cur_glob_navp_sub);
                cur_glob_navp_sub = new_topic;
                rosnodejs.nh.subscribe(cur_glob_navp_sub, "nav_msgs/Path", on_global_navp_msg);
            }
        }

        dlog(`${name} child process exited with code ${code}`);
    });
    return proc;
}

function on_global_navp_msg(navmsg) {
    // Totally skip if no sockets are ready
    if (!at_least_one_client_ready()) {
        return;
    }

    const packet = new Buffer.alloc(get_navp_packet_size(navmsg));
    add_navp_to_packet(navmsg, glob_navp_pcket_header, packet, 0);
    send_packet_to_clients(packet);
}

function on_local_navp_msg(navmsg) {
    if (!at_least_one_client_ready()) {
        return;
    }

    const packet = new Buffer.alloc(get_navp_packet_size(navmsg));
    add_navp_to_packet(navmsg, loc_navp_pcket_header, packet, 0);
    send_packet_to_clients(packet);
}

function run_navigation_gmapping() {
    dlog(`Running ${NAV_PACKAGE_NAME}`);
    return run_child_process("roslaunch", [NAV_PACKAGE_NAME, "gmapping.launch", "scan_topic:=" + SCAN_TOPIC_NAME], false, false);
}

function run_navigation_move_base() {
    return run_child_process("roslaunch", [NAV_PACKAGE_NAME, "move_base.launch"], true, true);
}

function update_param_check(cur_param_proc, pstack) {
    // If param items are available and we aren't waiting on another parameter
    // process to finish, process the next item.
    if (!cur_param_proc && pstack.length !== 0) {
        const pobj = pstack.shift();
        let msg = `Setting node ${pobj.node} parameter ${pobj.param_name} to ${JSON.stringify(pobj.param_val)}`;

        // Spawn a child process using dynamic_reconfigure to set the parameter for the node - this will most likely
        // kill and restart the node
        cur_param_proc = run_child_process("rosrun", ["dynamic_reconfigure", "dynparam", "set", pobj.node, pobj.param_name, pobj.param_val], true, true, true, msg);
        setTimeout(proc_timeout, 20000, cur_param_proc);
        ilog(msg);
    }
}

const avg_array = [];
const RESET_PACKET_LOSS = 10;
let cur_packet_loss_count = 0;

function update_and_send_misc_stats()
{
    misc_stats.conn_count = connected_client_count();
    misc_stats.cur_bw_mbps = (total_cur_data_bytes*8000) / (MISC_STAT_PERIOD_MS*1048576);
    total_cur_data_bytes = 0;
    
    avg_array.push(misc_stats.cur_bw_mbps);
    if (avg_array.length > MISC_STAT_AVG_CNT) {
        avg_array.shift();
    }
    misc_stats.avg_bw_mbps = sum_elements(avg_array) / avg_array.length;

    misc_stats_log();

    ++cur_packet_loss_count;
    if (cur_packet_loss_count === RESET_PACKET_LOSS) {
        cur_packet_loss_count = 0;
        misc_stats.missed_packets = 0;
    }
    

    if (!at_least_one_client_ready()) {
        return;
    }
    const packet = new Buffer.alloc(get_misc_stats_packet_size());
    add_misc_stats_to_packet(misc_stats, packet, 0);
    send_packet_to_clients(packet);
}

// Create command server ROS node wich will relay our socket communications to control and monitor the robotrosnodejs
rosnodejs.initNode("/command_server")
    .then((ros_node) => {

        gmapping_proc = run_navigation_gmapping();
        navigation_proc = run_navigation_move_base();

        setInterval(send_tforms, 30, frame_tforms);
        setInterval(update_param_check, 100, cur_param_proc, param_stack);
        setInterval(update_and_send_misc_stats, MISC_STAT_PERIOD_MS);

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/map", "nav_msgs/OccupancyGrid",
                           (occ_grid_msg) => {
                               cm_log(`${occ_grid_msg.info.height*occ_grid_msg.info.width},,`);
                               send_occ_grid_to_clients(occ_grid_msg, prev_frame_map_msg, map_header);
                               prev_frame_map_msg = occ_grid_msg;
                           });

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/move_base/global_costmap/costmap", "nav_msgs/OccupancyGrid",
                           (occ_grid_msg) => {
                               // cm_log(`global_costmap, ${occ_grid_msg.info.height*occ_grid_msg.info.width}`);
                               send_occ_grid_to_clients(occ_grid_msg, prev_frame_glob_cm_msg, glob_cm_header, true);
                               prev_frame_glob_cm_msg = occ_grid_msg;
                               ilog("Global costmap full update");
                           });

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/move_base/global_costmap/costmap_updates", "map_msgs/OccupancyGridUpdate",
                           (occ_grid_msg) => {
                               cm_log(`,${occ_grid_msg.height*occ_grid_msg.width},`);
                               send_occ_grid_from_update_to_clients(occ_grid_msg, prev_frame_glob_cm_msg, glob_cm_header);
                           });

        // Subscribe to the occupancy grid map messages - send packet with the map info
        ros_node.subscribe("/move_base/local_costmap/costmap", "nav_msgs/OccupancyGrid",
                           (occ_grid_msg) => {
                               cm_log(`,,${occ_grid_msg.info.height*occ_grid_msg.info.width}`);
                               send_occ_grid_to_clients(occ_grid_msg, prev_frame_loc_cm_msg, loc_cm_header);
                               prev_frame_loc_cm_msg = occ_grid_msg;
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

        ros_node.subscribe(cur_glob_navp_sub, "nav_msgs/Path", on_global_navp_msg);

        ros_node.subscribe(cur_loc_navp_sub, "nav_msgs/Path", on_local_navp_msg);

        // Subscribe to the laser scan messages - send a packet with the scan info
        ros_node.subscribe(SCAN_TOPIC_NAME, "sensor_msgs/LaserScan",
                           (scan) => {
                               // Totally skip if no sockets are ready
                               if (!at_least_one_client_ready()) {
                                   return;
                               }

                               const packet = new Buffer.alloc(get_scan_packet_size(scan));
                               add_scan_to_packet(scan, packet, 0);
                               send_packet_to_clients(packet);
                           }, { throttleMs: 100 });

        ros_node.subscribe("/move_base/status", "actionlib_msgs/GoalStatusArray",
                           (status_msg) => {
                               current_goal_status_msg.status_list = status_msg.status_list;

                               // Totally skip if no sockets are ready
                               if (!at_least_one_client_ready()) {
                                   return;
                               }
                               
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
    });

function proc_timeout(proc) {
    if (proc !== undefined) {
        proc['timeout'] = true;
        proc.kill('SIGINT');
    }
}

function subscribe_for_image_topic(ros_node, image_reqs) {
    const subscriber_count = image_reqs.length;
    if (subscriber_count > 0) {
        const ms_period = PER_IMAGE_MS_DELAY * subscriber_count + connected_client_count() * PER_CONNECT_MS_DELAY - IMAGE_PERIOD_CONST_MS;
        
        ilog(`Subscribing to image topic at period of ${ms_period} ms`);
        return ros_node.subscribe("/camera/left/image_color/compressed", "sensor_msgs/CompressedImage",
                                  (comp_image) => {
                                      // Totally skip if no sockets are ready
                                      if (!at_least_one_client_ready()) {
                                          return;
                                      }

                                      const packet = new Buffer.alloc(get_compressed_image_header_packet_size(comp_image));
                                      add_compressed_image_header_to_packet(comp_image, packet, 0);
                                      // Send the image header info
                                      send_packet_to_clients(packet);
                                      
                                      // Just send the data blob from the image directly - no need to copy to the array buffer packet
                                      send_packet_to_clients(comp_image.data);
                                  }, { throttleMs: ms_period });
    }
    return null;
}

function handle_image_subsriber_count_change(image_reqs) {
    if (jackal_cam_sub) {
        rosnodejs.nh.unsubscribe("/camera/left/image_color/compressed").then(function () {
            ilog(`Unsubscribed from image topic due to subscriber count change (new count ${image_reqs.length})`);
            jackal_cam_sub = subscribe_for_image_topic(rosnodejs.nh, image_reqs);
        });
    }
    else if (image_reqs.length > 0) {
        jackal_cam_sub = subscribe_for_image_topic(rosnodejs.nh, image_reqs);
    }
}

function parse_incoming_data(data, sckt) {
    total_cur_data_bytes += data.length;
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
                const navmsg = { poses: [] };
                on_global_navp_msg(navmsg);
                on_local_navp_msg(navmsg);
                cancel_goal_pub.publish(current_goal_status_msg.status_list[i].goal_id);
            }
        }
    }
    else if (hdr === enable_img_cmd_id.type) {
        image_requestors.push(sckt);
        handle_image_subsriber_count_change(image_requestors);
    }
    else if (hdr === disable_img_cmd_id.type) {
        remove_element_at_index(get_element_index(sckt, image_requestors), image_requestors);
        handle_image_subsriber_count_change(image_requestors);
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
                        if (request_count == 0) {
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
        navigation_proc.kill('SIGINT');
        gmapping_proc.kill('SIGINT');
        prev_frame_glob_cm_msg = {};
        prev_frame_map_msg = {};        
        gmapping_proc = run_navigation_gmapping();
        navigation_proc = run_navigation_move_base();
        send_console_text_to_clients("Cleared maps and restarted navigation nodes");
    }
    else if (hdr === set_params_cmd_header.type) {
        let cmd_obj = parse_param_cmd(data);
        if (Object.keys(cmd_obj).length !== 0) {
            for (const [node, params] of Object.entries(cmd_obj)) {
                for (const [param_name, param_val] of Object.entries(params)) {
                    param_stack.push({ 'node': node, 'param_name': param_name, 'param_val': param_val });
                }
            }
        }
    }
}


// Serve to browsers via websockets
wss.on("connection", web_sckt => {
    // sending message
    //web_sckt.binaryType = "arraybuffer";
    web_sckt.on("message", data => {
        parse_incoming_data(data, web_sckt);
    });

    // handling what to do when clients disconnects from server
    web_sckt.on("close", (code, reason) => {
        remove_socket_from_array(web_sckt, wsockets);
        remove_element_at_index(get_element_index(web_sckt, image_requestors), image_requestors);
        handle_image_subsriber_count_change(image_requestors);
        ilog(`Connection to ws client closed with code ${code} for reason ${reason} - ${wsockets.length} ws clients remain connected`);
    });

    // handling client connection error
    web_sckt.on("error", (error) => {
        wlog(`An error on a web socket has occured: ${error}`);
    });

    wsockets.push(web_sckt);
    web_sckt.ready_for_more_data = true;
    send_prev_occ_grid_to_new_client(prev_frame_glob_cm_msg, glob_cm_header, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    send_prev_occ_grid_to_new_client(prev_frame_map_msg, map_header, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    send_static_tforms_to_new_client(static_tforms, web_sckt, (sckt, pckt) => { sckt.send(pckt); });
    handle_image_subsriber_count_change(image_requestors);
    ilog(`New client connected for web sockets - ${wsockets.length} web sockets connected`);
});


// Serve to desktop/smartphone clients - these are not web sockets
non_browser_server.on("connection", (dt_skt) => {
    // sending message
    dt_skt.on("data", data => {
        parse_incoming_data(data, dt_skt);
    });

    dt_skt.on("error", err => {
        ilog(`Connection to non ws client ${dt_skt.remoteAddress} encountered error: ${err}`);
    });

    // handling what to do when clients disconnects from server
    dt_skt.on("close", () => {
        remove_socket_from_array(dt_skt, dt_sockets);
        remove_element_at_index(get_element_index(dt_skt, image_requestors), image_requestors);
        handle_image_subsriber_count_change(image_requestors);
        ilog(`Connection to non ws client ${dt_skt.remoteAddress}:${dt_skt.remotePort} was closed - ${dt_sockets.length} non ws clients remain connected`);
    });
    dt_sockets.push(dt_skt);
    dt_skt.ready_for_more_data = true;
    send_prev_occ_grid_to_new_client(prev_frame_glob_cm_msg, glob_cm_header, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
    send_prev_occ_grid_to_new_client(prev_frame_map_msg, map_header, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
    send_static_tforms_to_new_client(static_tforms, dt_skt, (sckt, pckt) => { sckt.write(pckt); });
    handle_image_subsriber_count_change(image_requestors);
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
    res.sendFile(path.join(__dirname, 'emscripten', 'uaf_clearpath_ctrl.html'));
});

// Serve up the emscripten generated main page
app.get('/control', function (req, res) {
    res.sendFile(path.join(__dirname, 'emscripten', 'uaf_clearpath_ctrl.html'));
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
