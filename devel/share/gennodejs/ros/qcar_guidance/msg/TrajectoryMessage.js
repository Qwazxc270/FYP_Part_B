// Auto-generated. Do not edit!

// (in-package qcar_guidance.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TrajectoryMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoint_times = null;
      this.waypoint_x = null;
      this.waypoint_y = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoint_times')) {
        this.waypoint_times = initObj.waypoint_times
      }
      else {
        this.waypoint_times = [];
      }
      if (initObj.hasOwnProperty('waypoint_x')) {
        this.waypoint_x = initObj.waypoint_x
      }
      else {
        this.waypoint_x = [];
      }
      if (initObj.hasOwnProperty('waypoint_y')) {
        this.waypoint_y = initObj.waypoint_y
      }
      else {
        this.waypoint_y = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryMessage
    // Serialize message field [waypoint_times]
    bufferOffset = _arraySerializer.float64(obj.waypoint_times, buffer, bufferOffset, null);
    // Serialize message field [waypoint_x]
    bufferOffset = _arraySerializer.float64(obj.waypoint_x, buffer, bufferOffset, null);
    // Serialize message field [waypoint_y]
    bufferOffset = _arraySerializer.float64(obj.waypoint_y, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryMessage
    let len;
    let data = new TrajectoryMessage(null);
    // Deserialize message field [waypoint_times]
    data.waypoint_times = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [waypoint_x]
    data.waypoint_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [waypoint_y]
    data.waypoint_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.waypoint_times.length;
    length += 8 * object.waypoint_x.length;
    length += 8 * object.waypoint_y.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qcar_guidance/TrajectoryMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3970ebd2c3a27d3351680b715e22ec1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] waypoint_times
    float64[] waypoint_x
    float64[] waypoint_y
    float64 velocity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryMessage(null);
    if (msg.waypoint_times !== undefined) {
      resolved.waypoint_times = msg.waypoint_times;
    }
    else {
      resolved.waypoint_times = []
    }

    if (msg.waypoint_x !== undefined) {
      resolved.waypoint_x = msg.waypoint_x;
    }
    else {
      resolved.waypoint_x = []
    }

    if (msg.waypoint_y !== undefined) {
      resolved.waypoint_y = msg.waypoint_y;
    }
    else {
      resolved.waypoint_y = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    return resolved;
    }
};

module.exports = TrajectoryMessage;
