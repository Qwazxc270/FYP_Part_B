// Auto-generated. Do not edit!

// (in-package qcar_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ConeDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.colour = null;
      this.direction = null;
      this.range = null;
      this.position = null;
      this.position_covariance = null;
      this.bbox_x = null;
      this.bbox_y = null;
      this.bbox_w = null;
      this.bbox_h = null;
      this.pixel_count = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('colour')) {
        this.colour = initObj.colour
      }
      else {
        this.colour = 0;
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('position_covariance')) {
        this.position_covariance = initObj.position_covariance
      }
      else {
        this.position_covariance = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('bbox_x')) {
        this.bbox_x = initObj.bbox_x
      }
      else {
        this.bbox_x = 0;
      }
      if (initObj.hasOwnProperty('bbox_y')) {
        this.bbox_y = initObj.bbox_y
      }
      else {
        this.bbox_y = 0;
      }
      if (initObj.hasOwnProperty('bbox_w')) {
        this.bbox_w = initObj.bbox_w
      }
      else {
        this.bbox_w = 0;
      }
      if (initObj.hasOwnProperty('bbox_h')) {
        this.bbox_h = initObj.bbox_h
      }
      else {
        this.bbox_h = 0;
      }
      if (initObj.hasOwnProperty('pixel_count')) {
        this.pixel_count = initObj.pixel_count
      }
      else {
        this.pixel_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConeDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [colour]
    bufferOffset = _serializer.uint8(obj.colour, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.direction, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float64(obj.range, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Check that the constant length array field [position_covariance] has the right length
    if (obj.position_covariance.length !== 4) {
      throw new Error('Unable to serialize array field position_covariance - length must be 4')
    }
    // Serialize message field [position_covariance]
    bufferOffset = _arraySerializer.float64(obj.position_covariance, buffer, bufferOffset, 4);
    // Serialize message field [bbox_x]
    bufferOffset = _serializer.int32(obj.bbox_x, buffer, bufferOffset);
    // Serialize message field [bbox_y]
    bufferOffset = _serializer.int32(obj.bbox_y, buffer, bufferOffset);
    // Serialize message field [bbox_w]
    bufferOffset = _serializer.int32(obj.bbox_w, buffer, bufferOffset);
    // Serialize message field [bbox_h]
    bufferOffset = _serializer.int32(obj.bbox_h, buffer, bufferOffset);
    // Serialize message field [pixel_count]
    bufferOffset = _serializer.int32(obj.pixel_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConeDetection
    let len;
    let data = new ConeDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [colour]
    data.colour = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_covariance]
    data.position_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [bbox_x]
    data.bbox_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_y]
    data.bbox_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_w]
    data.bbox_w = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox_h]
    data.bbox_h = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pixel_count]
    data.pixel_count = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 109;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qcar_navigation/ConeDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41df40de730901088a49c65ee720b31e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 UNKNOWN = 0
    uint8 BLUE = 1
    uint8 YELLOW = 2
    uint8 ORANGE = 3
    uint8 colour
    
    geometry_msgs/Vector3 direction
    
    float64 range
    
    geometry_msgs/Point position
    
    float64[4] position_covariance
    
    int32 bbox_x
    int32 bbox_y
    int32 bbox_w
    int32 bbox_h
    
    int32 pixel_count
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConeDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.colour !== undefined) {
      resolved.colour = msg.colour;
    }
    else {
      resolved.colour = 0
    }

    if (msg.direction !== undefined) {
      resolved.direction = geometry_msgs.msg.Vector3.Resolve(msg.direction)
    }
    else {
      resolved.direction = new geometry_msgs.msg.Vector3()
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.position_covariance !== undefined) {
      resolved.position_covariance = msg.position_covariance;
    }
    else {
      resolved.position_covariance = new Array(4).fill(0)
    }

    if (msg.bbox_x !== undefined) {
      resolved.bbox_x = msg.bbox_x;
    }
    else {
      resolved.bbox_x = 0
    }

    if (msg.bbox_y !== undefined) {
      resolved.bbox_y = msg.bbox_y;
    }
    else {
      resolved.bbox_y = 0
    }

    if (msg.bbox_w !== undefined) {
      resolved.bbox_w = msg.bbox_w;
    }
    else {
      resolved.bbox_w = 0
    }

    if (msg.bbox_h !== undefined) {
      resolved.bbox_h = msg.bbox_h;
    }
    else {
      resolved.bbox_h = 0
    }

    if (msg.pixel_count !== undefined) {
      resolved.pixel_count = msg.pixel_count;
    }
    else {
      resolved.pixel_count = 0
    }

    return resolved;
    }
};

// Constants for message
ConeDetection.Constants = {
  UNKNOWN: 0,
  BLUE: 1,
  YELLOW: 2,
  ORANGE: 3,
}

module.exports = ConeDetection;
