// Auto-generated. Do not edit!

// (in-package arm_move.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class mission {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.T = null;
      this.E = null;
      this.L = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('T')) {
        this.T = initObj.T
      }
      else {
        this.T = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('E')) {
        this.E = initObj.E
      }
      else {
        this.E = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('L')) {
        this.L = initObj.L
      }
      else {
        this.L = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mission
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [T]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.T, buffer, bufferOffset);
    // Serialize message field [E]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.E, buffer, bufferOffset);
    // Serialize message field [L]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.L, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mission
    let len;
    let data = new mission(null);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [T]
    data.T = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [E]
    data.E = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [L]
    data.L = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_move/mission';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ebf46cc34619c6ab6b248306e7f7e41';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 type
    geometry_msgs/Point T
    geometry_msgs/Point E
    geometry_msgs/Point L
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
    const resolved = new mission(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.T !== undefined) {
      resolved.T = geometry_msgs.msg.Point.Resolve(msg.T)
    }
    else {
      resolved.T = new geometry_msgs.msg.Point()
    }

    if (msg.E !== undefined) {
      resolved.E = geometry_msgs.msg.Point.Resolve(msg.E)
    }
    else {
      resolved.E = new geometry_msgs.msg.Point()
    }

    if (msg.L !== undefined) {
      resolved.L = geometry_msgs.msg.Point.Resolve(msg.L)
    }
    else {
      resolved.L = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = mission;
