// Auto-generated. Do not edit!

// (in-package nlink_parser.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class IotFrame0Node {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uid = null;
      this.cnt = null;
      this.dis = null;
      this.aoa_angle_horizontal = null;
      this.aoa_angle_vertical = null;
    }
    else {
      if (initObj.hasOwnProperty('uid')) {
        this.uid = initObj.uid
      }
      else {
        this.uid = 0;
      }
      if (initObj.hasOwnProperty('cnt')) {
        this.cnt = initObj.cnt
      }
      else {
        this.cnt = 0;
      }
      if (initObj.hasOwnProperty('dis')) {
        this.dis = initObj.dis
      }
      else {
        this.dis = 0.0;
      }
      if (initObj.hasOwnProperty('aoa_angle_horizontal')) {
        this.aoa_angle_horizontal = initObj.aoa_angle_horizontal
      }
      else {
        this.aoa_angle_horizontal = 0.0;
      }
      if (initObj.hasOwnProperty('aoa_angle_vertical')) {
        this.aoa_angle_vertical = initObj.aoa_angle_vertical
      }
      else {
        this.aoa_angle_vertical = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IotFrame0Node
    // Serialize message field [uid]
    bufferOffset = _serializer.uint32(obj.uid, buffer, bufferOffset);
    // Serialize message field [cnt]
    bufferOffset = _serializer.uint8(obj.cnt, buffer, bufferOffset);
    // Serialize message field [dis]
    bufferOffset = _serializer.float32(obj.dis, buffer, bufferOffset);
    // Serialize message field [aoa_angle_horizontal]
    bufferOffset = _serializer.float32(obj.aoa_angle_horizontal, buffer, bufferOffset);
    // Serialize message field [aoa_angle_vertical]
    bufferOffset = _serializer.float32(obj.aoa_angle_vertical, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IotFrame0Node
    let len;
    let data = new IotFrame0Node(null);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cnt]
    data.cnt = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dis]
    data.dis = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [aoa_angle_horizontal]
    data.aoa_angle_horizontal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [aoa_angle_vertical]
    data.aoa_angle_vertical = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nlink_parser/IotFrame0Node';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18c74fd1a18895f584bf54c3563d0a60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 uid
    uint8 cnt
    float32 dis
    float32 aoa_angle_horizontal
    float32 aoa_angle_vertical
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IotFrame0Node(null);
    if (msg.uid !== undefined) {
      resolved.uid = msg.uid;
    }
    else {
      resolved.uid = 0
    }

    if (msg.cnt !== undefined) {
      resolved.cnt = msg.cnt;
    }
    else {
      resolved.cnt = 0
    }

    if (msg.dis !== undefined) {
      resolved.dis = msg.dis;
    }
    else {
      resolved.dis = 0.0
    }

    if (msg.aoa_angle_horizontal !== undefined) {
      resolved.aoa_angle_horizontal = msg.aoa_angle_horizontal;
    }
    else {
      resolved.aoa_angle_horizontal = 0.0
    }

    if (msg.aoa_angle_vertical !== undefined) {
      resolved.aoa_angle_vertical = msg.aoa_angle_vertical;
    }
    else {
      resolved.aoa_angle_vertical = 0.0
    }

    return resolved;
    }
};

module.exports = IotFrame0Node;
