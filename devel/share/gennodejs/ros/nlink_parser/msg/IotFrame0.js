// Auto-generated. Do not edit!

// (in-package nlink_parser.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IotFrame0Node = require('./IotFrame0Node.js');

//-----------------------------------------------------------

class IotFrame0 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uid = null;
      this.nodes = null;
    }
    else {
      if (initObj.hasOwnProperty('uid')) {
        this.uid = initObj.uid
      }
      else {
        this.uid = 0;
      }
      if (initObj.hasOwnProperty('nodes')) {
        this.nodes = initObj.nodes
      }
      else {
        this.nodes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IotFrame0
    // Serialize message field [uid]
    bufferOffset = _serializer.uint32(obj.uid, buffer, bufferOffset);
    // Serialize message field [nodes]
    // Serialize the length for message field [nodes]
    bufferOffset = _serializer.uint32(obj.nodes.length, buffer, bufferOffset);
    obj.nodes.forEach((val) => {
      bufferOffset = IotFrame0Node.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IotFrame0
    let len;
    let data = new IotFrame0(null);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [nodes]
    // Deserialize array length for message field [nodes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodes[i] = IotFrame0Node.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 17 * object.nodes.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nlink_parser/IotFrame0';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5942449595b10adf20faeffdc7ac5270';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 uid
    IotFrame0Node[] nodes
    
    ================================================================================
    MSG: nlink_parser/IotFrame0Node
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
    const resolved = new IotFrame0(null);
    if (msg.uid !== undefined) {
      resolved.uid = msg.uid;
    }
    else {
      resolved.uid = 0
    }

    if (msg.nodes !== undefined) {
      resolved.nodes = new Array(msg.nodes.length);
      for (let i = 0; i < resolved.nodes.length; ++i) {
        resolved.nodes[i] = IotFrame0Node.Resolve(msg.nodes[i]);
      }
    }
    else {
      resolved.nodes = []
    }

    return resolved;
    }
};

module.exports = IotFrame0;
