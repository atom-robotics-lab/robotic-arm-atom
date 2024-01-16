// Auto-generated. Do not edit!

// (in-package ajgar_core.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class tfValueSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tfArray = null;
    }
    else {
      if (initObj.hasOwnProperty('tfArray')) {
        this.tfArray = initObj.tfArray
      }
      else {
        this.tfArray = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tfValueSrvRequest
    // Serialize message field [tfArray]
    bufferOffset = _arraySerializer.float32(obj.tfArray, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tfValueSrvRequest
    let len;
    let data = new tfValueSrvRequest(null);
    // Deserialize message field [tfArray]
    data.tfArray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.tfArray.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_core/tfValueSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c8b8c86b312e11339447f3b6046d437b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] tfArray 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tfValueSrvRequest(null);
    if (msg.tfArray !== undefined) {
      resolved.tfArray = msg.tfArray;
    }
    else {
      resolved.tfArray = []
    }

    return resolved;
    }
};

class tfValueSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reachedGoal = null;
    }
    else {
      if (initObj.hasOwnProperty('reachedGoal')) {
        this.reachedGoal = initObj.reachedGoal
      }
      else {
        this.reachedGoal = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tfValueSrvResponse
    // Serialize message field [reachedGoal]
    bufferOffset = _serializer.bool(obj.reachedGoal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tfValueSrvResponse
    let len;
    let data = new tfValueSrvResponse(null);
    // Deserialize message field [reachedGoal]
    data.reachedGoal = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_core/tfValueSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81492924865f72c18a6f4bdd91cb99e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool reachedGoal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tfValueSrvResponse(null);
    if (msg.reachedGoal !== undefined) {
      resolved.reachedGoal = msg.reachedGoal;
    }
    else {
      resolved.reachedGoal = false
    }

    return resolved;
    }
};

module.exports = {
  Request: tfValueSrvRequest,
  Response: tfValueSrvResponse,
  md5sum() { return 'e2d3842affcc45c41842273c7f827da7'; },
  datatype() { return 'ajgar_core/tfValueSrv'; }
};
