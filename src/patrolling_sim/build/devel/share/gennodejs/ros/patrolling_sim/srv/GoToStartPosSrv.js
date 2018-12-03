// Auto-generated. Do not edit!

// (in-package patrolling_sim.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GoToStartPosSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.teamsize = null;
      this.sleep_between_goals = null;
    }
    else {
      if (initObj.hasOwnProperty('teamsize')) {
        this.teamsize = initObj.teamsize
      }
      else {
        this.teamsize = new std_msgs.msg.UInt8();
      }
      if (initObj.hasOwnProperty('sleep_between_goals')) {
        this.sleep_between_goals = initObj.sleep_between_goals
      }
      else {
        this.sleep_between_goals = new std_msgs.msg.UInt8();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoToStartPosSrvRequest
    // Serialize message field [teamsize]
    bufferOffset = std_msgs.msg.UInt8.serialize(obj.teamsize, buffer, bufferOffset);
    // Serialize message field [sleep_between_goals]
    bufferOffset = std_msgs.msg.UInt8.serialize(obj.sleep_between_goals, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToStartPosSrvRequest
    let len;
    let data = new GoToStartPosSrvRequest(null);
    // Deserialize message field [teamsize]
    data.teamsize = std_msgs.msg.UInt8.deserialize(buffer, bufferOffset);
    // Deserialize message field [sleep_between_goals]
    data.sleep_between_goals = std_msgs.msg.UInt8.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'patrolling_sim/GoToStartPosSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46245284711564dca6f0ea10c19a6573';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/UInt8 teamsize
    std_msgs/UInt8 sleep_between_goals
    
    ================================================================================
    MSG: std_msgs/UInt8
    uint8 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoToStartPosSrvRequest(null);
    if (msg.teamsize !== undefined) {
      resolved.teamsize = std_msgs.msg.UInt8.Resolve(msg.teamsize)
    }
    else {
      resolved.teamsize = new std_msgs.msg.UInt8()
    }

    if (msg.sleep_between_goals !== undefined) {
      resolved.sleep_between_goals = std_msgs.msg.UInt8.Resolve(msg.sleep_between_goals)
    }
    else {
      resolved.sleep_between_goals = new std_msgs.msg.UInt8()
    }

    return resolved;
    }
};

class GoToStartPosSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoToStartPosSrvResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToStartPosSrvResponse
    let len;
    let data = new GoToStartPosSrvResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'patrolling_sim/GoToStartPosSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoToStartPosSrvResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: GoToStartPosSrvRequest,
  Response: GoToStartPosSrvResponse,
  md5sum() { return '46245284711564dca6f0ea10c19a6573'; },
  datatype() { return 'patrolling_sim/GoToStartPosSrv'; }
};
