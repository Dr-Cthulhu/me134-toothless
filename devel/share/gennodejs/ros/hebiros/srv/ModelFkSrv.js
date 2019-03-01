// Auto-generated. Do not edit!

// (in-package hebiros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ModelFkSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.positions = null;
      this.frame_type = null;
    }
    else {
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('frame_type')) {
        this.frame_type = initObj.frame_type
      }
      else {
        this.frame_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelFkSrvRequest
    // Serialize message field [positions]
    bufferOffset = _arraySerializer.float64(obj.positions, buffer, bufferOffset, null);
    // Serialize message field [frame_type]
    bufferOffset = _serializer.int32(obj.frame_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelFkSrvRequest
    let len;
    let data = new ModelFkSrvRequest(null);
    // Deserialize message field [positions]
    data.positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [frame_type]
    data.frame_type = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.positions.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/ModelFkSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd146385f1eb1d3268bc4b0516bec43a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 FrameTypeCenterOfMass = 0
    int32 FrameTypeOutput = 1
    float64[] positions
    int32 frame_type
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelFkSrvRequest(null);
    if (msg.positions !== undefined) {
      resolved.positions = msg.positions;
    }
    else {
      resolved.positions = []
    }

    if (msg.frame_type !== undefined) {
      resolved.frame_type = msg.frame_type;
    }
    else {
      resolved.frame_type = 0
    }

    return resolved;
    }
};

// Constants for message
ModelFkSrvRequest.Constants = {
  FRAMETYPECENTEROFMASS: 0,
  FRAMETYPEOUTPUT: 1,
}

class ModelFkSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frames = null;
    }
    else {
      if (initObj.hasOwnProperty('frames')) {
        this.frames = initObj.frames
      }
      else {
        this.frames = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelFkSrvResponse
    // Serialize message field [frames]
    bufferOffset = _arraySerializer.float64(obj.frames, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelFkSrvResponse
    let len;
    let data = new ModelFkSrvResponse(null);
    // Deserialize message field [frames]
    data.frames = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.frames.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/ModelFkSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2dbcb8d5fc4b1d0fa7d1af4a1a7dc9b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] frames
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelFkSrvResponse(null);
    if (msg.frames !== undefined) {
      resolved.frames = msg.frames;
    }
    else {
      resolved.frames = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ModelFkSrvRequest,
  Response: ModelFkSrvResponse,
  md5sum() { return '0a7e86c595699ff4b039262ca7d90490'; },
  datatype() { return 'hebiros/ModelFkSrv'; }
};
