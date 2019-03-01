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

class AddModelFromURDFSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model_name = null;
      this.description_param = null;
    }
    else {
      if (initObj.hasOwnProperty('model_name')) {
        this.model_name = initObj.model_name
      }
      else {
        this.model_name = '';
      }
      if (initObj.hasOwnProperty('description_param')) {
        this.description_param = initObj.description_param
      }
      else {
        this.description_param = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddModelFromURDFSrvRequest
    // Serialize message field [model_name]
    bufferOffset = _serializer.string(obj.model_name, buffer, bufferOffset);
    // Serialize message field [description_param]
    bufferOffset = _serializer.string(obj.description_param, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddModelFromURDFSrvRequest
    let len;
    let data = new AddModelFromURDFSrvRequest(null);
    // Deserialize message field [model_name]
    data.model_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [description_param]
    data.description_param = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.model_name.length;
    length += object.description_param.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/AddModelFromURDFSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7503667ca5355c5a12e4de26d45dcf94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string model_name
    
    string description_param
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddModelFromURDFSrvRequest(null);
    if (msg.model_name !== undefined) {
      resolved.model_name = msg.model_name;
    }
    else {
      resolved.model_name = ''
    }

    if (msg.description_param !== undefined) {
      resolved.description_param = msg.description_param;
    }
    else {
      resolved.description_param = ''
    }

    return resolved;
    }
};

class AddModelFromURDFSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddModelFromURDFSrvResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddModelFromURDFSrvResponse
    let len;
    let data = new AddModelFromURDFSrvResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/AddModelFromURDFSrvResponse';
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
    const resolved = new AddModelFromURDFSrvResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: AddModelFromURDFSrvRequest,
  Response: AddModelFromURDFSrvResponse,
  md5sum() { return '7503667ca5355c5a12e4de26d45dcf94'; },
  datatype() { return 'hebiros/AddModelFromURDFSrv'; }
};
