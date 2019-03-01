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

class AddGroupFromURDFSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.group_name = null;
    }
    else {
      if (initObj.hasOwnProperty('group_name')) {
        this.group_name = initObj.group_name
      }
      else {
        this.group_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddGroupFromURDFSrvRequest
    // Serialize message field [group_name]
    bufferOffset = _serializer.string(obj.group_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddGroupFromURDFSrvRequest
    let len;
    let data = new AddGroupFromURDFSrvRequest(null);
    // Deserialize message field [group_name]
    data.group_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.group_name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/AddGroupFromURDFSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '967d0b0c0d858ded8a6a69abbce0c981';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string group_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddGroupFromURDFSrvRequest(null);
    if (msg.group_name !== undefined) {
      resolved.group_name = msg.group_name;
    }
    else {
      resolved.group_name = ''
    }

    return resolved;
    }
};

class AddGroupFromURDFSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddGroupFromURDFSrvResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddGroupFromURDFSrvResponse
    let len;
    let data = new AddGroupFromURDFSrvResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebiros/AddGroupFromURDFSrvResponse';
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
    const resolved = new AddGroupFromURDFSrvResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: AddGroupFromURDFSrvRequest,
  Response: AddGroupFromURDFSrvResponse,
  md5sum() { return '967d0b0c0d858ded8a6a69abbce0c981'; },
  datatype() { return 'hebiros/AddGroupFromURDFSrv'; }
};
