// Auto-generated. Do not edit!

// (in-package dsr_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let LogAlarm = require('../msg/LogAlarm.js');

//-----------------------------------------------------------

class GetLastAlarmRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLastAlarmRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLastAlarmRequest
    let len;
    let data = new GetLastAlarmRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetLastAlarmRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_last_alarm
    ###Struct.LOG_ARARM
    #____________________________________________________________________________________________
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLastAlarmRequest(null);
    return resolved;
    }
};

class GetLastAlarmResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.log_alarm = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('log_alarm')) {
        this.log_alarm = initObj.log_alarm
      }
      else {
        this.log_alarm = new LogAlarm();
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLastAlarmResponse
    // Serialize message field [log_alarm]
    bufferOffset = LogAlarm.serialize(obj.log_alarm, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLastAlarmResponse
    let len;
    let data = new GetLastAlarmResponse(null);
    // Deserialize message field [log_alarm]
    data.log_alarm = LogAlarm.deserialize(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += LogAlarm.getMessageSize(object.log_alarm);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetLastAlarmResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a6d300dc8ba91bb8020a696fecebd6f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    LogAlarm log_alarm
    bool        success
    
    ================================================================================
    MSG: dsr_msgs/LogAlarm
    #____________________________________________________________________________________________
    # log of alarm
    #____________________________________________________________________________________________
    
    int32         level
    int32         group
    int32         index
    string[3]     param
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLastAlarmResponse(null);
    if (msg.log_alarm !== undefined) {
      resolved.log_alarm = LogAlarm.Resolve(msg.log_alarm)
    }
    else {
      resolved.log_alarm = new LogAlarm()
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetLastAlarmRequest,
  Response: GetLastAlarmResponse,
  md5sum() { return 'a6d300dc8ba91bb8020a696fecebd6f2'; },
  datatype() { return 'dsr_msgs/GetLastAlarm'; }
};
