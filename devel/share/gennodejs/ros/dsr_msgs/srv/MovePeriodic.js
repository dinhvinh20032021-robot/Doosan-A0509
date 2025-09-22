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


//-----------------------------------------------------------

class MovePeriodicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.amp = null;
      this.periodic = null;
      this.acc = null;
      this.repeat = null;
      this.ref = null;
      this.syncType = null;
    }
    else {
      if (initObj.hasOwnProperty('amp')) {
        this.amp = initObj.amp
      }
      else {
        this.amp = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('periodic')) {
        this.periodic = initObj.periodic
      }
      else {
        this.periodic = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('repeat')) {
        this.repeat = initObj.repeat
      }
      else {
        this.repeat = 0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
      if (initObj.hasOwnProperty('syncType')) {
        this.syncType = initObj.syncType
      }
      else {
        this.syncType = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MovePeriodicRequest
    // Check that the constant length array field [amp] has the right length
    if (obj.amp.length !== 6) {
      throw new Error('Unable to serialize array field amp - length must be 6')
    }
    // Serialize message field [amp]
    bufferOffset = _arraySerializer.float64(obj.amp, buffer, bufferOffset, 6);
    // Check that the constant length array field [periodic] has the right length
    if (obj.periodic.length !== 6) {
      throw new Error('Unable to serialize array field periodic - length must be 6')
    }
    // Serialize message field [periodic]
    bufferOffset = _arraySerializer.float64(obj.periodic, buffer, bufferOffset, 6);
    // Serialize message field [acc]
    bufferOffset = _serializer.float64(obj.acc, buffer, bufferOffset);
    // Serialize message field [repeat]
    bufferOffset = _serializer.int8(obj.repeat, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [syncType]
    bufferOffset = _serializer.int8(obj.syncType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovePeriodicRequest
    let len;
    let data = new MovePeriodicRequest(null);
    // Deserialize message field [amp]
    data.amp = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [periodic]
    data.periodic = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [acc]
    data.acc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [repeat]
    data.repeat = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [syncType]
    data.syncType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 107;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MovePeriodicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b6eac8fb0c07b8dd41f6b01792ade3f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_periodic  
    #____________________________________________________________________________________________
    
    float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   
    float64[6] periodic         # Period (time for 1 cycle) [sec]
    float64    acc              # Acc-, dec- time [sec] 
    int8       repeat           # Repetition count 
    int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                # <DR_WORLD is only available in M2.40 or later 
    
    int8       syncType #=0     # SYNC = 0, ASYNC = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MovePeriodicRequest(null);
    if (msg.amp !== undefined) {
      resolved.amp = msg.amp;
    }
    else {
      resolved.amp = new Array(6).fill(0)
    }

    if (msg.periodic !== undefined) {
      resolved.periodic = msg.periodic;
    }
    else {
      resolved.periodic = new Array(6).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.repeat !== undefined) {
      resolved.repeat = msg.repeat;
    }
    else {
      resolved.repeat = 0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    if (msg.syncType !== undefined) {
      resolved.syncType = msg.syncType;
    }
    else {
      resolved.syncType = 0
    }

    return resolved;
    }
};

class MovePeriodicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MovePeriodicResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovePeriodicResponse
    let len;
    let data = new MovePeriodicResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MovePeriodicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MovePeriodicResponse(null);
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
  Request: MovePeriodicRequest,
  Response: MovePeriodicResponse,
  md5sum() { return 'd18cadb0c21c585bdf0c4a1e6878f1a6'; },
  datatype() { return 'dsr_msgs/MovePeriodic'; }
};
