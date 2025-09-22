// Auto-generated. Do not edit!

// (in-package dsr_msgs.srv)


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

class MoveSplineJointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.posCnt = null;
      this.vel = null;
      this.acc = null;
      this.time = null;
      this.mode = null;
      this.syncType = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('posCnt')) {
        this.posCnt = initObj.posCnt
      }
      else {
        this.posCnt = 0;
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
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
    // Serializes a message object of type MoveSplineJointRequest
    // Serialize message field [pos]
    // Serialize the length for message field [pos]
    bufferOffset = _serializer.uint32(obj.pos.length, buffer, bufferOffset);
    obj.pos.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [posCnt]
    bufferOffset = _serializer.int8(obj.posCnt, buffer, bufferOffset);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 6) {
      throw new Error('Unable to serialize array field vel - length must be 6')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, 6);
    // Check that the constant length array field [acc] has the right length
    if (obj.acc.length !== 6) {
      throw new Error('Unable to serialize array field acc - length must be 6')
    }
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, 6);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Serialize message field [syncType]
    bufferOffset = _serializer.int8(obj.syncType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveSplineJointRequest
    let len;
    let data = new MoveSplineJointRequest(null);
    // Deserialize message field [pos]
    // Deserialize array length for message field [pos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pos[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [posCnt]
    data.posCnt = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [syncType]
    data.syncType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.pos.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 111;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveSplineJointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1900c7787fce969908abb61dfa5501e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_spline_joint  
    ###float64[100][6] pos         # target
    #____________________________________________________________________________________________
    
    std_msgs/Float64MultiArray[] pos         # target [100][6] pos
    int8       posCnt                        # target cnt 
    float64[6] vel                           # set joint velocity: [deg/sec]
    float64[6] acc                           # set joint acceleration: [deg/sec2] 
    float64    time #= 0.0                   # Time [sec] 
    int8       mode #= 0                     # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
    int8       syncType #=0                  # SYNC = 0, ASYNC = 1
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveSplineJointRequest(null);
    if (msg.pos !== undefined) {
      resolved.pos = new Array(msg.pos.length);
      for (let i = 0; i < resolved.pos.length; ++i) {
        resolved.pos[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.pos[i]);
      }
    }
    else {
      resolved.pos = []
    }

    if (msg.posCnt !== undefined) {
      resolved.posCnt = msg.posCnt;
    }
    else {
      resolved.posCnt = 0
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = new Array(6).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = new Array(6).fill(0)
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
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

class MoveSplineJointResponse {
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
    // Serializes a message object of type MoveSplineJointResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveSplineJointResponse
    let len;
    let data = new MoveSplineJointResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveSplineJointResponse';
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
    const resolved = new MoveSplineJointResponse(null);
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
  Request: MoveSplineJointRequest,
  Response: MoveSplineJointResponse,
  md5sum() { return '38e3ecc2960089549a21015a74e932c2'; },
  datatype() { return 'dsr_msgs/MoveSplineJoint'; }
};
