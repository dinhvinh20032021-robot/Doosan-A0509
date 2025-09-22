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

class MoveBlendingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.segment = null;
      this.posCnt = null;
      this.vel = null;
      this.acc = null;
      this.time = null;
      this.ref = null;
      this.mode = null;
      this.syncType = null;
    }
    else {
      if (initObj.hasOwnProperty('segment')) {
        this.segment = initObj.segment
      }
      else {
        this.segment = [];
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
        this.vel = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
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
    // Serializes a message object of type MoveBlendingRequest
    // Serialize message field [segment]
    // Serialize the length for message field [segment]
    bufferOffset = _serializer.uint32(obj.segment.length, buffer, bufferOffset);
    obj.segment.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [posCnt]
    bufferOffset = _serializer.int8(obj.posCnt, buffer, bufferOffset);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 2) {
      throw new Error('Unable to serialize array field vel - length must be 2')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, 2);
    // Check that the constant length array field [acc] has the right length
    if (obj.acc.length !== 2) {
      throw new Error('Unable to serialize array field acc - length must be 2')
    }
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, 2);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int8(obj.mode, buffer, bufferOffset);
    // Serialize message field [syncType]
    bufferOffset = _serializer.int8(obj.syncType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveBlendingRequest
    let len;
    let data = new MoveBlendingRequest(null);
    // Deserialize message field [segment]
    // Deserialize array length for message field [segment]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.segment = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.segment[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [posCnt]
    data.posCnt = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [syncType]
    data.syncType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.segment.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveBlendingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed742b5d9ac862a827abaa0e77801ff8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # move_blending  
    #float64[50][6] pos              # target
    #std_msgs/Float64MultiArray[] pos1   # target1 [50][6]
    #std_msgs/Float64MultiArray[] pos2   # target2 [50][6]
    #int8[50]       segment              # LINE=0 , CIRCLE=1
    #float64[50]    radius               # Radius of segment 
    #____________________________________________________________________________________________
    
    std_msgs/Float64MultiArray[] segment #50 x (pos1[6]:pos2[6]:type[1]:radius[1])        
    int8           posCnt               # target cnt 
    float64[2]     vel                  # set velocity: [mm/sec], [deg/sec]
    float64[2]     acc                  # set acceleration: [mm/sec2], [deg/sec2]
    float64        time #= 0.0          # Time [sec] 
    int8           ref                  # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                        # <DR_WORLD is only available in M2.40 or later 
    int8           mode #= 0            # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
    int8           syncType #=0         # SYNC = 0, ASYNC = 1
    
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
    const resolved = new MoveBlendingRequest(null);
    if (msg.segment !== undefined) {
      resolved.segment = new Array(msg.segment.length);
      for (let i = 0; i < resolved.segment.length; ++i) {
        resolved.segment[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.segment[i]);
      }
    }
    else {
      resolved.segment = []
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
      resolved.vel = new Array(2).fill(0)
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = new Array(2).fill(0)
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
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

class MoveBlendingResponse {
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
    // Serializes a message object of type MoveBlendingResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveBlendingResponse
    let len;
    let data = new MoveBlendingResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/MoveBlendingResponse';
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
    const resolved = new MoveBlendingResponse(null);
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
  Request: MoveBlendingRequest,
  Response: MoveBlendingResponse,
  md5sum() { return 'a42e68466eb54571ea0cfd94985bc0c0'; },
  datatype() { return 'dsr_msgs/MoveBlending'; }
};
