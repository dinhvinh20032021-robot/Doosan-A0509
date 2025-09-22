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

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GetCurrentPosxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ref = null;
    }
    else {
      if (initObj.hasOwnProperty('ref')) {
        this.ref = initObj.ref
      }
      else {
        this.ref = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentPosxRequest
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentPosxRequest
    let len;
    let data = new GetCurrentPosxRequest(null);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentPosxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d7349a71916d8843a1403dfc21af071';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_current_posx()  
    #____________________________________________________________________________________________
    # This service returns the current task position.
    
    int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                                 # <ref is only available in M2.40 or later> 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentPosxRequest(null);
    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    return resolved;
    }
};

class GetCurrentPosxResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_pos_info = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('task_pos_info')) {
        this.task_pos_info = initObj.task_pos_info
      }
      else {
        this.task_pos_info = [];
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
    // Serializes a message object of type GetCurrentPosxResponse
    // Serialize message field [task_pos_info]
    // Serialize the length for message field [task_pos_info]
    bufferOffset = _serializer.uint32(obj.task_pos_info.length, buffer, bufferOffset);
    obj.task_pos_info.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentPosxResponse
    let len;
    let data = new GetCurrentPosxResponse(null);
    // Deserialize message field [task_pos_info]
    // Deserialize array length for message field [task_pos_info]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.task_pos_info = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.task_pos_info[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.task_pos_info.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentPosxResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37e9c3a315ae85643f43be90c1253499';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]
    bool        success   
    
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
    const resolved = new GetCurrentPosxResponse(null);
    if (msg.task_pos_info !== undefined) {
      resolved.task_pos_info = new Array(msg.task_pos_info.length);
      for (let i = 0; i < resolved.task_pos_info.length; ++i) {
        resolved.task_pos_info[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.task_pos_info[i]);
      }
    }
    else {
      resolved.task_pos_info = []
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
  Request: GetCurrentPosxRequest,
  Response: GetCurrentPosxResponse,
  md5sum() { return 'a35de69e7061270e867fdb5acbc17c91'; },
  datatype() { return 'dsr_msgs/GetCurrentPosx'; }
};
