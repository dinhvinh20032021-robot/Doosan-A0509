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

class GetCurrentRotmRequest {
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
    // Serializes a message object of type GetCurrentRotmRequest
    // Serialize message field [ref]
    bufferOffset = _serializer.int8(obj.ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRotmRequest
    let len;
    let data = new GetCurrentRotmRequest(null);
    // Deserialize message field [ref]
    data.ref = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentRotmRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d7349a71916d8843a1403dfc21af071';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # get_current_rotm(ref)  
    #____________________________________________________________________________________________
    # This service returns the direction and matrix of the current tool based on the ref coordinate.
    
    int8        ref               # DR_BASE(0), DR_WORLD(2)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentRotmRequest(null);
    if (msg.ref !== undefined) {
      resolved.ref = msg.ref;
    }
    else {
      resolved.ref = 0
    }

    return resolved;
    }
};

class GetCurrentRotmResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rot_matrix = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('rot_matrix')) {
        this.rot_matrix = initObj.rot_matrix
      }
      else {
        this.rot_matrix = [];
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
    // Serializes a message object of type GetCurrentRotmResponse
    // Serialize message field [rot_matrix]
    // Serialize the length for message field [rot_matrix]
    bufferOffset = _serializer.uint32(obj.rot_matrix.length, buffer, bufferOffset);
    obj.rot_matrix.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRotmResponse
    let len;
    let data = new GetCurrentRotmResponse(null);
    // Deserialize message field [rot_matrix]
    // Deserialize array length for message field [rot_matrix]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rot_matrix = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rot_matrix[i] = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.rot_matrix.forEach((val) => {
      length += std_msgs.msg.Float64MultiArray.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/GetCurrentRotmResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d6b7bde2fb0a24acb299e2dc0b9a576';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray[] rot_matrix  # target[3][3] Rotation matrix
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
    const resolved = new GetCurrentRotmResponse(null);
    if (msg.rot_matrix !== undefined) {
      resolved.rot_matrix = new Array(msg.rot_matrix.length);
      for (let i = 0; i < resolved.rot_matrix.length; ++i) {
        resolved.rot_matrix[i] = std_msgs.msg.Float64MultiArray.Resolve(msg.rot_matrix[i]);
      }
    }
    else {
      resolved.rot_matrix = []
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
  Request: GetCurrentRotmRequest,
  Response: GetCurrentRotmResponse,
  md5sum() { return 'b547f5d15d7bef34dd869e770e7e403f'; },
  datatype() { return 'dsr_msgs/GetCurrentRotm'; }
};
