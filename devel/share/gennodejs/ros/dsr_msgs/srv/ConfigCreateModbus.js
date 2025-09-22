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

class ConfigCreateModbusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.ip = null;
      this.port = null;
      this.reg_type = null;
      this.index = null;
      this.value = null;
      this.slave_id = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('ip')) {
        this.ip = initObj.ip
      }
      else {
        this.ip = '';
      }
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('reg_type')) {
        this.reg_type = initObj.reg_type
      }
      else {
        this.reg_type = 0;
      }
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
      if (initObj.hasOwnProperty('slave_id')) {
        this.slave_id = initObj.slave_id
      }
      else {
        this.slave_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigCreateModbusRequest
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [ip]
    bufferOffset = _serializer.string(obj.ip, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.int32(obj.port, buffer, bufferOffset);
    // Serialize message field [reg_type]
    bufferOffset = _serializer.int8(obj.reg_type, buffer, bufferOffset);
    // Serialize message field [index]
    bufferOffset = _serializer.int8(obj.index, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.int8(obj.value, buffer, bufferOffset);
    // Serialize message field [slave_id]
    bufferOffset = _serializer.int32(obj.slave_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigCreateModbusRequest
    let len;
    let data = new ConfigCreateModbusRequest(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ip]
    data.ip = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [reg_type]
    data.reg_type = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [index]
    data.index = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [slave_id]
    data.slave_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.ip);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ConfigCreateModbusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1976ab2b25981e03b4741ddfc2ade1f8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #____________________________________________________________________________________________
    # config_create_modbus  
    # This service registers the Modbus signal. 
    #____________________________________________________________________________________________
    
    string      name       # modbus signal symbol 
    string      ip         # external device ip
    int32       port       # external device port     
    int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)
    int8        index      # modbus signal index(0 ~ 9999)
    int8        value      # modbus singla value(unsigned value ; 0 ~ 65535)
    int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) 
                           # <slave_id is only available in M2.40 or later versions>  
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigCreateModbusRequest(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.ip !== undefined) {
      resolved.ip = msg.ip;
    }
    else {
      resolved.ip = ''
    }

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.reg_type !== undefined) {
      resolved.reg_type = msg.reg_type;
    }
    else {
      resolved.reg_type = 0
    }

    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    if (msg.slave_id !== undefined) {
      resolved.slave_id = msg.slave_id;
    }
    else {
      resolved.slave_id = 0
    }

    return resolved;
    }
};

class ConfigCreateModbusResponse {
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
    // Serializes a message object of type ConfigCreateModbusResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigCreateModbusResponse
    let len;
    let data = new ConfigCreateModbusResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dsr_msgs/ConfigCreateModbusResponse';
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
    const resolved = new ConfigCreateModbusResponse(null);
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
  Request: ConfigCreateModbusRequest,
  Response: ConfigCreateModbusResponse,
  md5sum() { return '21b56fd5fb77f7fccfc8eb6d995cd64a'; },
  datatype() { return 'dsr_msgs/ConfigCreateModbus'; }
};
