// Auto-generated. Do not edit!

// (in-package exploration_manager.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DroneState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.grid_ids = null;
      this.recent_attempt_time = null;
      this.stamp = null;
      this.pos = null;
      this.vel = null;
      this.yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('grid_ids')) {
        this.grid_ids = initObj.grid_ids
      }
      else {
        this.grid_ids = [];
      }
      if (initObj.hasOwnProperty('recent_attempt_time')) {
        this.recent_attempt_time = initObj.recent_attempt_time
      }
      else {
        this.recent_attempt_time = 0.0;
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = 0.0;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = [];
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DroneState
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int32(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [grid_ids]
    bufferOffset = _arraySerializer.int8(obj.grid_ids, buffer, bufferOffset, null);
    // Serialize message field [recent_attempt_time]
    bufferOffset = _serializer.float64(obj.recent_attempt_time, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.float64(obj.stamp, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float32(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DroneState
    let len;
    let data = new DroneState(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [grid_ids]
    data.grid_ids = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [recent_attempt_time]
    data.recent_attempt_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.grid_ids.length;
    length += 4 * object.pos.length;
    length += 4 * object.vel.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'exploration_manager/DroneState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3d2ae28cc1da43ded73b6bd55766455';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_id
    
    int8[] grid_ids
    float64 recent_attempt_time
    float64 stamp
    
    # only used for simulation
    float32[] pos
    float32[] vel
    float32 yaw
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DroneState(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.grid_ids !== undefined) {
      resolved.grid_ids = msg.grid_ids;
    }
    else {
      resolved.grid_ids = []
    }

    if (msg.recent_attempt_time !== undefined) {
      resolved.recent_attempt_time = msg.recent_attempt_time;
    }
    else {
      resolved.recent_attempt_time = 0.0
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = 0.0
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = []
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    return resolved;
    }
};

module.exports = DroneState;
