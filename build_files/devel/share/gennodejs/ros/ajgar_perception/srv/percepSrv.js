// Auto-generated. Do not edit!

// (in-package ajgar_perception.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class percepSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag = null;
    }
    else {
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type percepSrvRequest
    // Serialize message field [flag]
    bufferOffset = _serializer.int32(obj.flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type percepSrvRequest
    let len;
    let data = new percepSrvRequest(null);
    // Deserialize message field [flag]
    data.flag = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_perception/percepSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa0dbc5596ec12974ea3a17a045b36e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 flag
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new percepSrvRequest(null);
    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = 0
    }

    return resolved;
    }
};

class percepSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.maskValue = null;
      this.tfArray = null;
    }
    else {
      if (initObj.hasOwnProperty('maskValue')) {
        this.maskValue = initObj.maskValue
      }
      else {
        this.maskValue = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('tfArray')) {
        this.tfArray = initObj.tfArray
      }
      else {
        this.tfArray = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type percepSrvResponse
    // Serialize message field [maskValue]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.maskValue, buffer, bufferOffset);
    // Serialize message field [tfArray]
    bufferOffset = _arraySerializer.float32(obj.tfArray, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type percepSrvResponse
    let len;
    let data = new percepSrvResponse(null);
    // Deserialize message field [maskValue]
    data.maskValue = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [tfArray]
    data.tfArray = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.maskValue);
    length += 4 * object.tfArray.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_perception/percepSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '917cc2a04d488f9811cb5549b4663ac3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 maskValue 
    float32[] tfArray
    
     
    
    
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new percepSrvResponse(null);
    if (msg.maskValue !== undefined) {
      resolved.maskValue = sensor_msgs.msg.PointCloud2.Resolve(msg.maskValue)
    }
    else {
      resolved.maskValue = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.tfArray !== undefined) {
      resolved.tfArray = msg.tfArray;
    }
    else {
      resolved.tfArray = []
    }

    return resolved;
    }
};

module.exports = {
  Request: percepSrvRequest,
  Response: percepSrvResponse,
  md5sum() { return 'a5c0d5df4352df7c34178b1c281fe9a4'; },
  datatype() { return 'ajgar_perception/percepSrv'; }
};
