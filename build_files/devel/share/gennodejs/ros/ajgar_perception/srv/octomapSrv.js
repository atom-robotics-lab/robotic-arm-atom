// Auto-generated. Do not edit!

// (in-package ajgar_perception.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class octomapSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kinectInputPt = null;
      this.maskInputPt = null;
    }
    else {
      if (initObj.hasOwnProperty('kinectInputPt')) {
        this.kinectInputPt = initObj.kinectInputPt
      }
      else {
        this.kinectInputPt = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('maskInputPt')) {
        this.maskInputPt = initObj.maskInputPt
      }
      else {
        this.maskInputPt = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomapSrvRequest
    // Serialize message field [kinectInputPt]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.kinectInputPt, buffer, bufferOffset);
    // Serialize message field [maskInputPt]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.maskInputPt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomapSrvRequest
    let len;
    let data = new octomapSrvRequest(null);
    // Deserialize message field [kinectInputPt]
    data.kinectInputPt = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [maskInputPt]
    data.maskInputPt = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.kinectInputPt);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.maskInputPt);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_perception/octomapSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8633ebf80b13de6d62c444415402e8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 kinectInputPt
    sensor_msgs/PointCloud2 maskInputPt 
    
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
    const resolved = new octomapSrvRequest(null);
    if (msg.kinectInputPt !== undefined) {
      resolved.kinectInputPt = sensor_msgs.msg.PointCloud2.Resolve(msg.kinectInputPt)
    }
    else {
      resolved.kinectInputPt = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.maskInputPt !== undefined) {
      resolved.maskInputPt = sensor_msgs.msg.PointCloud2.Resolve(msg.maskInputPt)
    }
    else {
      resolved.maskInputPt = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

class octomapSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.outputPt = null;
    }
    else {
      if (initObj.hasOwnProperty('outputPt')) {
        this.outputPt = initObj.outputPt
      }
      else {
        this.outputPt = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomapSrvResponse
    // Serialize message field [outputPt]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.outputPt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomapSrvResponse
    let len;
    let data = new octomapSrvResponse(null);
    // Deserialize message field [outputPt]
    data.outputPt = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.outputPt);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ajgar_perception/octomapSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9271dde98567138fdc0d4ea20a833ddb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 outputPt 
     
    
    
    
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
    const resolved = new octomapSrvResponse(null);
    if (msg.outputPt !== undefined) {
      resolved.outputPt = sensor_msgs.msg.PointCloud2.Resolve(msg.outputPt)
    }
    else {
      resolved.outputPt = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

module.exports = {
  Request: octomapSrvRequest,
  Response: octomapSrvResponse,
  md5sum() { return '616b90dff5e1a5812063533505c3044e'; },
  datatype() { return 'ajgar_perception/octomapSrv'; }
};
