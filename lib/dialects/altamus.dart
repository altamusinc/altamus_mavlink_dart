import 'dart:typed_data';
import 'package:dart_mavlink/mavlink_dialect.dart';
import 'package:dart_mavlink/mavlink_message.dart';
import 'package:dart_mavlink/types.dart';

/// Micro air vehicle / autopilot classes. This identifies the individual model.
///
/// MAV_AUTOPILOT
typedef MavAutopilot = int;

/// Generic autopilot, full support for everything
///
/// MAV_AUTOPILOT_GENERIC
const MavAutopilot mavAutopilotGeneric = 0;

/// No valid autopilot, e.g. a GCS or other MAVLink component
///
/// MAV_AUTOPILOT_INVALID
const MavAutopilot mavAutopilotInvalid = 8;

/// MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
///
/// MAV_TYPE
typedef MavType = int;

/// Generic micro air vehicle
///
/// MAV_TYPE_GENERIC
const MavType mavTypeGeneric = 0;

/// Fixed wing aircraft.
///
/// MAV_TYPE_FIXED_WING
const MavType mavTypeFixedWing = 1;

/// Operator control unit / ground control station
///
/// MAV_TYPE_GCS
const MavType mavTypeGcs = 6;

/// PX4 Autopilot - http://px4.io/
///
/// MAV_AUTOPILOT_PX4
const MavType mavAutopilotPx4 = 12;

/// Onboard companion controller
///
/// MAV_TYPE_ONBOARD_CONTROLLER
const MavType mavTypeOnboardController = 18;

/// Gimbal
///
/// MAV_TYPE_GIMBAL
const MavType mavTypeGimbal = 26;

/// ADSB system
///
/// MAV_TYPE_ADSB
const MavType mavTypeAdsb = 27;

/// Camera
///
/// MAV_TYPE_CAMERA
const MavType mavTypeCamera = 30;

///
/// MAV_CMD
typedef MavCmd = int;

/// Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
///
/// MAV_CMD_SET_MESSAGE_INTERVAL
const MavCmd mavCmdSetMessageInterval = 511;

/// Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
///
/// MAV_CMD_REQUEST_MESSAGE
const MavCmd mavCmdRequestMessage = 512;

/// These flags encode the MAV mode.
///
/// MAV_MODE_FLAG
typedef MavModeFlag = int;

/// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
///
/// MAV_MODE_FLAG_SAFETY_ARMED
const MavModeFlag mavModeFlagSafetyArmed = 128;

/// 0b01000000 remote control input is enabled.
///
/// MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
const MavModeFlag mavModeFlagManualInputEnabled = 64;

/// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
///
/// MAV_MODE_FLAG_HIL_ENABLED
const MavModeFlag mavModeFlagHilEnabled = 32;

/// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
///
/// MAV_MODE_FLAG_STABILIZE_ENABLED
const MavModeFlag mavModeFlagStabilizeEnabled = 16;

/// 0b00001000 guided mode enabled, system flies waypoints / mission items.
///
/// MAV_MODE_FLAG_GUIDED_ENABLED
const MavModeFlag mavModeFlagGuidedEnabled = 8;

/// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
///
/// MAV_MODE_FLAG_AUTO_ENABLED
const MavModeFlag mavModeFlagAutoEnabled = 4;

/// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
///
/// MAV_MODE_FLAG_TEST_ENABLED
const MavModeFlag mavModeFlagTestEnabled = 2;

/// 0b00000001 Reserved for future use.
///
/// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
const MavModeFlag mavModeFlagCustomModeEnabled = 1;

/// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
///
/// MAV_MODE_FLAG_DECODE_POSITION
typedef MavModeFlagDecodePosition = int;

/// First bit:  10000000
///
/// MAV_MODE_FLAG_DECODE_POSITION_SAFETY
const MavModeFlagDecodePosition mavModeFlagDecodePositionSafety = 128;

/// Second bit: 01000000
///
/// MAV_MODE_FLAG_DECODE_POSITION_MANUAL
const MavModeFlagDecodePosition mavModeFlagDecodePositionManual = 64;

/// Third bit:  00100000
///
/// MAV_MODE_FLAG_DECODE_POSITION_HIL
const MavModeFlagDecodePosition mavModeFlagDecodePositionHil = 32;

/// Fourth bit: 00010000
///
/// MAV_MODE_FLAG_DECODE_POSITION_STABILIZE
const MavModeFlagDecodePosition mavModeFlagDecodePositionStabilize = 16;

/// Fifth bit:  00001000
///
/// MAV_MODE_FLAG_DECODE_POSITION_GUIDED
const MavModeFlagDecodePosition mavModeFlagDecodePositionGuided = 8;

/// Sixth bit:   00000100
///
/// MAV_MODE_FLAG_DECODE_POSITION_AUTO
const MavModeFlagDecodePosition mavModeFlagDecodePositionAuto = 4;

/// Seventh bit: 00000010
///
/// MAV_MODE_FLAG_DECODE_POSITION_TEST
const MavModeFlagDecodePosition mavModeFlagDecodePositionTest = 2;

/// Eighth bit: 00000001
///
/// MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE
const MavModeFlagDecodePosition mavModeFlagDecodePositionCustomMode = 1;

///
/// MAV_STATE
typedef MavState = int;

/// Uninitialized system, state is unknown.
///
/// MAV_STATE_UNINIT
const MavState mavStateUninit = 0;

/// System is booting up.
///
/// MAV_STATE_BOOT
const MavState mavStateBoot = 1;

/// System is calibrating and not flight-ready.
///
/// MAV_STATE_CALIBRATING
const MavState mavStateCalibrating = 2;

/// System is grounded and on standby. It can be launched any time.
///
/// MAV_STATE_STANDBY
const MavState mavStateStandby = 3;

/// System is active and might be already airborne. Motors are engaged.
///
/// MAV_STATE_ACTIVE
const MavState mavStateActive = 4;

/// System is in a non-normal flight mode (failsafe). It can however still navigate.
///
/// MAV_STATE_CRITICAL
const MavState mavStateCritical = 5;

/// System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down.
///
/// MAV_STATE_EMERGENCY
const MavState mavStateEmergency = 6;

/// System just initialized its power-down sequence, will shut down now.
///
/// MAV_STATE_POWEROFF
const MavState mavStatePoweroff = 7;

/// System is terminating itself (failsafe or commanded).
///
/// MAV_STATE_FLIGHT_TERMINATION
const MavState mavStateFlightTermination = 8;

/// Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).
/// Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.
/// When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.
///
/// MAV_COMPONENT
typedef MavComponent = int;

/// Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.
///
/// MAV_COMP_ID_ALL
const MavComponent mavCompIdAll = 0;

/// System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
///
/// MAV_COMP_ID_AUTOPILOT1
const MavComponent mavCompIdAutopilot1 = 1;

/// Gimbal #1.
///
/// MAV_COMP_ID_GIMBAL
const MavComponent mavCompIdGimbal = 154;

/// Type of mission items being requested/sent in mission protocol.
///
/// MAV_MISSION_TYPE
typedef MavMissionType = int;

/// Items are mission commands for main mission.
///
/// MAV_MISSION_TYPE_MISSION
const MavMissionType mavMissionTypeMission = 0;

/// Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
///
/// MAV_MISSION_TYPE_FENCE
const MavMissionType mavMissionTypeFence = 1;

/// Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.
///
/// MAV_MISSION_TYPE_RALLY
const MavMissionType mavMissionTypeRally = 2;

/// Only used in MISSION_CLEAR_ALL to clear all mission types.
///
/// MAV_MISSION_TYPE_ALL
const MavMissionType mavMissionTypeAll = 255;

/// Result from a MAVLink command (MAV_CMD)
///
/// MAV_RESULT
typedef MavResult = int;

/// Command is valid (is supported and has valid parameters), and was
/// executed.
///
/// MAV_RESULT_ACCEPTED
const MavResult mavResultAccepted = 0;

/// Command is valid, but cannot be executed at this time. This is used to
/// indicate a problem that should be fixed just by waiting (e.g. a state machine is
/// busy, can't arm because have not got GPS lock, etc.). Retrying later should
/// work.
///
/// MAV_RESULT_TEMPORARILY_REJECTED
const MavResult mavResultTemporarilyRejected = 1;

/// Command is invalid (is supported but has invalid parameters). Retrying
/// same command and parameters will not work.
///
/// MAV_RESULT_DENIED
const MavResult mavResultDenied = 2;

/// Command is not supported (unknown).
///
/// MAV_RESULT_UNSUPPORTED
const MavResult mavResultUnsupported = 3;

/// Command is valid, but execution has failed. This is used to indicate
/// any non-temporary or unexpected problem, i.e. any problem that must be fixed
/// before the command can succeed/be retried. For example, attempting to write a
/// file when out of memory, attempting to arm when sensors are not calibrated, etc.
///
/// MAV_RESULT_FAILED
const MavResult mavResultFailed = 4;

/// Command is valid and is being executed. This will be followed by
/// further progress updates, i.e. the component may send further COMMAND_ACK
/// messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the
/// implementation), and must terminate by sending a COMMAND_ACK message with final
/// result of the operation. The COMMAND_ACK.progress field can be used to indicate
/// the progress of the operation.
///
/// MAV_RESULT_IN_PROGRESS
const MavResult mavResultInProgress = 5;

/// Command has been cancelled (as a result of receiving a COMMAND_CANCEL
/// message).
///
/// MAV_RESULT_CANCELLED
const MavResult mavResultCancelled = 6;

/// Coordinate frames used by MAVLink. Not all frames are supported by all
///
///
/// MAV_FRAME
typedef MavFrame = int;

/// Global (WGS84) coordinate frame + altitude relative to mean sea level
/// (MSL).
///
/// MAV_FRAME_GLOBAL
const MavFrame mavFrameGlobal = 0;

/// NED local tangent frame (x: North, y: East, z: Down) with origin fixed
/// relative to earth.
///
/// MAV_FRAME_LOCAL_NED
const MavFrame mavFrameLocalNed = 1;

/// Type of GPS fix
///
/// GPS_FIX_TYPE
typedef GpsFixType = int;

/// No GPS connected
///
/// GPS_FIX_TYPE_NO_GPS
const GpsFixType gpsFixTypeNoGps = 0;

/// No position information, GPS is connected
///
/// GPS_FIX_TYPE_NO_FIX
const GpsFixType gpsFixTypeNoFix = 1;

/// 2D position
///
/// GPS_FIX_TYPE_2D_FIX
const GpsFixType gpsFixType2dFix = 2;

/// 3D position
///
/// GPS_FIX_TYPE_3D_FIX
const GpsFixType gpsFixType3dFix = 3;

/// DGPS/SBAS aided 3D position
///
/// GPS_FIX_TYPE_DGPS
const GpsFixType gpsFixTypeDgps = 4;

/// RTK float, 3D position
///
/// GPS_FIX_TYPE_RTK_FLOAT
const GpsFixType gpsFixTypeRtkFloat = 5;

/// RTK Fixed, 3D position
///
/// GPS_FIX_TYPE_RTK_FIXED
const GpsFixType gpsFixTypeRtkFixed = 6;

/// Static fixed, typically used for base stations
///
/// GPS_FIX_TYPE_STATIC
const GpsFixType gpsFixTypeStatic = 7;

/// PPP, 3D position.
///
/// GPS_FIX_TYPE_PPP
const GpsFixType gpsFixTypePpp = 8;

/// Components within the EOS scanner
///
/// EOS_COMPONENT
typedef EosComponent = int;

///
/// EOS_COMPONENT_LIDAR
const EosComponent eosComponentLidar = 1;

///
/// EOS_COMPONENT_YAW_MOTOR
const EosComponent eosComponentYawMotor = 2;

///
/// EOS_COMPONENT_PITCH_MOTOR
const EosComponent eosComponentPitchMotor = 4;

///
/// EOS_COMPONENT_GPS
const EosComponent eosComponentGps = 8;

///
/// EOS_COMPONENT_COMPASS
const EosComponent eosComponentCompass = 16;

///
/// EOS_COMPONENT_MCU
const EosComponent eosComponentMcu = 32;

///
/// EOS_COMPONENT_LED
const EosComponent eosComponentLed = 64;

///
/// EOS_COMPONENT_ACCEL
const EosComponent eosComponentAccel = 128;

///
/// EOS_COMPONENT_POWER_SENSOR
const EosComponent eosComponentPowerSensor = 256;

///
/// EOS_COMPONENT_SERIAL_BRIDGE
const EosComponent eosComponentSerialBridge = 512;

///
/// EOS_COMPONENT_PORT_EXPANDER
const EosComponent eosComponentPortExpander = 1024;

///
/// EOS_COMPONENT_FLASH
const EosComponent eosComponentFlash = 2048;

///
/// EOS_COMPONENT_ALL
const EosComponent eosComponentAll = 4096;

/// State of the device
///
/// EOS_STATE
typedef EosState = int;

///
/// EOS_STATE_IDLE
const EosState eosStateIdle = 1;

///
/// EOS_STATE_UPLOADING
const EosState eosStateUploading = 2;

///
/// EOS_STATE_SCANNING
const EosState eosStateScanning = 4;

///
/// EOS_STATE_INIT
const EosState eosStateInit = 8;

///
/// EOS_STATE_ERROR
const EosState eosStateError = 16;

/// Behaviors a motor can execute
///
/// MOTOR_BEHAVIOR
typedef MotorBehavior = int;

///
/// MOTOR_BEHAVIOR_MOTOR_ENABLE
const MotorBehavior motorBehaviorMotorEnable = 1;

///
/// MOTOR_BEHAVIOR_MOTOR_DISABLE
const MotorBehavior motorBehaviorMotorDisable = 2;

///
/// MOTOR_BEHAVIOR_MOTOR_RPM
const MotorBehavior motorBehaviorMotorRpm = 4;

///
/// MOTOR_BEHAVIOR_DEVICE_RPM
const MotorBehavior motorBehaviorDeviceRpm = 8;

///
/// MOTOR_BEHAVIOR_VACTUAL
const MotorBehavior motorBehaviorVactual = 16;

///
/// MOTOR_BEHAVIOR_GOTO_ANGLE
const MotorBehavior motorBehaviorGotoAngle = 32;

///
/// MOTOR_BEHAVIOR_STEP
const MotorBehavior motorBehaviorStep = 64;

///
/// MOTOR_BEHAVIOR_HOME
const MotorBehavior motorBehaviorHome = 128;

/// Power behavior to execute
///
/// EOS_COMPONENT_POWER_BEHAVIOR
typedef EosComponentPowerBehavior = int;

///
/// EOS_COMPONENT_POWER_BEHAVIOR_ENABLE
const EosComponentPowerBehavior eosComponentPowerBehaviorEnable = 1;

///
/// EOS_COMPONENT_POWER_BEHAVIOR_DISABLE
const EosComponentPowerBehavior eosComponentPowerBehaviorDisable = 2;

///
/// EOS_COMPONENT_POWER_BEHAVIOR_REBOOT
const EosComponentPowerBehavior eosComponentPowerBehaviorReboot = 4;

/// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
///
/// HEARTBEAT
class Heartbeat implements MavlinkMessage {
  static const int _mavlinkMessageId = 0;

  static const int _mavlinkCrcExtra = 50;

  static const int mavlinkEncodedLength = 9;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// A bitfield for use for autopilot-specific flags
  ///
  /// MAVLink type: uint32_t
  ///
  /// custom_mode
  final uint32_t customMode;

  /// Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavType]
  ///
  /// type
  final MavType type;

  /// Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavAutopilot]
  ///
  /// autopilot
  final MavAutopilot autopilot;

  /// System mode bitmap.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavModeFlag]
  ///
  /// base_mode
  final MavModeFlag baseMode;

  /// System status flag.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavState]
  ///
  /// system_status
  final MavState systemStatus;

  /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
  ///
  /// MAVLink type: uint8_t
  ///
  /// mavlink_version
  final uint8_t mavlinkVersion;

  Heartbeat({
    required this.customMode,
    required this.type,
    required this.autopilot,
    required this.baseMode,
    required this.systemStatus,
    required this.mavlinkVersion,
  });

  factory Heartbeat.parse(ByteData data_) {
    if (data_.lengthInBytes < Heartbeat.mavlinkEncodedLength) {
      var len = Heartbeat.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var customMode = data_.getUint32(0, Endian.little);
    var type = data_.getUint8(4);
    var autopilot = data_.getUint8(5);
    var baseMode = data_.getUint8(6);
    var systemStatus = data_.getUint8(7);
    var mavlinkVersion = data_.getUint8(8);

    return Heartbeat(
        customMode: customMode,
        type: type,
        autopilot: autopilot,
        baseMode: baseMode,
        systemStatus: systemStatus,
        mavlinkVersion: mavlinkVersion);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint32(0, customMode, Endian.little);
    data_.setUint8(4, type);
    data_.setUint8(5, autopilot);
    data_.setUint8(6, baseMode);
    data_.setUint8(7, systemStatus);
    data_.setUint8(8, mavlinkVersion);
    return data_;
  }
}

/// Version and capability of protocol version. This message can be requested with MAV_CMD_REQUEST_MESSAGE and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to a request for PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
///
/// PROTOCOL_VERSION
class ProtocolVersion implements MavlinkMessage {
  static const int _mavlinkMessageId = 300;

  static const int _mavlinkCrcExtra = 217;

  static const int mavlinkEncodedLength = 22;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
  ///
  /// MAVLink type: uint16_t
  ///
  /// version
  final uint16_t version;

  /// Minimum MAVLink version supported
  ///
  /// MAVLink type: uint16_t
  ///
  /// min_version
  final uint16_t minVersion;

  /// Maximum MAVLink version supported (set to the same value as version by default)
  ///
  /// MAVLink type: uint16_t
  ///
  /// max_version
  final uint16_t maxVersion;

  /// The first 8 bytes (not characters printed in hex!) of the git hash.
  ///
  /// MAVLink type: uint8_t[8]
  ///
  /// spec_version_hash
  final List<int8_t> specVersionHash;

  /// The first 8 bytes (not characters printed in hex!) of the git hash.
  ///
  /// MAVLink type: uint8_t[8]
  ///
  /// library_version_hash
  final List<int8_t> libraryVersionHash;

  ProtocolVersion({
    required this.version,
    required this.minVersion,
    required this.maxVersion,
    required this.specVersionHash,
    required this.libraryVersionHash,
  });

  factory ProtocolVersion.parse(ByteData data_) {
    if (data_.lengthInBytes < ProtocolVersion.mavlinkEncodedLength) {
      var len = ProtocolVersion.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var version = data_.getUint16(0, Endian.little);
    var minVersion = data_.getUint16(2, Endian.little);
    var maxVersion = data_.getUint16(4, Endian.little);
    var specVersionHash = MavlinkMessage.asUint8List(data_, 6, 8);
    var libraryVersionHash = MavlinkMessage.asUint8List(data_, 14, 8);

    return ProtocolVersion(
        version: version,
        minVersion: minVersion,
        maxVersion: maxVersion,
        specVersionHash: specVersionHash,
        libraryVersionHash: libraryVersionHash);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint16(0, version, Endian.little);
    data_.setUint16(2, minVersion, Endian.little);
    data_.setUint16(4, maxVersion, Endian.little);
    MavlinkMessage.setUint8List(data_, 6, specVersionHash);
    MavlinkMessage.setUint8List(data_, 14, libraryVersionHash);
    return data_;
  }
}

/// The global position, as returned by the Global Positioning System (GPS).
/// This is
/// NOT the global position estimate of the system, but rather a RAW sensor value. See
/// message GLOBAL_POSITION_INT for the global position estimate.
///
/// GPS_RAW_INT
class GpsRawInt implements MavlinkMessage {
  static const int _mavlinkMessageId = 24;

  static const int _mavlinkCrcExtra = 24;

  static const int mavlinkEncodedLength = 52;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Timestamp (UNIX Epoch time or time
  /// since system boot). The receiving end can infer timestamp format (since 1.1.1970 or
  /// since system boot) by checking for the magnitude of the number.
  ///
  /// MAVLink type: uint64_t
  ///
  /// units: us
  ///
  /// time_usec
  final uint64_t timeUsec;

  /// Latitude (WGS84, EGM96 ellipsoid)
  ///
  /// MAVLink type: int32_t
  ///
  /// units: degE7
  ///
  /// lat
  final int32_t lat;

  /// Longitude (WGS84, EGM96 ellipsoid)
  ///
  /// MAVLink type: int32_t
  ///
  /// units: degE7
  ///
  /// lon
  final int32_t lon;

  /// Altitude (MSL). Positive for up. Note that
  /// virtually all GPS modules provide the MSL altitude in addition to the WGS84
  /// altitude.
  ///
  /// MAVLink type: int32_t
  ///
  /// units: mm
  ///
  /// alt
  final int32_t alt;

  /// GPS HDOP
  /// horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
  ///
  /// MAVLink type: uint16_t
  ///
  /// eph
  final uint16_t eph;

  /// GPS VDOP
  /// vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
  ///
  /// MAVLink type: uint16_t
  ///
  /// epv
  final uint16_t epv;

  /// GPS ground speed. If
  /// unknown, set to: UINT16_MAX
  ///
  /// MAVLink type: uint16_t
  ///
  /// units: cm/s
  ///
  /// vel
  final uint16_t vel;

  /// Course over ground
  /// (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
  /// unknown, set to: UINT16_MAX
  ///
  /// MAVLink type: uint16_t
  ///
  /// units: cdeg
  ///
  /// cog
  final uint16_t cog;

  /// GPS fix type.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [GpsFixType]
  ///
  /// fix_type
  final GpsFixType fixType;

  /// Number of satellites
  /// visible. If unknown, set to UINT8_MAX
  ///
  /// MAVLink type: uint8_t
  ///
  /// satellites_visible
  final uint8_t satellitesVisible;

  /// Altitude (above WGS84, EGM96
  /// ellipsoid). Positive for up.
  ///
  /// MAVLink type: int32_t
  ///
  /// units: mm
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// alt_ellipsoid
  final int32_t altEllipsoid;

  /// Position uncertainty.
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: mm
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// h_acc
  final uint32_t hAcc;

  /// Altitude uncertainty.
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: mm
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// v_acc
  final uint32_t vAcc;

  /// Speed uncertainty.
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: mm
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// vel_acc
  final uint32_t velAcc;

  /// Heading / track uncertainty
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: degE5
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// hdg_acc
  final uint32_t hdgAcc;

  /// Yaw in earth frame from
  /// north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is
  /// configured to provide yaw and is currently unable to provide it. Use 36000 for
  /// north.
  ///
  /// MAVLink type: uint16_t
  ///
  /// units: cdeg
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// yaw
  final uint16_t yaw;

  GpsRawInt({
    required this.timeUsec,
    required this.lat,
    required this.lon,
    required this.alt,
    required this.eph,
    required this.epv,
    required this.vel,
    required this.cog,
    required this.fixType,
    required this.satellitesVisible,
    required this.altEllipsoid,
    required this.hAcc,
    required this.vAcc,
    required this.velAcc,
    required this.hdgAcc,
    required this.yaw,
  });

  factory GpsRawInt.parse(ByteData data_) {
    if (data_.lengthInBytes < GpsRawInt.mavlinkEncodedLength) {
      var len = GpsRawInt.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var timeUsec = data_.getUint64(0, Endian.little);
    var lat = data_.getInt32(8, Endian.little);
    var lon = data_.getInt32(12, Endian.little);
    var alt = data_.getInt32(16, Endian.little);
    var eph = data_.getUint16(20, Endian.little);
    var epv = data_.getUint16(22, Endian.little);
    var vel = data_.getUint16(24, Endian.little);
    var cog = data_.getUint16(26, Endian.little);
    var fixType = data_.getUint8(28);
    var satellitesVisible = data_.getUint8(29);
    var altEllipsoid = data_.getInt32(30, Endian.little);
    var hAcc = data_.getUint32(34, Endian.little);
    var vAcc = data_.getUint32(38, Endian.little);
    var velAcc = data_.getUint32(42, Endian.little);
    var hdgAcc = data_.getUint32(46, Endian.little);
    var yaw = data_.getUint16(50, Endian.little);

    return GpsRawInt(
        timeUsec: timeUsec,
        lat: lat,
        lon: lon,
        alt: alt,
        eph: eph,
        epv: epv,
        vel: vel,
        cog: cog,
        fixType: fixType,
        satellitesVisible: satellitesVisible,
        altEllipsoid: altEllipsoid,
        hAcc: hAcc,
        vAcc: vAcc,
        velAcc: velAcc,
        hdgAcc: hdgAcc,
        yaw: yaw);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint64(0, timeUsec, Endian.little);
    data_.setInt32(8, lat, Endian.little);
    data_.setInt32(12, lon, Endian.little);
    data_.setInt32(16, alt, Endian.little);
    data_.setUint16(20, eph, Endian.little);
    data_.setUint16(22, epv, Endian.little);
    data_.setUint16(24, vel, Endian.little);
    data_.setUint16(26, cog, Endian.little);
    data_.setUint8(28, fixType);
    data_.setUint8(29, satellitesVisible);
    data_.setInt32(30, altEllipsoid, Endian.little);
    data_.setUint32(34, hAcc, Endian.little);
    data_.setUint32(38, vAcc, Endian.little);
    data_.setUint32(42, velAcc, Endian.little);
    data_.setUint32(46, hdgAcc, Endian.little);
    data_.setUint16(50, yaw, Endian.little);
    return data_;
  }
}

/// Message encoding a mission item. This message is emitted to announce
/// the presence of a mission item and to set a mission item on the system. The mission
/// item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude.
/// Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU).
/// NaN may be used to indicate an optional/default value (e.g. to use the system's
/// current latitude or yaw rather than a specific value). See also
/// https://mavlink.io/en/services/mission.html.
///
/// MISSION_ITEM
class MissionItem implements MavlinkMessage {
  static const int _mavlinkMessageId = 39;

  static const int _mavlinkCrcExtra = 254;

  static const int mavlinkEncodedLength = 38;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// PARAM1, see MAV_CMD enum
  ///
  /// MAVLink type: float
  ///
  /// param1
  final float param1;

  /// PARAM2, see MAV_CMD enum
  ///
  /// MAVLink type: float
  ///
  /// param2
  final float param2;

  /// PARAM3, see MAV_CMD enum
  ///
  /// MAVLink type: float
  ///
  /// param3
  final float param3;

  /// PARAM4, see MAV_CMD enum
  ///
  /// MAVLink type: float
  ///
  /// param4
  final float param4;

  /// PARAM5 / local: X coordinate, global: latitude
  ///
  /// MAVLink type: float
  ///
  /// x
  final float x;

  /// PARAM6 / local: Y coordinate, global: longitude
  ///
  /// MAVLink type: float
  ///
  /// y
  final float y;

  /// PARAM7 / local: Z coordinate, global: altitude (relative or
  /// absolute, depending on frame).
  ///
  /// MAVLink type: float
  ///
  /// z
  final float z;

  /// Sequence
  ///
  /// MAVLink type: uint16_t
  ///
  /// seq
  final uint16_t seq;

  /// The scheduled action for the
  /// waypoint.
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [MavCmd]
  ///
  /// command
  final MavCmd command;

  /// System ID
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Component ID
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_component
  final uint8_t targetComponent;

  /// The coordinate system of the
  /// waypoint.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavFrame]
  ///
  /// frame
  final MavFrame frame;

  /// false:0, true:1
  ///
  /// MAVLink type: uint8_t
  ///
  /// current
  final uint8_t current;

  /// Autocontinue to next waypoint. 0: false, 1:
  /// true. Set false to pause mission after the item completes.
  ///
  /// MAVLink type: uint8_t
  ///
  /// autocontinue
  final uint8_t autocontinue;

  /// Mission type.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavMissionType]
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// mission_type
  final MavMissionType missionType;

  MissionItem({
    required this.param1,
    required this.param2,
    required this.param3,
    required this.param4,
    required this.x,
    required this.y,
    required this.z,
    required this.seq,
    required this.command,
    required this.targetSystem,
    required this.targetComponent,
    required this.frame,
    required this.current,
    required this.autocontinue,
    required this.missionType,
  });

  factory MissionItem.parse(ByteData data_) {
    if (data_.lengthInBytes < MissionItem.mavlinkEncodedLength) {
      var len = MissionItem.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var param1 = data_.getFloat32(0, Endian.little);
    var param2 = data_.getFloat32(4, Endian.little);
    var param3 = data_.getFloat32(8, Endian.little);
    var param4 = data_.getFloat32(12, Endian.little);
    var x = data_.getFloat32(16, Endian.little);
    var y = data_.getFloat32(20, Endian.little);
    var z = data_.getFloat32(24, Endian.little);
    var seq = data_.getUint16(28, Endian.little);
    var command = data_.getUint16(30, Endian.little);
    var targetSystem = data_.getUint8(32);
    var targetComponent = data_.getUint8(33);
    var frame = data_.getUint8(34);
    var current = data_.getUint8(35);
    var autocontinue = data_.getUint8(36);
    var missionType = data_.getUint8(37);

    return MissionItem(
        param1: param1,
        param2: param2,
        param3: param3,
        param4: param4,
        x: x,
        y: y,
        z: z,
        seq: seq,
        command: command,
        targetSystem: targetSystem,
        targetComponent: targetComponent,
        frame: frame,
        current: current,
        autocontinue: autocontinue,
        missionType: missionType);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, param1, Endian.little);
    data_.setFloat32(4, param2, Endian.little);
    data_.setFloat32(8, param3, Endian.little);
    data_.setFloat32(12, param4, Endian.little);
    data_.setFloat32(16, x, Endian.little);
    data_.setFloat32(20, y, Endian.little);
    data_.setFloat32(24, z, Endian.little);
    data_.setUint16(28, seq, Endian.little);
    data_.setUint16(30, command, Endian.little);
    data_.setUint8(32, targetSystem);
    data_.setUint8(33, targetComponent);
    data_.setUint8(34, frame);
    data_.setUint8(35, current);
    data_.setUint8(36, autocontinue);
    data_.setUint8(37, missionType);
    return data_;
  }
}

/// Report status of a command. Includes feedback whether the command was
/// executed. The command microservice is documented at
/// https://mavlink.io/en/services/command.html
///
/// COMMAND_ACK
class CommandAck implements MavlinkMessage {
  static const int _mavlinkMessageId = 77;

  static const int _mavlinkCrcExtra = 143;

  static const int mavlinkEncodedLength = 10;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Command ID (of acknowledged
  /// command).
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [MavCmd]
  ///
  /// command
  final MavCmd command;

  /// Result of command.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavResult]
  ///
  /// result
  final MavResult result;

  /// The progress
  /// percentage when result is MAV_RESULT_IN_PROGRESS. Values: [0-100], or UINT8_MAX if
  /// the progress is unknown.
  ///
  /// MAVLink type: uint8_t
  ///
  /// units: %
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// progress
  final uint8_t progress;

  /// Additional result information. Can be set
  /// with a command-specific enum containing command-specific error reasons for why the
  /// command might be denied. If used, the associated enum must be documented in the
  /// corresponding MAV_CMD (this enum should have a 0 value to indicate "unused" or
  /// "unknown").
  ///
  /// MAVLink type: int32_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// result_param2
  final int32_t resultParam2;

  /// System ID of the target recipient. This is
  /// the ID of the system that sent the command for which this COMMAND_ACK is an
  /// acknowledgement.
  ///
  /// MAVLink type: uint8_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Component ID of the target recipient. This
  /// is the ID of the system that sent the command for which this COMMAND_ACK is an
  /// acknowledgement.
  ///
  /// MAVLink type: uint8_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// target_component
  final uint8_t targetComponent;

  CommandAck({
    required this.command,
    required this.result,
    required this.progress,
    required this.resultParam2,
    required this.targetSystem,
    required this.targetComponent,
  });

  factory CommandAck.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandAck.mavlinkEncodedLength) {
      var len = CommandAck.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var command = data_.getUint16(0, Endian.little);
    var result = data_.getUint8(2);
    var progress = data_.getUint8(3);
    var resultParam2 = data_.getInt32(4, Endian.little);
    var targetSystem = data_.getUint8(8);
    var targetComponent = data_.getUint8(9);

    return CommandAck(
        command: command,
        result: result,
        progress: progress,
        resultParam2: resultParam2,
        targetSystem: targetSystem,
        targetComponent: targetComponent);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint16(0, command, Endian.little);
    data_.setUint8(2, result);
    data_.setUint8(3, progress);
    data_.setInt32(4, resultParam2, Endian.little);
    data_.setUint8(8, targetSystem);
    data_.setUint8(9, targetComponent);
    return data_;
  }
}

/// Cancel a long running command. The target system should respond with a
/// COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long
/// running process was cancelled. If it has already completed, the cancel action can be
/// ignored. The cancel action can be retried until some sort of acknowledgement to the
/// original command has been received. The command microservice is documented at
/// https://mavlink.io/en/services/command.html
///
/// COMMAND_CANCEL
class CommandCancel implements MavlinkMessage {
  static const int _mavlinkMessageId = 80;

  static const int _mavlinkCrcExtra = 14;

  static const int mavlinkEncodedLength = 4;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Command ID (of command to cancel).
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [MavCmd]
  ///
  /// command
  final MavCmd command;

  /// System executing long running command. Should
  /// not be broadcast (0).
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Component executing long running command.
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_component
  final uint8_t targetComponent;

  CommandCancel({
    required this.command,
    required this.targetSystem,
    required this.targetComponent,
  });

  factory CommandCancel.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandCancel.mavlinkEncodedLength) {
      var len = CommandCancel.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var command = data_.getUint16(0, Endian.little);
    var targetSystem = data_.getUint8(2);
    var targetComponent = data_.getUint8(3);

    return CommandCancel(
        command: command,
        targetSystem: targetSystem,
        targetComponent: targetComponent);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint16(0, command, Endian.little);
    data_.setUint8(2, targetSystem);
    data_.setUint8(3, targetComponent);
    return data_;
  }
}

/// Readings from the lidar
///
/// LIDAR_READING
class LidarReading implements MavlinkMessage {
  static const int _mavlinkMessageId = 1;

  static const int _mavlinkCrcExtra = 134;

  static const int mavlinkEncodedLength = 40;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  ///
  ///
  /// MAVLink type: uint32_t[10]
  ///
  /// units: foo
  ///
  /// readings
  final List<int32_t> readings;

  LidarReading({
    required this.readings,
  });

  factory LidarReading.parse(ByteData data_) {
    if (data_.lengthInBytes < LidarReading.mavlinkEncodedLength) {
      var len = LidarReading.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var readings = MavlinkMessage.asUint32List(data_, 0, 10);

    return LidarReading(readings: readings);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    MavlinkMessage.setUint32List(data_, 0, readings);
    return data_;
  }
}

/// Message that will component devices on or off in the EOS scanner
///
/// COMPONENT_POWER_CONTROL
class ComponentPowerControl implements MavlinkMessage {
  static const int _mavlinkMessageId = 2;

  static const int _mavlinkCrcExtra = 246;

  static const int mavlinkEncodedLength = 3;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Device to target
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [EosComponent]
  ///
  /// device
  final EosComponent device;

  /// Behavior to execute
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [EosComponentPowerBehavior]
  ///
  /// behavior
  final EosComponentPowerBehavior behavior;

  ComponentPowerControl({
    required this.device,
    required this.behavior,
  });

  factory ComponentPowerControl.parse(ByteData data_) {
    if (data_.lengthInBytes < ComponentPowerControl.mavlinkEncodedLength) {
      var len =
          ComponentPowerControl.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var device = data_.getUint16(0, Endian.little);
    var behavior = data_.getUint8(2);

    return ComponentPowerControl(device: device, behavior: behavior);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint16(0, device, Endian.little);
    data_.setUint8(2, behavior);
    return data_;
  }
}

/// Overall System Status Message
///
/// SYSTEM_STATUS
class SystemStatus implements MavlinkMessage {
  static const int _mavlinkMessageId = 3;

  static const int _mavlinkCrcExtra = 252;

  static const int mavlinkEncodedLength = 5;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Bitmask of devices and their power status: 1 = on 0 = off
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [EosComponent]
  ///
  /// power_status_bitmask
  final EosComponent powerStatusBitmask;

  /// Bitmask of health of devices: 1 = healthy 0 = unhealthy
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [EosComponent]
  ///
  /// health_status_bitmask
  final EosComponent healthStatusBitmask;

  /// Current State of the Device
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [EosState]
  ///
  /// state
  final EosState state;

  SystemStatus({
    required this.powerStatusBitmask,
    required this.healthStatusBitmask,
    required this.state,
  });

  factory SystemStatus.parse(ByteData data_) {
    if (data_.lengthInBytes < SystemStatus.mavlinkEncodedLength) {
      var len = SystemStatus.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var powerStatusBitmask = data_.getUint16(0, Endian.little);
    var healthStatusBitmask = data_.getUint16(2, Endian.little);
    var state = data_.getUint8(4);

    return SystemStatus(
        powerStatusBitmask: powerStatusBitmask,
        healthStatusBitmask: healthStatusBitmask,
        state: state);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint16(0, powerStatusBitmask, Endian.little);
    data_.setUint16(2, healthStatusBitmask, Endian.little);
    data_.setUint8(4, state);
    return data_;
  }
}

/// Controls Motors
///
/// MOTOR_CONTROL
class MotorControl implements MavlinkMessage {
  static const int _mavlinkMessageId = 4;

  static const int _mavlinkCrcExtra = 37;

  static const int mavlinkEncodedLength = 18;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// RPM to set motor shaft too. Ignores gearing. Only used if MOTOR_BEHAVIOR_MOTOR_RPM is selected.
  ///
  /// MAVLink type: float
  ///
  /// motor_rpm
  final float motorRpm;

  /// Angle to goto. Accounts for gearing. 0-360. Values above 360 will be wrapped around.
  ///
  /// MAVLink type: float
  ///
  /// target_angle
  final float targetAngle;

  /// RPM to set device too. Takes into account gearing. Only used if MOTOR_BEHAVIOR_DEVICE_RPM is selected
  ///
  /// MAVLink type: float
  ///
  /// device_rpm
  final float deviceRpm;

  /// Number of steps to execute. Negative values will step backwards. Only used if MOTOR_BEHAVIOR_STEP is selected
  ///
  /// MAVLink type: int16_t
  ///
  /// steps_count
  final int16_t stepsCount;

  /// VACTUAL value to send to stepper driver. Negative values will go backwards. Only used if MOTOR_BEHAVIOR_VACTUAL is selected
  ///
  /// MAVLink type: int16_t
  ///
  /// vactual
  final int16_t vactual;

  /// Which motor to target. Only responds to EOS_COMPONENT_YAW_MOTOR and EOS_COMPONENT_PITCH_MOTOR
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [EosComponent]
  ///
  /// target
  final EosComponent target;

  /// Behavior to Execute
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MotorBehavior]
  ///
  /// behavior
  final MotorBehavior behavior;

  MotorControl({
    required this.motorRpm,
    required this.targetAngle,
    required this.deviceRpm,
    required this.stepsCount,
    required this.vactual,
    required this.target,
    required this.behavior,
  });

  factory MotorControl.parse(ByteData data_) {
    if (data_.lengthInBytes < MotorControl.mavlinkEncodedLength) {
      var len = MotorControl.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var motorRpm = data_.getFloat32(0, Endian.little);
    var targetAngle = data_.getFloat32(4, Endian.little);
    var deviceRpm = data_.getFloat32(8, Endian.little);
    var stepsCount = data_.getInt16(12, Endian.little);
    var vactual = data_.getInt16(14, Endian.little);
    var target = data_.getUint8(16);
    var behavior = data_.getUint8(17);

    return MotorControl(
        motorRpm: motorRpm,
        targetAngle: targetAngle,
        deviceRpm: deviceRpm,
        stepsCount: stepsCount,
        vactual: vactual,
        target: target,
        behavior: behavior);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, motorRpm, Endian.little);
    data_.setFloat32(4, targetAngle, Endian.little);
    data_.setFloat32(8, deviceRpm, Endian.little);
    data_.setInt16(12, stepsCount, Endian.little);
    data_.setInt16(14, vactual, Endian.little);
    data_.setUint8(16, target);
    data_.setUint8(17, behavior);
    return data_;
  }
}

/// Motor settings. If emitted by device, represents current settings. If emitted by control software, device will update accordingly
///
/// MOTOR_SETTINGS
class MotorSettings implements MavlinkMessage {
  static const int _mavlinkMessageId = 5;

  static const int _mavlinkCrcExtra = 39;

  static const int mavlinkEncodedLength = 24;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Gearing ratio to apply, specified as Motor Teeth:Device Teeth; Send as float i.e. 20.0/72.0 becomes 0.2777777
  ///
  /// MAVLink type: float
  ///
  /// gearing_ratio
  final float gearingRatio;

  /// Rate at which microsteps are being triggered by the driver internal oscillator. Read Only.
  ///
  /// MAVLink type: float
  ///
  /// units: Hz
  ///
  /// usteps_rate
  final float ustepsRate;

  /// Device angle travelled per microstep. Read Only.
  ///
  /// MAVLink type: float
  ///
  /// units: deg
  ///
  /// ustep_angle
  final float ustepAngle;

  /// Motor current 0-2500
  ///
  /// MAVLink type: uint16_t
  ///
  /// current
  final uint16_t current;

  /// Number of steps to move from home position after homing.
  ///
  /// MAVLink type: uint16_t
  ///
  /// home_offset_steps
  final uint16_t homeOffsetSteps;

  /// Number of steps from home switch triggering to repeatable index pulse. Read Only.
  ///
  /// MAVLink type: uint16_t
  ///
  /// steps_to_next_index
  final uint16_t stepsToNextIndex;

  /// Which motor we're referring to.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [EosComponent]
  ///
  /// motor
  final EosComponent motor;

  /// Microsteps used for stepping 1-256 in powers of 2
  ///
  /// MAVLink type: uint8_t
  ///
  /// microsteps
  final uint8_t microsteps;

  /// Boolean if spread cycle is enabled. 0 = disabled, 1 = enabled
  ///
  /// MAVLink type: uint8_t
  ///
  /// spread_cycle
  final uint8_t spreadCycle;

  /// Boolean if pwm_autoscale is enabled. 0 = disabled, 1 = enabled
  ///
  /// MAVLink type: uint8_t
  ///
  /// pwm_autoscale
  final uint8_t pwmAutoscale;

  /// Boolean if pwm_autograd is enabled. 0 = disabled, 1 = enabled
  ///
  /// MAVLink type: uint8_t
  ///
  /// pwm_autograd
  final uint8_t pwmAutograd;

  /// Mininimum steps between index pulse and home switch. Set to 0 to not enforce a minimum
  ///
  /// MAVLink type: uint8_t
  ///
  /// min_steps_to_next_index
  final uint8_t minStepsToNextIndex;

  MotorSettings({
    required this.gearingRatio,
    required this.ustepsRate,
    required this.ustepAngle,
    required this.current,
    required this.homeOffsetSteps,
    required this.stepsToNextIndex,
    required this.motor,
    required this.microsteps,
    required this.spreadCycle,
    required this.pwmAutoscale,
    required this.pwmAutograd,
    required this.minStepsToNextIndex,
  });

  factory MotorSettings.parse(ByteData data_) {
    if (data_.lengthInBytes < MotorSettings.mavlinkEncodedLength) {
      var len = MotorSettings.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var gearingRatio = data_.getFloat32(0, Endian.little);
    var ustepsRate = data_.getFloat32(4, Endian.little);
    var ustepAngle = data_.getFloat32(8, Endian.little);
    var current = data_.getUint16(12, Endian.little);
    var homeOffsetSteps = data_.getUint16(14, Endian.little);
    var stepsToNextIndex = data_.getUint16(16, Endian.little);
    var motor = data_.getUint8(18);
    var microsteps = data_.getUint8(19);
    var spreadCycle = data_.getUint8(20);
    var pwmAutoscale = data_.getUint8(21);
    var pwmAutograd = data_.getUint8(22);
    var minStepsToNextIndex = data_.getUint8(23);

    return MotorSettings(
        gearingRatio: gearingRatio,
        ustepsRate: ustepsRate,
        ustepAngle: ustepAngle,
        current: current,
        homeOffsetSteps: homeOffsetSteps,
        stepsToNextIndex: stepsToNextIndex,
        motor: motor,
        microsteps: microsteps,
        spreadCycle: spreadCycle,
        pwmAutoscale: pwmAutoscale,
        pwmAutograd: pwmAutograd,
        minStepsToNextIndex: minStepsToNextIndex);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, gearingRatio, Endian.little);
    data_.setFloat32(4, ustepsRate, Endian.little);
    data_.setFloat32(8, ustepAngle, Endian.little);
    data_.setUint16(12, current, Endian.little);
    data_.setUint16(14, homeOffsetSteps, Endian.little);
    data_.setUint16(16, stepsToNextIndex, Endian.little);
    data_.setUint8(18, motor);
    data_.setUint8(19, microsteps);
    data_.setUint8(20, spreadCycle);
    data_.setUint8(21, pwmAutoscale);
    data_.setUint8(22, pwmAutograd);
    data_.setUint8(23, minStepsToNextIndex);
    return data_;
  }
}

/// Current Status of the motor
///
/// MOTOR_STATUS
class MotorStatus implements MavlinkMessage {
  static const int _mavlinkMessageId = 6;

  static const int _mavlinkCrcExtra = 61;

  static const int mavlinkEncodedLength = 23;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Current RPM of the motor.
  ///
  /// MAVLink type: float
  ///
  /// motor_rpm
  final float motorRpm;

  /// Current RPM of the device, accounting for gearing.
  ///
  /// MAVLink type: float
  ///
  /// device_rpm
  final float deviceRpm;

  /// Measured RPM, if external sensor is available
  ///
  /// MAVLink type: float
  ///
  /// measured_rpm
  final float measuredRpm;

  /// Current angle: 0-360. INF if not homed.
  ///
  /// MAVLink type: float
  ///
  /// current_angle
  final float currentAngle;

  /// Current VACTUAL value.
  ///
  /// MAVLink type: uint16_t
  ///
  /// vactual
  final uint16_t vactual;

  /// Number of steps from home. UINT16_MAX if motor is in RPM mode.
  ///
  /// MAVLink type: int16_t
  ///
  /// steps_count
  final int16_t stepsCount;

  /// Which motor we're referring to.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [EosComponent]
  ///
  /// motor
  final EosComponent motor;

  /// Boolean if the motor is enabled or not. 0 = disabled, 1 = enabled.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enabled
  final uint8_t enabled;

  /// Boolean if the motor is homed or not. 0 = not homed, 1 = homed.
  ///
  /// MAVLink type: uint8_t
  ///
  /// homed
  final uint8_t homed;

  MotorStatus({
    required this.motorRpm,
    required this.deviceRpm,
    required this.measuredRpm,
    required this.currentAngle,
    required this.vactual,
    required this.stepsCount,
    required this.motor,
    required this.enabled,
    required this.homed,
  });

  factory MotorStatus.parse(ByteData data_) {
    if (data_.lengthInBytes < MotorStatus.mavlinkEncodedLength) {
      var len = MotorStatus.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var motorRpm = data_.getFloat32(0, Endian.little);
    var deviceRpm = data_.getFloat32(4, Endian.little);
    var measuredRpm = data_.getFloat32(8, Endian.little);
    var currentAngle = data_.getFloat32(12, Endian.little);
    var vactual = data_.getUint16(16, Endian.little);
    var stepsCount = data_.getInt16(18, Endian.little);
    var motor = data_.getUint8(20);
    var enabled = data_.getUint8(21);
    var homed = data_.getUint8(22);

    return MotorStatus(
        motorRpm: motorRpm,
        deviceRpm: deviceRpm,
        measuredRpm: measuredRpm,
        currentAngle: currentAngle,
        vactual: vactual,
        stepsCount: stepsCount,
        motor: motor,
        enabled: enabled,
        homed: homed);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, motorRpm, Endian.little);
    data_.setFloat32(4, deviceRpm, Endian.little);
    data_.setFloat32(8, measuredRpm, Endian.little);
    data_.setFloat32(12, currentAngle, Endian.little);
    data_.setUint16(16, vactual, Endian.little);
    data_.setInt16(18, stepsCount, Endian.little);
    data_.setUint8(20, motor);
    data_.setUint8(21, enabled);
    data_.setUint8(22, homed);
    return data_;
  }
}

/// Indentifiying information about the EOS device
///
/// IDENTIFIER
class Identifier implements MavlinkMessage {
  static const int _mavlinkMessageId = 7;

  static const int _mavlinkCrcExtra = 110;

  static const int mavlinkEncodedLength = 50;

  @override
  int get mavlinkMessageId => _mavlinkMessageId;

  @override
  int get mavlinkCrcExtra => _mavlinkCrcExtra;

  /// Particle ID of device. Unique and unchangable
  ///
  /// MAVLink type: char[24]
  ///
  /// particle_id
  final List<char> particleId;

  /// local IPV4 Address of the device
  ///
  /// MAVLink type: uint8_t[4]
  ///
  /// local_ip
  final List<int8_t> localIp;

  /// MAC address of the device
  ///
  /// MAVLink type: uint8_t[6]
  ///
  /// mac
  final List<int8_t> mac;

  /// Friendly name of device i.e. P2-123456
  ///
  /// MAVLink type: char[16]
  ///
  /// name
  final List<char> name;

  Identifier({
    required this.particleId,
    required this.localIp,
    required this.mac,
    required this.name,
  });

  factory Identifier.parse(ByteData data_) {
    if (data_.lengthInBytes < Identifier.mavlinkEncodedLength) {
      var len = Identifier.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List() + List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var particleId = MavlinkMessage.asInt8List(data_, 0, 24);
    var localIp = MavlinkMessage.asUint8List(data_, 24, 4);
    var mac = MavlinkMessage.asUint8List(data_, 28, 6);
    var name = MavlinkMessage.asInt8List(data_, 34, 16);

    return Identifier(
        particleId: particleId, localIp: localIp, mac: mac, name: name);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    MavlinkMessage.setInt8List(data_, 0, particleId);
    MavlinkMessage.setUint8List(data_, 24, localIp);
    MavlinkMessage.setUint8List(data_, 28, mac);
    MavlinkMessage.setInt8List(data_, 34, name);
    return data_;
  }
}

class MavlinkDialectAltamus implements MavlinkDialect {
  static const int mavlinkVersion = 1;

  @override
  int get version => mavlinkVersion;

  @override
  MavlinkMessage? parse(int messageID, ByteData data) {
    switch (messageID) {
      case 0:
        return Heartbeat.parse(data);
      case 300:
        return ProtocolVersion.parse(data);
      case 24:
        return GpsRawInt.parse(data);
      case 39:
        return MissionItem.parse(data);
      case 77:
        return CommandAck.parse(data);
      case 80:
        return CommandCancel.parse(data);
      case 1:
        return LidarReading.parse(data);
      case 2:
        return ComponentPowerControl.parse(data);
      case 3:
        return SystemStatus.parse(data);
      case 4:
        return MotorControl.parse(data);
      case 5:
        return MotorSettings.parse(data);
      case 6:
        return MotorStatus.parse(data);
      case 7:
        return Identifier.parse(data);
      default:
        return null;
    }
  }

  @override
  int crcExtra(int messageID) {
    switch (messageID) {
      case 0:
        return Heartbeat._mavlinkCrcExtra;
      case 300:
        return ProtocolVersion._mavlinkCrcExtra;
      case 24:
        return GpsRawInt._mavlinkCrcExtra;
      case 39:
        return MissionItem._mavlinkCrcExtra;
      case 77:
        return CommandAck._mavlinkCrcExtra;
      case 80:
        return CommandCancel._mavlinkCrcExtra;
      case 1:
        return LidarReading._mavlinkCrcExtra;
      case 2:
        return ComponentPowerControl._mavlinkCrcExtra;
      case 3:
        return SystemStatus._mavlinkCrcExtra;
      case 4:
        return MotorControl._mavlinkCrcExtra;
      case 5:
        return MotorSettings._mavlinkCrcExtra;
      case 6:
        return MotorStatus._mavlinkCrcExtra;
      case 7:
        return Identifier._mavlinkCrcExtra;
      default:
        return -1;
    }
  }
}
