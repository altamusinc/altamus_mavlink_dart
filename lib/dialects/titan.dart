import 'dart:typed_data';
import 'package:dart_mavlink/mavlink_dialect.dart';
import 'package:dart_mavlink/mavlink_message.dart';
import 'package:dart_mavlink/types.dart';
import 'dart:convert';

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

/// MAV FTP error codes (https://mavlink.io/en/services/ftp.html)
///
/// MAV_FTP_ERR
typedef MavFtpErr = int;

/// None: No error
///
/// MAV_FTP_ERR_NONE
const MavFtpErr mavFtpErrNone = 0;

/// Fail: Unknown failure
///
/// MAV_FTP_ERR_FAIL
const MavFtpErr mavFtpErrFail = 1;

/// FailErrno: Command failed, Err number sent back in PayloadHeader.data[1].
/// This is a file-system error number understood by the server operating system.
///
/// MAV_FTP_ERR_FAILERRNO
const MavFtpErr mavFtpErrFailerrno = 2;

/// InvalidDataSize: Payload size is invalid
///
/// MAV_FTP_ERR_INVALIDDATASIZE
const MavFtpErr mavFtpErrInvaliddatasize = 3;

/// InvalidSession: Session is not currently open
///
/// MAV_FTP_ERR_INVALIDSESSION
const MavFtpErr mavFtpErrInvalidsession = 4;

/// NoSessionsAvailable: All available sessions are already in use
///
/// MAV_FTP_ERR_NOSESSIONSAVAILABLE
const MavFtpErr mavFtpErrNosessionsavailable = 5;

/// EOF: Offset past end of file for ListDirectory and ReadFile commands
///
/// MAV_FTP_ERR_EOF
const MavFtpErr mavFtpErrEof = 6;

/// UnknownCommand: Unknown command / opcode
///
/// MAV_FTP_ERR_UNKNOWNCOMMAND
const MavFtpErr mavFtpErrUnknowncommand = 7;

/// FileExists: File/directory already exists
///
/// MAV_FTP_ERR_FILEEXISTS
const MavFtpErr mavFtpErrFileexists = 8;

/// FileProtected: File/directory is write protected
///
/// MAV_FTP_ERR_FILEPROTECTED
const MavFtpErr mavFtpErrFileprotected = 9;

/// FileNotFound: File/directory not found
///
/// MAV_FTP_ERR_FILENOTFOUND
const MavFtpErr mavFtpErrFilenotfound = 10;

/// MAV FTP opcodes: https://mavlink.io/en/services/ftp.html
///
/// MAV_FTP_OPCODE
typedef MavFtpOpcode = int;

/// None. Ignored, always ACKed
///
/// MAV_FTP_OPCODE_NONE
const MavFtpOpcode mavFtpOpcodeNone = 0;

/// TerminateSession: Terminates open Read session
///
/// MAV_FTP_OPCODE_TERMINATESESSION
const MavFtpOpcode mavFtpOpcodeTerminatesession = 1;

/// ResetSessions: Terminates all open read sessions
///
/// MAV_FTP_OPCODE_RESETSESSION
const MavFtpOpcode mavFtpOpcodeResetsession = 2;

/// ListDirectory. List files and directories in path from offset
///
/// MAV_FTP_OPCODE_LISTDIRECTORY
const MavFtpOpcode mavFtpOpcodeListdirectory = 3;

/// OpenFileRO: Opens file at path for reading, returns session
///
/// MAV_FTP_OPCODE_OPENFILERO
const MavFtpOpcode mavFtpOpcodeOpenfilero = 4;

/// ReadFile: Reads size bytes from offset in session
///
/// MAV_FTP_OPCODE_READFILE
const MavFtpOpcode mavFtpOpcodeReadfile = 5;

/// CreateFile: Creates file at path for writing, returns session
///
/// MAV_FTP_OPCODE_CREATEFILE
const MavFtpOpcode mavFtpOpcodeCreatefile = 6;

/// WriteFile: Writes size bytes to offset in session
///
/// MAV_FTP_OPCODE_WRITEFILE
const MavFtpOpcode mavFtpOpcodeWritefile = 7;

/// RemoveFile: Remove file at path
///
/// MAV_FTP_OPCODE_REMOVEFILE
const MavFtpOpcode mavFtpOpcodeRemovefile = 8;

/// CreateDirectory: Creates directory at path
///
/// MAV_FTP_OPCODE_CREATEDIRECTORY
const MavFtpOpcode mavFtpOpcodeCreatedirectory = 9;

/// RemoveDirectory: Removes directory at path. The directory must be empty.
///
/// MAV_FTP_OPCODE_REMOVEDIRECTORY
const MavFtpOpcode mavFtpOpcodeRemovedirectory = 10;

/// OpenFileWO: Opens file at path for writing, returns session
///
/// MAV_FTP_OPCODE_OPENFILEWO
const MavFtpOpcode mavFtpOpcodeOpenfilewo = 11;

/// TruncateFile: Truncate file at path to offset length
///
/// MAV_FTP_OPCODE_TRUNCATEFILE
const MavFtpOpcode mavFtpOpcodeTruncatefile = 12;

/// Rename: Rename path1 to path2
///
/// MAV_FTP_OPCODE_RENAME
const MavFtpOpcode mavFtpOpcodeRename = 13;

/// CalcFileCRC32: Calculate CRC32 for file at path
///
/// MAV_FTP_OPCODE_CALCFILECRC
const MavFtpOpcode mavFtpOpcodeCalcfilecrc = 14;

/// BurstReadFile: Burst download session file
///
/// MAV_FTP_OPCODE_BURSTREADFILE
const MavFtpOpcode mavFtpOpcodeBurstreadfile = 15;

/// ACK: ACK response
///
/// MAV_FTP_OPCODE_ACK
const MavFtpOpcode mavFtpOpcodeAck = 128;

/// NAK: NAK response
///
/// MAV_FTP_OPCODE_NAK
const MavFtpOpcode mavFtpOpcodeNak = 129;

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

/// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
///
/// MAV_SEVERITY
typedef MavSeverity = int;

/// System is unusable. This is a "panic" condition.
///
/// MAV_SEVERITY_EMERGENCY
const MavSeverity mavSeverityEmergency = 0;

/// Action should be taken immediately. Indicates error in non-critical systems.
///
/// MAV_SEVERITY_ALERT
const MavSeverity mavSeverityAlert = 1;

/// Action must be taken immediately. Indicates failure in a primary system.
///
/// MAV_SEVERITY_CRITICAL
const MavSeverity mavSeverityCritical = 2;

/// Indicates an error in secondary/redundant systems.
///
/// MAV_SEVERITY_ERROR
const MavSeverity mavSeverityError = 3;

/// Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
///
/// MAV_SEVERITY_WARNING
const MavSeverity mavSeverityWarning = 4;

/// An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
///
/// MAV_SEVERITY_NOTICE
const MavSeverity mavSeverityNotice = 5;

/// Normal operational messages. Useful for logging. No action is required for these messages.
///
/// MAV_SEVERITY_INFO
const MavSeverity mavSeverityInfo = 6;

/// Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
///
/// MAV_SEVERITY_DEBUG
const MavSeverity mavSeverityDebug = 7;

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

/// State of the water tracking device
///
/// SYSTEM_STATE
typedef SystemState = int;

///
/// SYSTEM_STATE_INITALIZING
const SystemState systemStateInitalizing = 1;

///
/// SYSTEM_STATE_READY
const SystemState systemStateReady = 2;

///
/// SYSTEM_STATE_FILLING_IDENTIFIED
const SystemState systemStateFillingIdentified = 3;

///
/// SYSTEM_STATE_FILLING_UNIDENTIFIED
const SystemState systemStateFillingUnidentified = 4;

///
/// SYSTEM_STATE_PAUSE
const SystemState systemStatePause = 5;

///
/// SYSTEM_STATE_UPLOADING
const SystemState systemStateUploading = 6;

///
/// SYSTEM_STATE_OVERRIDE_ON
const SystemState systemStateOverrideOn = 7;

///
/// SYSTEM_STATE_ERROR
const SystemState systemStateError = 8;

/// State inputs to water tracking device
///
/// STATE_INPUT
typedef StateInput = int;

///
/// STATE_INPUT_BADGE_PRESENT
const StateInput stateInputBadgePresent = 1;

///
/// STATE_INPUT_BADGE_LEAVE
const StateInput stateInputBadgeLeave = 2;

///
/// STATE_INPUT_PULSER_PULSE
const StateInput stateInputPulserPulse = 3;

///
/// STATE_INPUT_OVERRIDE_ON
const StateInput stateInputOverrideOn = 4;

///
/// STATE_INPUT_OVERRIDE_OFF
const StateInput stateInputOverrideOff = 5;

///
/// STATE_INPUT_ADMIN_START
const StateInput stateInputAdminStart = 6;

///
/// STATE_INPUT_ADMIN_STOP
const StateInput stateInputAdminStop = 7;

///
/// STATE_INPUT_UPLOAD_REQUEST
const StateInput stateInputUploadRequest = 8;

///
/// STATE_INPUT_UPLOAD_COMPLETE
const StateInput stateInputUploadComplete = 9;

///
/// STATE_INPUT_UPLOAD_RETRY
const StateInput stateInputUploadRetry = 10;

///
/// STATE_INPUT_UPLOAD_ABORT
const StateInput stateInputUploadAbort = 11;

///
/// STATE_INPUT_FILL_LIMIT
const StateInput stateInputFillLimit = 12;

/// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
///
/// HEARTBEAT
class Heartbeat implements MavlinkMessage {
  static const int msgId = 0;

  static const int crcExtra = 50;

  static const int mavlinkEncodedLength = 9;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  Heartbeat.fromJson(Map<String, dynamic> json)
      : customMode = json['customMode'],
        type = json['type'],
        autopilot = json['autopilot'],
        baseMode = json['baseMode'],
        systemStatus = json['systemStatus'],
        mavlinkVersion = json['mavlinkVersion'];
  Heartbeat copyWith({
    uint32_t? customMode,
    MavType? type,
    MavAutopilot? autopilot,
    MavModeFlag? baseMode,
    MavState? systemStatus,
    uint8_t? mavlinkVersion,
  }) {
    return Heartbeat(
      customMode: customMode ?? this.customMode,
      type: type ?? this.type,
      autopilot: autopilot ?? this.autopilot,
      baseMode: baseMode ?? this.baseMode,
      systemStatus: systemStatus ?? this.systemStatus,
      mavlinkVersion: mavlinkVersion ?? this.mavlinkVersion,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'customMode': customMode,
        'type': type,
        'autopilot': autopilot,
        'baseMode': baseMode,
        'systemStatus': systemStatus,
        'mavlinkVersion': mavlinkVersion,
      };

  factory Heartbeat.parse(ByteData data_) {
    if (data_.lengthInBytes < Heartbeat.mavlinkEncodedLength) {
      var len = Heartbeat.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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

/// Request to control this MAV
///
/// CHANGE_OPERATOR_CONTROL
class ChangeOperatorControl implements MavlinkMessage {
  static const int msgId = 5;

  static const int crcExtra = 217;

  static const int mavlinkEncodedLength = 28;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  String get passkeyAsString => convertMavlinkCharListToString(_passkey);
  List<char> get passkey => _passkey;

  /// System the GCS requests control for
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_system
  final uint8_t targetSystem;

  /// 0: request control of this MAV, 1: Release control of this MAV
  ///
  /// MAVLink type: uint8_t
  ///
  /// control_request
  final uint8_t controlRequest;

  /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
  ///
  /// MAVLink type: uint8_t
  ///
  /// units: rad
  ///
  /// version
  final uint8_t version;

  /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
  ///
  /// MAVLink type: char[25]
  ///
  /// passkey
  final List<char> _passkey;

  ChangeOperatorControl({
    required this.targetSystem,
    required this.controlRequest,
    required this.version,
    required passkey,
  }) : _passkey = passkey;

  ChangeOperatorControl.fromJson(Map<String, dynamic> json)
      : targetSystem = json['targetSystem'],
        controlRequest = json['controlRequest'],
        version = json['version'],
        _passkey = convertStringtoMavlinkCharList(json['passkey'], length: 25);
  ChangeOperatorControl copyWith({
    uint8_t? targetSystem,
    uint8_t? controlRequest,
    uint8_t? version,
    List<char>? passkey,
  }) {
    return ChangeOperatorControl(
      targetSystem: targetSystem ?? this.targetSystem,
      controlRequest: controlRequest ?? this.controlRequest,
      version: version ?? this.version,
      passkey: passkey ?? this.passkey,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'targetSystem': targetSystem,
        'controlRequest': controlRequest,
        'version': version,
        'passkey': _passkey,
      };

  factory ChangeOperatorControl.parse(ByteData data_) {
    if (data_.lengthInBytes < ChangeOperatorControl.mavlinkEncodedLength) {
      var len =
          ChangeOperatorControl.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var targetSystem = data_.getUint8(0);
    var controlRequest = data_.getUint8(1);
    var version = data_.getUint8(2);
    var passkey = MavlinkMessage.asUint8List(data_, 3, 25);

    return ChangeOperatorControl(
        targetSystem: targetSystem,
        controlRequest: controlRequest,
        version: version,
        passkey: passkey);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint8(0, targetSystem);
    data_.setUint8(1, controlRequest);
    data_.setUint8(2, version);
    MavlinkMessage.setUint8List(data_, 3, passkey);
    return data_;
  }
}

/// Accept / deny control of this MAV
///
/// CHANGE_OPERATOR_CONTROL_ACK
class ChangeOperatorControlAck implements MavlinkMessage {
  static const int msgId = 6;

  static const int crcExtra = 104;

  static const int mavlinkEncodedLength = 3;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// ID of the GCS this message
  ///
  /// MAVLink type: uint8_t
  ///
  /// gcs_system_id
  final uint8_t gcsSystemId;

  /// 0: request control of this MAV, 1: Release control of this MAV
  ///
  /// MAVLink type: uint8_t
  ///
  /// control_request
  final uint8_t controlRequest;

  /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
  ///
  /// MAVLink type: uint8_t
  ///
  /// ack
  final uint8_t ack;

  ChangeOperatorControlAck({
    required this.gcsSystemId,
    required this.controlRequest,
    required this.ack,
  });

  ChangeOperatorControlAck.fromJson(Map<String, dynamic> json)
      : gcsSystemId = json['gcsSystemId'],
        controlRequest = json['controlRequest'],
        ack = json['ack'];
  ChangeOperatorControlAck copyWith({
    uint8_t? gcsSystemId,
    uint8_t? controlRequest,
    uint8_t? ack,
  }) {
    return ChangeOperatorControlAck(
      gcsSystemId: gcsSystemId ?? this.gcsSystemId,
      controlRequest: controlRequest ?? this.controlRequest,
      ack: ack ?? this.ack,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'gcsSystemId': gcsSystemId,
        'controlRequest': controlRequest,
        'ack': ack,
      };

  factory ChangeOperatorControlAck.parse(ByteData data_) {
    if (data_.lengthInBytes < ChangeOperatorControlAck.mavlinkEncodedLength) {
      var len =
          ChangeOperatorControlAck.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var gcsSystemId = data_.getUint8(0);
    var controlRequest = data_.getUint8(1);
    var ack = data_.getUint8(2);

    return ChangeOperatorControlAck(
        gcsSystemId: gcsSystemId, controlRequest: controlRequest, ack: ack);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint8(0, gcsSystemId);
    data_.setUint8(1, controlRequest);
    data_.setUint8(2, ack);
    return data_;
  }
}

/// Version and capability of protocol version. This message can be requested with MAV_CMD_REQUEST_MESSAGE and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to a request for PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
///
/// PROTOCOL_VERSION
class ProtocolVersion implements MavlinkMessage {
  static const int msgId = 300;

  static const int crcExtra = 217;

  static const int mavlinkEncodedLength = 22;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  ProtocolVersion.fromJson(Map<String, dynamic> json)
      : version = json['version'],
        minVersion = json['minVersion'],
        maxVersion = json['maxVersion'],
        specVersionHash = List<int>.from(json['specVersionHash']),
        libraryVersionHash = List<int>.from(json['libraryVersionHash']);
  ProtocolVersion copyWith({
    uint16_t? version,
    uint16_t? minVersion,
    uint16_t? maxVersion,
    List<int8_t>? specVersionHash,
    List<int8_t>? libraryVersionHash,
  }) {
    return ProtocolVersion(
      version: version ?? this.version,
      minVersion: minVersion ?? this.minVersion,
      maxVersion: maxVersion ?? this.maxVersion,
      specVersionHash: specVersionHash ?? this.specVersionHash,
      libraryVersionHash: libraryVersionHash ?? this.libraryVersionHash,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'version': version,
        'minVersion': minVersion,
        'maxVersion': maxVersion,
        'specVersionHash': specVersionHash,
        'libraryVersionHash': libraryVersionHash,
      };

  factory ProtocolVersion.parse(ByteData data_) {
    if (data_.lengthInBytes < ProtocolVersion.mavlinkEncodedLength) {
      var len = ProtocolVersion.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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
  static const int msgId = 24;

  static const int crcExtra = 24;

  static const int mavlinkEncodedLength = 52;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  GpsRawInt.fromJson(Map<String, dynamic> json)
      : timeUsec = json['timeUsec'],
        lat = json['lat'],
        lon = json['lon'],
        alt = json['alt'],
        eph = json['eph'],
        epv = json['epv'],
        vel = json['vel'],
        cog = json['cog'],
        fixType = json['fixType'],
        satellitesVisible = json['satellitesVisible'],
        altEllipsoid = json['altEllipsoid'],
        hAcc = json['hAcc'],
        vAcc = json['vAcc'],
        velAcc = json['velAcc'],
        hdgAcc = json['hdgAcc'],
        yaw = json['yaw'];
  GpsRawInt copyWith({
    uint64_t? timeUsec,
    int32_t? lat,
    int32_t? lon,
    int32_t? alt,
    uint16_t? eph,
    uint16_t? epv,
    uint16_t? vel,
    uint16_t? cog,
    GpsFixType? fixType,
    uint8_t? satellitesVisible,
    int32_t? altEllipsoid,
    uint32_t? hAcc,
    uint32_t? vAcc,
    uint32_t? velAcc,
    uint32_t? hdgAcc,
    uint16_t? yaw,
  }) {
    return GpsRawInt(
      timeUsec: timeUsec ?? this.timeUsec,
      lat: lat ?? this.lat,
      lon: lon ?? this.lon,
      alt: alt ?? this.alt,
      eph: eph ?? this.eph,
      epv: epv ?? this.epv,
      vel: vel ?? this.vel,
      cog: cog ?? this.cog,
      fixType: fixType ?? this.fixType,
      satellitesVisible: satellitesVisible ?? this.satellitesVisible,
      altEllipsoid: altEllipsoid ?? this.altEllipsoid,
      hAcc: hAcc ?? this.hAcc,
      vAcc: vAcc ?? this.vAcc,
      velAcc: velAcc ?? this.velAcc,
      hdgAcc: hdgAcc ?? this.hdgAcc,
      yaw: yaw ?? this.yaw,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'timeUsec': timeUsec,
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'eph': eph,
        'epv': epv,
        'vel': vel,
        'cog': cog,
        'fixType': fixType,
        'satellitesVisible': satellitesVisible,
        'altEllipsoid': altEllipsoid,
        'hAcc': hAcc,
        'vAcc': vAcc,
        'velAcc': velAcc,
        'hdgAcc': hdgAcc,
        'yaw': yaw,
      };

  factory GpsRawInt.parse(ByteData data_) {
    if (data_.lengthInBytes < GpsRawInt.mavlinkEncodedLength) {
      var len = GpsRawInt.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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
  static const int msgId = 39;

  static const int crcExtra = 254;

  static const int mavlinkEncodedLength = 38;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  MissionItem.fromJson(Map<String, dynamic> json)
      : param1 = json['param1'],
        param2 = json['param2'],
        param3 = json['param3'],
        param4 = json['param4'],
        x = json['x'],
        y = json['y'],
        z = json['z'],
        seq = json['seq'],
        command = json['command'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'],
        frame = json['frame'],
        current = json['current'],
        autocontinue = json['autocontinue'],
        missionType = json['missionType'];
  MissionItem copyWith({
    float? param1,
    float? param2,
    float? param3,
    float? param4,
    float? x,
    float? y,
    float? z,
    uint16_t? seq,
    MavCmd? command,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
    MavFrame? frame,
    uint8_t? current,
    uint8_t? autocontinue,
    MavMissionType? missionType,
  }) {
    return MissionItem(
      param1: param1 ?? this.param1,
      param2: param2 ?? this.param2,
      param3: param3 ?? this.param3,
      param4: param4 ?? this.param4,
      x: x ?? this.x,
      y: y ?? this.y,
      z: z ?? this.z,
      seq: seq ?? this.seq,
      command: command ?? this.command,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
      frame: frame ?? this.frame,
      current: current ?? this.current,
      autocontinue: autocontinue ?? this.autocontinue,
      missionType: missionType ?? this.missionType,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'param1': param1,
        'param2': param2,
        'param3': param3,
        'param4': param4,
        'x': x,
        'y': y,
        'z': z,
        'seq': seq,
        'command': command,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
        'frame': frame,
        'current': current,
        'autocontinue': autocontinue,
        'missionType': missionType,
      };

  factory MissionItem.parse(ByteData data_) {
    if (data_.lengthInBytes < MissionItem.mavlinkEncodedLength) {
      var len = MissionItem.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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

/// Send a command with up to seven parameters to the MAV, where params 5 and 6 are integers and the other values are floats. This is preferred over COMMAND_LONG as it allows the MAV_FRAME to be specified for interpreting positional information, such as altitude. COMMAND_INT is also preferred when sending latitude and longitude data in params 5 and 6, as it allows for greater precision. Param 5 and 6 encode positional data as scaled integers, where the scaling depends on the actual command value. NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). The command microservice is documented at https://mavlink.io/en/services/command.html
///
/// COMMAND_INT
class CommandInt implements MavlinkMessage {
  static const int msgId = 75;

  static const int crcExtra = 158;

  static const int mavlinkEncodedLength = 35;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
  ///
  /// MAVLink type: int32_t
  ///
  /// x
  final int32_t x;

  /// PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
  ///
  /// MAVLink type: int32_t
  ///
  /// y
  final int32_t y;

  /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
  ///
  /// MAVLink type: float
  ///
  /// z
  final float z;

  /// The scheduled action for the mission item.
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

  /// The coordinate system of the COMMAND.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavFrame]
  ///
  /// frame
  final MavFrame frame;

  /// Not used.
  ///
  /// MAVLink type: uint8_t
  ///
  /// current
  final uint8_t current;

  /// Not used (set 0).
  ///
  /// MAVLink type: uint8_t
  ///
  /// autocontinue
  final uint8_t autocontinue;

  CommandInt({
    required this.param1,
    required this.param2,
    required this.param3,
    required this.param4,
    required this.x,
    required this.y,
    required this.z,
    required this.command,
    required this.targetSystem,
    required this.targetComponent,
    required this.frame,
    required this.current,
    required this.autocontinue,
  });

  CommandInt.fromJson(Map<String, dynamic> json)
      : param1 = json['param1'],
        param2 = json['param2'],
        param3 = json['param3'],
        param4 = json['param4'],
        x = json['x'],
        y = json['y'],
        z = json['z'],
        command = json['command'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'],
        frame = json['frame'],
        current = json['current'],
        autocontinue = json['autocontinue'];
  CommandInt copyWith({
    float? param1,
    float? param2,
    float? param3,
    float? param4,
    int32_t? x,
    int32_t? y,
    float? z,
    MavCmd? command,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
    MavFrame? frame,
    uint8_t? current,
    uint8_t? autocontinue,
  }) {
    return CommandInt(
      param1: param1 ?? this.param1,
      param2: param2 ?? this.param2,
      param3: param3 ?? this.param3,
      param4: param4 ?? this.param4,
      x: x ?? this.x,
      y: y ?? this.y,
      z: z ?? this.z,
      command: command ?? this.command,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
      frame: frame ?? this.frame,
      current: current ?? this.current,
      autocontinue: autocontinue ?? this.autocontinue,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'param1': param1,
        'param2': param2,
        'param3': param3,
        'param4': param4,
        'x': x,
        'y': y,
        'z': z,
        'command': command,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
        'frame': frame,
        'current': current,
        'autocontinue': autocontinue,
      };

  factory CommandInt.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandInt.mavlinkEncodedLength) {
      var len = CommandInt.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var param1 = data_.getFloat32(0, Endian.little);
    var param2 = data_.getFloat32(4, Endian.little);
    var param3 = data_.getFloat32(8, Endian.little);
    var param4 = data_.getFloat32(12, Endian.little);
    var x = data_.getInt32(16, Endian.little);
    var y = data_.getInt32(20, Endian.little);
    var z = data_.getFloat32(24, Endian.little);
    var command = data_.getUint16(28, Endian.little);
    var targetSystem = data_.getUint8(30);
    var targetComponent = data_.getUint8(31);
    var frame = data_.getUint8(32);
    var current = data_.getUint8(33);
    var autocontinue = data_.getUint8(34);

    return CommandInt(
        param1: param1,
        param2: param2,
        param3: param3,
        param4: param4,
        x: x,
        y: y,
        z: z,
        command: command,
        targetSystem: targetSystem,
        targetComponent: targetComponent,
        frame: frame,
        current: current,
        autocontinue: autocontinue);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, param1, Endian.little);
    data_.setFloat32(4, param2, Endian.little);
    data_.setFloat32(8, param3, Endian.little);
    data_.setFloat32(12, param4, Endian.little);
    data_.setInt32(16, x, Endian.little);
    data_.setInt32(20, y, Endian.little);
    data_.setFloat32(24, z, Endian.little);
    data_.setUint16(28, command, Endian.little);
    data_.setUint8(30, targetSystem);
    data_.setUint8(31, targetComponent);
    data_.setUint8(32, frame);
    data_.setUint8(33, current);
    data_.setUint8(34, autocontinue);
    return data_;
  }
}

/// Send a command with up to seven parameters to the MAV. COMMAND_INT is generally preferred when sending MAV_CMD commands that include positional information; it offers higher precision and allows the MAV_FRAME to be specified (which may otherwise be ambiguous, particularly for altitude). The command microservice is documented at https://mavlink.io/en/services/command.html
///
/// COMMAND_LONG
class CommandLong implements MavlinkMessage {
  static const int msgId = 76;

  static const int crcExtra = 152;

  static const int mavlinkEncodedLength = 33;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// Parameter 1 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param1
  final float param1;

  /// Parameter 2 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param2
  final float param2;

  /// Parameter 3 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param3
  final float param3;

  /// Parameter 4 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param4
  final float param4;

  /// Parameter 5 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param5
  final float param5;

  /// Parameter 6 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param6
  final float param6;

  /// Parameter 7 (for the specific command).
  ///
  /// MAVLink type: float
  ///
  /// param7
  final float param7;

  /// Command ID (of command to send).
  ///
  /// MAVLink type: uint16_t
  ///
  /// enum: [MavCmd]
  ///
  /// command
  final MavCmd command;

  /// System which should execute the command
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Component which should execute the command, 0 for all components
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_component
  final uint8_t targetComponent;

  /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
  ///
  /// MAVLink type: uint8_t
  ///
  /// confirmation
  final uint8_t confirmation;

  CommandLong({
    required this.param1,
    required this.param2,
    required this.param3,
    required this.param4,
    required this.param5,
    required this.param6,
    required this.param7,
    required this.command,
    required this.targetSystem,
    required this.targetComponent,
    required this.confirmation,
  });

  CommandLong.fromJson(Map<String, dynamic> json)
      : param1 = json['param1'],
        param2 = json['param2'],
        param3 = json['param3'],
        param4 = json['param4'],
        param5 = json['param5'],
        param6 = json['param6'],
        param7 = json['param7'],
        command = json['command'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'],
        confirmation = json['confirmation'];
  CommandLong copyWith({
    float? param1,
    float? param2,
    float? param3,
    float? param4,
    float? param5,
    float? param6,
    float? param7,
    MavCmd? command,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
    uint8_t? confirmation,
  }) {
    return CommandLong(
      param1: param1 ?? this.param1,
      param2: param2 ?? this.param2,
      param3: param3 ?? this.param3,
      param4: param4 ?? this.param4,
      param5: param5 ?? this.param5,
      param6: param6 ?? this.param6,
      param7: param7 ?? this.param7,
      command: command ?? this.command,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
      confirmation: confirmation ?? this.confirmation,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'param1': param1,
        'param2': param2,
        'param3': param3,
        'param4': param4,
        'param5': param5,
        'param6': param6,
        'param7': param7,
        'command': command,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
        'confirmation': confirmation,
      };

  factory CommandLong.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandLong.mavlinkEncodedLength) {
      var len = CommandLong.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var param1 = data_.getFloat32(0, Endian.little);
    var param2 = data_.getFloat32(4, Endian.little);
    var param3 = data_.getFloat32(8, Endian.little);
    var param4 = data_.getFloat32(12, Endian.little);
    var param5 = data_.getFloat32(16, Endian.little);
    var param6 = data_.getFloat32(20, Endian.little);
    var param7 = data_.getFloat32(24, Endian.little);
    var command = data_.getUint16(28, Endian.little);
    var targetSystem = data_.getUint8(30);
    var targetComponent = data_.getUint8(31);
    var confirmation = data_.getUint8(32);

    return CommandLong(
        param1: param1,
        param2: param2,
        param3: param3,
        param4: param4,
        param5: param5,
        param6: param6,
        param7: param7,
        command: command,
        targetSystem: targetSystem,
        targetComponent: targetComponent,
        confirmation: confirmation);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setFloat32(0, param1, Endian.little);
    data_.setFloat32(4, param2, Endian.little);
    data_.setFloat32(8, param3, Endian.little);
    data_.setFloat32(12, param4, Endian.little);
    data_.setFloat32(16, param5, Endian.little);
    data_.setFloat32(20, param6, Endian.little);
    data_.setFloat32(24, param7, Endian.little);
    data_.setUint16(28, command, Endian.little);
    data_.setUint8(30, targetSystem);
    data_.setUint8(31, targetComponent);
    data_.setUint8(32, confirmation);
    return data_;
  }
}

/// Report status of a command. Includes feedback whether the command was
/// executed. The command microservice is documented at
/// https://mavlink.io/en/services/command.html
///
/// COMMAND_ACK
class CommandAck implements MavlinkMessage {
  static const int msgId = 77;

  static const int crcExtra = 143;

  static const int mavlinkEncodedLength = 10;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  CommandAck.fromJson(Map<String, dynamic> json)
      : command = json['command'],
        result = json['result'],
        progress = json['progress'],
        resultParam2 = json['resultParam2'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'];
  CommandAck copyWith({
    MavCmd? command,
    MavResult? result,
    uint8_t? progress,
    int32_t? resultParam2,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
  }) {
    return CommandAck(
      command: command ?? this.command,
      result: result ?? this.result,
      progress: progress ?? this.progress,
      resultParam2: resultParam2 ?? this.resultParam2,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'command': command,
        'result': result,
        'progress': progress,
        'resultParam2': resultParam2,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
      };

  factory CommandAck.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandAck.mavlinkEncodedLength) {
      var len = CommandAck.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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
  static const int msgId = 80;

  static const int crcExtra = 14;

  static const int mavlinkEncodedLength = 4;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

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

  CommandCancel.fromJson(Map<String, dynamic> json)
      : command = json['command'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'];
  CommandCancel copyWith({
    MavCmd? command,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
  }) {
    return CommandCancel(
      command: command ?? this.command,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'command': command,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
      };

  factory CommandCancel.parse(ByteData data_) {
    if (data_.lengthInBytes < CommandCancel.mavlinkEncodedLength) {
      var len = CommandCancel.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
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

/// File transfer protocol message: https://mavlink.io/en/services/ftp.html.
///
/// FILE_TRANSFER_PROTOCOL
class FileTransferProtocol implements MavlinkMessage {
  static const int msgId = 110;

  static const int crcExtra = 84;

  static const int mavlinkEncodedLength = 254;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// Network ID (0 for broadcast)
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_network
  final uint8_t targetNetwork;

  /// System ID (0 for broadcast)
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Component ID (0 for broadcast)
  ///
  /// MAVLink type: uint8_t
  ///
  /// target_component
  final uint8_t targetComponent;

  /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields. The content/format of this block is defined in https://mavlink.io/en/services/ftp.html.
  ///
  /// MAVLink type: uint8_t[251]
  ///
  /// payload
  final List<int8_t> payload;

  FileTransferProtocol({
    required this.targetNetwork,
    required this.targetSystem,
    required this.targetComponent,
    required this.payload,
  });

  FileTransferProtocol.fromJson(Map<String, dynamic> json)
      : targetNetwork = json['targetNetwork'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'],
        payload = List<int>.from(json['payload']);
  FileTransferProtocol copyWith({
    uint8_t? targetNetwork,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
    List<int8_t>? payload,
  }) {
    return FileTransferProtocol(
      targetNetwork: targetNetwork ?? this.targetNetwork,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
      payload: payload ?? this.payload,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'targetNetwork': targetNetwork,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
        'payload': payload,
      };

  factory FileTransferProtocol.parse(ByteData data_) {
    if (data_.lengthInBytes < FileTransferProtocol.mavlinkEncodedLength) {
      var len = FileTransferProtocol.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var targetNetwork = data_.getUint8(0);
    var targetSystem = data_.getUint8(1);
    var targetComponent = data_.getUint8(2);
    var payload = MavlinkMessage.asUint8List(data_, 3, 251);

    return FileTransferProtocol(
        targetNetwork: targetNetwork,
        targetSystem: targetSystem,
        targetComponent: targetComponent,
        payload: payload);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint8(0, targetNetwork);
    data_.setUint8(1, targetSystem);
    data_.setUint8(2, targetComponent);
    MavlinkMessage.setUint8List(data_, 3, payload);
    return data_;
  }
}

///
/// Time synchronization message.
/// The message is used for both timesync requests and responses.
/// The request is sent with `ts1=syncing component timestamp` and `tc1=0`, and may be broadcast or targeted to a specific system/component.
/// The response is sent with `ts1=syncing component timestamp` (mirror back unchanged), and `tc1=responding component timestamp`, with the `target_system` and `target_component` set to ids of the original request.
/// Systems can determine if they are receiving a request or response based on the value of `tc`.
/// If the response has `target_system==target_component==0` the remote system has not been updated to use the component IDs and cannot reliably timesync; the requestor may report an error.
/// Timestamps are UNIX Epoch time or time since system boot in nanoseconds (the timestamp format can be inferred by checking for the magnitude of the number; generally it doesn't matter as only the offset is used).
/// The message sequence is repeated numerous times with results being filtered/averaged to estimate the offset.
///
///
/// TIMESYNC
class Timesync implements MavlinkMessage {
  static const int msgId = 111;

  static const int crcExtra = 34;

  static const int mavlinkEncodedLength = 18;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// Time sync timestamp 1. Syncing: 0. Responding: Timestamp of responding component.
  ///
  /// MAVLink type: int64_t
  ///
  /// units: ns
  ///
  /// tc1
  final int64_t tc1;

  /// Time sync timestamp 2. Timestamp of syncing component (mirrored in response).
  ///
  /// MAVLink type: int64_t
  ///
  /// units: ns
  ///
  /// ts1
  final int64_t ts1;

  /// Target system id. Request: 0 (broadcast) or id of specific system. Response must contain system id of the requesting component.
  ///
  /// MAVLink type: uint8_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// target_system
  final uint8_t targetSystem;

  /// Target component id. Request: 0 (broadcast) or id of specific component. Response must contain component id of the requesting component.
  ///
  /// MAVLink type: uint8_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// target_component
  final uint8_t targetComponent;

  Timesync({
    required this.tc1,
    required this.ts1,
    required this.targetSystem,
    required this.targetComponent,
  });

  Timesync.fromJson(Map<String, dynamic> json)
      : tc1 = json['tc1'],
        ts1 = json['ts1'],
        targetSystem = json['targetSystem'],
        targetComponent = json['targetComponent'];
  Timesync copyWith({
    int64_t? tc1,
    int64_t? ts1,
    uint8_t? targetSystem,
    uint8_t? targetComponent,
  }) {
    return Timesync(
      tc1: tc1 ?? this.tc1,
      ts1: ts1 ?? this.ts1,
      targetSystem: targetSystem ?? this.targetSystem,
      targetComponent: targetComponent ?? this.targetComponent,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'tc1': tc1,
        'ts1': ts1,
        'targetSystem': targetSystem,
        'targetComponent': targetComponent,
      };

  factory Timesync.parse(ByteData data_) {
    if (data_.lengthInBytes < Timesync.mavlinkEncodedLength) {
      var len = Timesync.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var tc1 = data_.getInt64(0, Endian.little);
    var ts1 = data_.getInt64(8, Endian.little);
    var targetSystem = data_.getUint8(16);
    var targetComponent = data_.getUint8(17);

    return Timesync(
        tc1: tc1,
        ts1: ts1,
        targetSystem: targetSystem,
        targetComponent: targetComponent);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setInt64(0, tc1, Endian.little);
    data_.setInt64(8, ts1, Endian.little);
    data_.setUint8(16, targetSystem);
    data_.setUint8(17, targetComponent);
    return data_;
  }
}

///
/// The interval between messages for a particular MAVLink message ID.
/// This message is sent in response to the MAV_CMD_REQUEST_MESSAGE command with param1=244 (this message) and param2=message_id (the id of the message for which the interval is required).
/// It may also be sent in response to MAV_CMD_GET_MESSAGE_INTERVAL.
/// This interface replaces DATA_STREAM.
///
/// MESSAGE_INTERVAL
class MessageInterval implements MavlinkMessage {
  static const int msgId = 244;

  static const int crcExtra = 95;

  static const int mavlinkEncodedLength = 6;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
  ///
  /// MAVLink type: int32_t
  ///
  /// units: us
  ///
  /// interval_us
  final int32_t intervalUs;

  /// The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
  ///
  /// MAVLink type: uint16_t
  ///
  /// message_id
  final uint16_t messageId;

  MessageInterval({
    required this.intervalUs,
    required this.messageId,
  });

  MessageInterval.fromJson(Map<String, dynamic> json)
      : intervalUs = json['intervalUs'],
        messageId = json['messageId'];
  MessageInterval copyWith({
    int32_t? intervalUs,
    uint16_t? messageId,
  }) {
    return MessageInterval(
      intervalUs: intervalUs ?? this.intervalUs,
      messageId: messageId ?? this.messageId,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'intervalUs': intervalUs,
        'messageId': messageId,
      };

  factory MessageInterval.parse(ByteData data_) {
    if (data_.lengthInBytes < MessageInterval.mavlinkEncodedLength) {
      var len = MessageInterval.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var intervalUs = data_.getInt32(0, Endian.little);
    var messageId = data_.getUint16(4, Endian.little);

    return MessageInterval(intervalUs: intervalUs, messageId: messageId);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setInt32(0, intervalUs, Endian.little);
    data_.setUint16(4, messageId, Endian.little);
    return data_;
  }
}

/// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
///
/// NAMED_VALUE_FLOAT
class NamedValueFloat implements MavlinkMessage {
  static const int msgId = 251;

  static const int crcExtra = 170;

  static const int mavlinkEncodedLength = 18;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  String get nameAsString => convertMavlinkCharListToString(_name);
  List<char> get name => _name;

  /// Timestamp (time since system boot).
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: ms
  ///
  /// time_boot_ms
  final uint32_t timeBootMs;

  /// Floating point value
  ///
  /// MAVLink type: float
  ///
  /// value
  final float value;

  /// Name of the debug variable
  ///
  /// MAVLink type: char[10]
  ///
  /// name
  final List<char> _name;

  NamedValueFloat({
    required this.timeBootMs,
    required this.value,
    required name,
  }) : _name = name;

  NamedValueFloat.fromJson(Map<String, dynamic> json)
      : timeBootMs = json['timeBootMs'],
        value = json['value'],
        _name = convertStringtoMavlinkCharList(json['name'], length: 10);
  NamedValueFloat copyWith({
    uint32_t? timeBootMs,
    float? value,
    List<char>? name,
  }) {
    return NamedValueFloat(
      timeBootMs: timeBootMs ?? this.timeBootMs,
      value: value ?? this.value,
      name: name ?? this.name,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'timeBootMs': timeBootMs,
        'value': value,
        'name': _name,
      };

  factory NamedValueFloat.parse(ByteData data_) {
    if (data_.lengthInBytes < NamedValueFloat.mavlinkEncodedLength) {
      var len = NamedValueFloat.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var timeBootMs = data_.getUint32(0, Endian.little);
    var value = data_.getFloat32(4, Endian.little);
    var name = MavlinkMessage.asUint8List(data_, 8, 10);

    return NamedValueFloat(timeBootMs: timeBootMs, value: value, name: name);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint32(0, timeBootMs, Endian.little);
    data_.setFloat32(4, value, Endian.little);
    MavlinkMessage.setUint8List(data_, 8, name);
    return data_;
  }
}

/// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
///
/// NAMED_VALUE_INT
class NamedValueInt implements MavlinkMessage {
  static const int msgId = 252;

  static const int crcExtra = 44;

  static const int mavlinkEncodedLength = 18;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  String get nameAsString => convertMavlinkCharListToString(_name);
  List<char> get name => _name;

  /// Timestamp (time since system boot).
  ///
  /// MAVLink type: uint32_t
  ///
  /// units: ms
  ///
  /// time_boot_ms
  final uint32_t timeBootMs;

  /// Signed integer value
  ///
  /// MAVLink type: int32_t
  ///
  /// value
  final int32_t value;

  /// Name of the debug variable
  ///
  /// MAVLink type: char[10]
  ///
  /// name
  final List<char> _name;

  NamedValueInt({
    required this.timeBootMs,
    required this.value,
    required name,
  }) : _name = name;

  NamedValueInt.fromJson(Map<String, dynamic> json)
      : timeBootMs = json['timeBootMs'],
        value = json['value'],
        _name = convertStringtoMavlinkCharList(json['name'], length: 10);
  NamedValueInt copyWith({
    uint32_t? timeBootMs,
    int32_t? value,
    List<char>? name,
  }) {
    return NamedValueInt(
      timeBootMs: timeBootMs ?? this.timeBootMs,
      value: value ?? this.value,
      name: name ?? this.name,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'timeBootMs': timeBootMs,
        'value': value,
        'name': _name,
      };

  factory NamedValueInt.parse(ByteData data_) {
    if (data_.lengthInBytes < NamedValueInt.mavlinkEncodedLength) {
      var len = NamedValueInt.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var timeBootMs = data_.getUint32(0, Endian.little);
    var value = data_.getInt32(4, Endian.little);
    var name = MavlinkMessage.asUint8List(data_, 8, 10);

    return NamedValueInt(timeBootMs: timeBootMs, value: value, name: name);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint32(0, timeBootMs, Endian.little);
    data_.setInt32(4, value, Endian.little);
    MavlinkMessage.setUint8List(data_, 8, name);
    return data_;
  }
}

/// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
///
/// STATUSTEXT
class Statustext implements MavlinkMessage {
  static const int msgId = 253;

  static const int crcExtra = 83;

  static const int mavlinkEncodedLength = 54;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  String get textAsString => convertMavlinkCharListToString(_text);
  List<char> get text => _text;

  /// Severity of status. Relies on the definitions within RFC-5424.
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [MavSeverity]
  ///
  /// severity
  final MavSeverity severity;

  /// Status text message, without null termination character
  ///
  /// MAVLink type: char[50]
  ///
  /// text
  final List<char> _text;

  /// Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
  ///
  /// MAVLink type: uint16_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// id
  final uint16_t id;

  /// This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
  ///
  /// MAVLink type: uint8_t
  ///
  /// Extensions field for MAVLink 2.
  ///
  /// chunk_seq
  final uint8_t chunkSeq;

  Statustext({
    required this.severity,
    required text,
    required this.id,
    required this.chunkSeq,
  }) : _text = text;

  Statustext.fromJson(Map<String, dynamic> json)
      : severity = json['severity'],
        _text = convertStringtoMavlinkCharList(json['text'], length: 50),
        id = json['id'],
        chunkSeq = json['chunkSeq'];
  Statustext copyWith({
    MavSeverity? severity,
    List<char>? text,
    uint16_t? id,
    uint8_t? chunkSeq,
  }) {
    return Statustext(
      severity: severity ?? this.severity,
      text: text ?? this.text,
      id: id ?? this.id,
      chunkSeq: chunkSeq ?? this.chunkSeq,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'severity': severity,
        'text': _text,
        'id': id,
        'chunkSeq': chunkSeq,
      };

  factory Statustext.parse(ByteData data_) {
    if (data_.lengthInBytes < Statustext.mavlinkEncodedLength) {
      var len = Statustext.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var severity = data_.getUint8(0);
    var text = MavlinkMessage.asUint8List(data_, 1, 50);
    var id = data_.getUint16(51, Endian.little);
    var chunkSeq = data_.getUint8(53);

    return Statustext(
        severity: severity, text: text, id: id, chunkSeq: chunkSeq);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint8(0, severity);
    MavlinkMessage.setUint8List(data_, 1, text);
    data_.setUint16(51, id, Endian.little);
    data_.setUint8(53, chunkSeq);
    return data_;
  }
}

/// State input event
///
/// STATE_INPUT_EVENT
class StateInputEvent implements MavlinkMessage {
  static const int msgId = 1;

  static const int crcExtra = 95;

  static const int mavlinkEncodedLength = 1;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  /// the state input event
  ///
  /// MAVLink type: uint8_t
  ///
  /// enum: [StateInput]
  ///
  /// event
  final StateInput event;

  StateInputEvent({
    required this.event,
  });

  StateInputEvent.fromJson(Map<String, dynamic> json) : event = json['event'];
  StateInputEvent copyWith({
    StateInput? event,
  }) {
    return StateInputEvent(
      event: event ?? this.event,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'event': event,
      };

  factory StateInputEvent.parse(ByteData data_) {
    if (data_.lengthInBytes < StateInputEvent.mavlinkEncodedLength) {
      var len = StateInputEvent.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var event = data_.getUint8(0);

    return StateInputEvent(event: event);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    data_.setUint8(0, event);
    return data_;
  }
}

/// Indentifiying information about the EOS device
///
/// IDENTIFIER
class Identifier implements MavlinkMessage {
  static const int msgId = 2;

  static const int crcExtra = 88;

  static const int mavlinkEncodedLength = 114;

  @override
  int get mavlinkMessageId => msgId;

  @override
  int get mavlinkCrcExtra => crcExtra;

  String get particleIdAsString => convertMavlinkCharListToString(_particleId);
  List<char> get particleId => _particleId;
  String get nameAsString => convertMavlinkCharListToString(_name);
  List<char> get name => _name;
  String get siteFriendlyNameAsString =>
      convertMavlinkCharListToString(_siteFriendlyName);
  List<char> get siteFriendlyName => _siteFriendlyName;
  String get siteNameAsString => convertMavlinkCharListToString(_siteName);
  List<char> get siteName => _siteName;

  /// Particle ID of device. Unique and unchangable
  ///
  /// MAVLink type: char[24]
  ///
  /// particle_id
  final List<char> _particleId;

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
  /// MAVLink type: char[20]
  ///
  /// name
  final List<char> _name;

  /// Friendly name for the site it's at,
  /// i.e. "57 Rock West"
  ///
  /// MAVLink type: char[30]
  ///
  /// site_friendly_name
  final List<char> _siteFriendlyName;

  /// Name of the site where the scanner is located,
  /// i.e. "Gainesville Plant"
  ///
  /// MAVLink type: char[30]
  ///
  /// site_name
  final List<char> _siteName;

  Identifier({
    required particleId,
    required this.localIp,
    required this.mac,
    required name,
    required siteFriendlyName,
    required siteName,
  })  : _particleId = particleId,
        _name = name,
        _siteFriendlyName = siteFriendlyName,
        _siteName = siteName;

  Identifier.fromJson(Map<String, dynamic> json)
      : _particleId =
            convertStringtoMavlinkCharList(json['particleId'], length: 24),
        localIp = List<int>.from(json['localIp']),
        mac = List<int>.from(json['mac']),
        _name = convertStringtoMavlinkCharList(json['name'], length: 20),
        _siteFriendlyName = convertStringtoMavlinkCharList(
            json['siteFriendlyName'],
            length: 30),
        _siteName =
            convertStringtoMavlinkCharList(json['siteName'], length: 30);
  Identifier copyWith({
    List<char>? particleId,
    List<int8_t>? localIp,
    List<int8_t>? mac,
    List<char>? name,
    List<char>? siteFriendlyName,
    List<char>? siteName,
  }) {
    return Identifier(
      particleId: particleId ?? this.particleId,
      localIp: localIp ?? this.localIp,
      mac: mac ?? this.mac,
      name: name ?? this.name,
      siteFriendlyName: siteFriendlyName ?? this.siteFriendlyName,
      siteName: siteName ?? this.siteName,
    );
  }

  @override
  Map<String, dynamic> toJson() => {
        'msgId': msgId,
        'particleId': _particleId,
        'localIp': localIp,
        'mac': mac,
        'name': _name,
        'siteFriendlyName': _siteFriendlyName,
        'siteName': _siteName,
      };

  factory Identifier.parse(ByteData data_) {
    if (data_.lengthInBytes < Identifier.mavlinkEncodedLength) {
      var len = Identifier.mavlinkEncodedLength - data_.lengthInBytes;
      var d = data_.buffer.asUint8List().sublist(0, data_.lengthInBytes) +
          List<int>.filled(len, 0);
      data_ = Uint8List.fromList(d).buffer.asByteData();
    }
    var particleId = MavlinkMessage.asUint8List(data_, 0, 24);
    var localIp = MavlinkMessage.asUint8List(data_, 24, 4);
    var mac = MavlinkMessage.asUint8List(data_, 28, 6);
    var name = MavlinkMessage.asUint8List(data_, 34, 20);
    var siteFriendlyName = MavlinkMessage.asUint8List(data_, 54, 30);
    var siteName = MavlinkMessage.asUint8List(data_, 84, 30);

    return Identifier(
        particleId: particleId,
        localIp: localIp,
        mac: mac,
        name: name,
        siteFriendlyName: siteFriendlyName,
        siteName: siteName);
  }

  @override
  ByteData serialize() {
    var data_ = ByteData(mavlinkEncodedLength);
    MavlinkMessage.setUint8List(data_, 0, particleId);
    MavlinkMessage.setUint8List(data_, 24, localIp);
    MavlinkMessage.setUint8List(data_, 28, mac);
    MavlinkMessage.setUint8List(data_, 34, name);
    MavlinkMessage.setUint8List(data_, 54, siteFriendlyName);
    MavlinkMessage.setUint8List(data_, 84, siteName);
    return data_;
  }
}

class MavlinkDialectTitan implements MavlinkDialect {
  static const int mavlinkVersion = 3;

  @override
  int get version => mavlinkVersion;

  @override
  MavlinkMessage? parse(int messageID, ByteData data) {
    switch (messageID) {
      case 0:
        return Heartbeat.parse(data);
      case 5:
        return ChangeOperatorControl.parse(data);
      case 6:
        return ChangeOperatorControlAck.parse(data);
      case 300:
        return ProtocolVersion.parse(data);
      case 24:
        return GpsRawInt.parse(data);
      case 39:
        return MissionItem.parse(data);
      case 75:
        return CommandInt.parse(data);
      case 76:
        return CommandLong.parse(data);
      case 77:
        return CommandAck.parse(data);
      case 80:
        return CommandCancel.parse(data);
      case 110:
        return FileTransferProtocol.parse(data);
      case 111:
        return Timesync.parse(data);
      case 244:
        return MessageInterval.parse(data);
      case 251:
        return NamedValueFloat.parse(data);
      case 252:
        return NamedValueInt.parse(data);
      case 253:
        return Statustext.parse(data);
      case 1:
        return StateInputEvent.parse(data);
      case 2:
        return Identifier.parse(data);
      default:
        return null;
    }
  }

  @override
  int crcExtra(int messageID) {
    switch (messageID) {
      case 0:
        return Heartbeat.crcExtra;
      case 5:
        return ChangeOperatorControl.crcExtra;
      case 6:
        return ChangeOperatorControlAck.crcExtra;
      case 300:
        return ProtocolVersion.crcExtra;
      case 24:
        return GpsRawInt.crcExtra;
      case 39:
        return MissionItem.crcExtra;
      case 75:
        return CommandInt.crcExtra;
      case 76:
        return CommandLong.crcExtra;
      case 77:
        return CommandAck.crcExtra;
      case 80:
        return CommandCancel.crcExtra;
      case 110:
        return FileTransferProtocol.crcExtra;
      case 111:
        return Timesync.crcExtra;
      case 244:
        return MessageInterval.crcExtra;
      case 251:
        return NamedValueFloat.crcExtra;
      case 252:
        return NamedValueInt.crcExtra;
      case 253:
        return Statustext.crcExtra;
      case 1:
        return StateInputEvent.crcExtra;
      case 2:
        return Identifier.crcExtra;
      default:
        return -1;
    }
  }
}

String convertMavlinkCharListToString(List<int>? charList) {
  if (charList == null) {
    return "";
  }
  List<int> trimmedName = [];
  for (int character in charList) {
    if (character != 0x00) {
      trimmedName.add(character);
    }
  }
  return ascii.decode(trimmedName);
}

Uint8List convertStringtoMavlinkCharList(String inputString, {int? length}) {
  // Use passed length if it's there otherwise just use size of the input string
  length = length ?? inputString.length;
  Uint8List charList = Uint8List(length);
  const asciiEncoder = AsciiEncoder();
  Uint8List stringAsList = asciiEncoder.convert(inputString);

  for (var i = 0; i < stringAsList.length; i++) {
    charList[i] = stringAsList[i];
  }
  return charList;
}
