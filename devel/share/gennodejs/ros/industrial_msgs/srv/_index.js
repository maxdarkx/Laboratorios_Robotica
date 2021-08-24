
"use strict";

let SetRemoteLoggerLevel = require('./SetRemoteLoggerLevel.js')
let CmdJointTrajectory = require('./CmdJointTrajectory.js')
let SetDrivePower = require('./SetDrivePower.js')
let GetRobotInfo = require('./GetRobotInfo.js')
let StartMotion = require('./StartMotion.js')
let StopMotion = require('./StopMotion.js')

module.exports = {
  SetRemoteLoggerLevel: SetRemoteLoggerLevel,
  CmdJointTrajectory: CmdJointTrajectory,
  SetDrivePower: SetDrivePower,
  GetRobotInfo: GetRobotInfo,
  StartMotion: StartMotion,
  StopMotion: StopMotion,
};
