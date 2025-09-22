
"use strict";

let StopRTControl = require('./StopRTControl.js')
let WriteDataRT = require('./WriteDataRT.js')
let GetRTControlOutputDataList = require('./GetRTControlOutputDataList.js')
let ReadDataRT = require('./ReadDataRT.js')
let SetRTControlOutput = require('./SetRTControlOutput.js')
let SetVelJRT = require('./SetVelJRT.js')
let GetRTControlInputVersionList = require('./GetRTControlInputVersionList.js')
let ConnectRTControl = require('./ConnectRTControl.js')
let GetRTControlInputDataList = require('./GetRTControlInputDataList.js')
let SetRTControlInput = require('./SetRTControlInput.js')
let DisconnectRTControl = require('./DisconnectRTControl.js')
let StartRTControl = require('./StartRTControl.js')
let GetRTControlOutputVersionList = require('./GetRTControlOutputVersionList.js')
let SetAccJRT = require('./SetAccJRT.js')
let SetAccXRT = require('./SetAccXRT.js')
let SetVelXRT = require('./SetVelXRT.js')

module.exports = {
  StopRTControl: StopRTControl,
  WriteDataRT: WriteDataRT,
  GetRTControlOutputDataList: GetRTControlOutputDataList,
  ReadDataRT: ReadDataRT,
  SetRTControlOutput: SetRTControlOutput,
  SetVelJRT: SetVelJRT,
  GetRTControlInputVersionList: GetRTControlInputVersionList,
  ConnectRTControl: ConnectRTControl,
  GetRTControlInputDataList: GetRTControlInputDataList,
  SetRTControlInput: SetRTControlInput,
  DisconnectRTControl: DisconnectRTControl,
  StartRTControl: StartRTControl,
  GetRTControlOutputVersionList: GetRTControlOutputVersionList,
  SetAccJRT: SetAccJRT,
  SetAccXRT: SetAccXRT,
  SetVelXRT: SetVelXRT,
};
