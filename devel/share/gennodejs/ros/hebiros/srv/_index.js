
"use strict";

let AddGroupFromNamesSrv = require('./AddGroupFromNamesSrv.js')
let ModelFkSrv = require('./ModelFkSrv.js')
let SizeSrv = require('./SizeSrv.js')
let SetFeedbackFrequencySrv = require('./SetFeedbackFrequencySrv.js')
let AddModelFromURDFSrv = require('./AddModelFromURDFSrv.js')
let SetCommandLifetimeSrv = require('./SetCommandLifetimeSrv.js')
let EntryListSrv = require('./EntryListSrv.js')
let AddGroupFromURDFSrv = require('./AddGroupFromURDFSrv.js')
let SendCommandWithAcknowledgementSrv = require('./SendCommandWithAcknowledgementSrv.js')

module.exports = {
  AddGroupFromNamesSrv: AddGroupFromNamesSrv,
  ModelFkSrv: ModelFkSrv,
  SizeSrv: SizeSrv,
  SetFeedbackFrequencySrv: SetFeedbackFrequencySrv,
  AddModelFromURDFSrv: AddModelFromURDFSrv,
  SetCommandLifetimeSrv: SetCommandLifetimeSrv,
  EntryListSrv: EntryListSrv,
  AddGroupFromURDFSrv: AddGroupFromURDFSrv,
  SendCommandWithAcknowledgementSrv: SendCommandWithAcknowledgementSrv,
};
