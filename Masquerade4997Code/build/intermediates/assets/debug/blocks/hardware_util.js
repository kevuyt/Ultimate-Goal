/**
 * @fileoverview Hardware utilities.
 * @author lizlooney@google.com (Liz Looney)
 */

// TODO(lizlooney): This file is misnamed. It includes utilities not related to hardware.

/**
 * Fetches the JavaScript code related to the hardware in the active configuration and calls the
 * callback.
 */
function fetchJavaScriptForHardware(callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    fetchJavaScriptForHardwareViaBlocksIO(callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    fetchJavaScriptForHardwareViaHttp(callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    fetchJavaScriptForHardwareViaFile(callback);
  }
}

/**
 * Sends a ping request and calls the callback.
 */
function sendPing(name, callback) {
  if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    sendPingViaHttp(name, callback);
  } else {
    callback(false);
  }
}

//..........................................................................
// Code used when html/js is within the WebView component within the
// Android app.

function fetchJavaScriptForHardwareViaBlocksIO(callback) {
  var jsHardware = blocksIO.fetchJavaScriptForHardware();
  if (jsHardware) {
    callback(jsHardware, '');
  } else {
    callback(null, 'Fetch JavaScript for Hardware failed.');
  }
}

//..........................................................................
// Code used when html/js is in a browser, loaded as a http:// URL.

// The following are generated dynamically in ProgrammingModeServer.fetchJavaScriptForServer():
// URI_HARDWARE
// URI_PING
// PARAM_NAME

function fetchJavaScriptForHardwareViaHttp(callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', URI_HARDWARE, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var jsHardware = xhr.responseText;
        callback(jsHardware, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch JavaScript for Hardware failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send();
}

function sendPingViaHttp(name, callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('POST', URI_PING, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        callback(true);
      } else {
        callback(false);
      }
    }
  };
  var params = PARAM_NAME + '=' + encodeURIComponent(name);
  xhr.send(params);
}

//..........................................................................
// Code used when html/js is in a browser, loaded as a file:// URL.

function fetchJavaScriptForHardwareViaFile(callback) {
  var jsHardware =
      "function isValidProjectName(projectName) {\n" +
      "  if (projectName) {\n" +
      "    return /^[a-zA-Z0-9]+$/.test(projectName);\n" +
      "  }\n" +
      "  return false;\n" +
      "}\n" +
      "function isValidSoundName(soundName) {\n" +
      "  if (soundName) {\n" +
      "    return /^[a-zA-Z0-9]+$/.test(soundName);\n" +
      "  }\n" +
      "  return false;\n" +
      "}\n" +
      "function createGamepadDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['gamepad1', 'gamepad1'],\n" +
      "      ['gamepad2', 'gamepad2'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "var accelerationIdentifier = 'accelerationAccess';\n" +
      "var androidAccelerometerIdentifier = 'androidAccelerometerAccess';\n" +
      "var androidGyroscopeIdentifier = 'androidGyroscopeAccess';\n" +
      "var androidOrientationIdentifier = 'androidOrientationAccess';\n" +
      "var androidSoundPoolIdentifier = 'androidSoundPoolAccess';\n" +
      "var androidTextToSpeechIdentifier = 'androidTextToSpeechAccess';\n" +
      "var angularVelocityIdentifier = 'angularVelocityAccess';\n" +
      "var bno055imuParametersIdentifier = 'bno055imuParametersAccess';\n" +
      "var colorIdentifier = 'colorAccess';\n" +
      "var dbgLogIdentifier = 'dbgLogAccess';\n" +
      "var elapsedTimeIdentifier = 'elapsedTimeAccess';\n" +
      "var linearOpModeIdentifier = 'linearOpMode';\n" +
      "var magneticFluxIdentifier = 'magneticFluxAccess';\n" +
      "var matrixFIdentifier = 'matrixFAccess';\n" +
      "var miscIdentifier = 'miscAccess';\n" +
      "var navigationIdentifier = 'navigationAccess';\n" +
      "var openGLMatrixIdentifier = 'openGLMatrixAccess';\n" +
      "var orientationIdentifier = 'orientationAccess';\n" +
      "var positionIdentifier = 'positionAccess';\n" +
      "var quaternionIdentifier = 'quaternionAccess';\n" +
      "var systemIdentifier = 'systemAccess';\n" +
      "var telemetryIdentifier = 'telemetry';\n" +
      "var temperatureIdentifier = 'temperatureAccess';\n" +
      "var vectorFIdentifier = 'vectorFAccess';\n" +
      "var velocityIdentifier = 'velocityAccess';\n" +
      "var vuforiaIdentifier = 'vuforiaAccess';\n" +
      "var vuforiaLocalizerIdentifier = 'vuforiaLocalizerAccess';\n" +
      "var vuforiaLocalizerParametersIdentifier = 'vuforiaLocalizerParametersAccess';\n" +
      "var vuforiaTrackableIdentifier = 'vuforiaTrackableAccess';\n" +
      "var vuforiaTrackableDefaultListenerIdentifier = 'vuforiaTrackableDefaultListenerAccess';\n" +
      "var vuforiaTrackablesIdentifier = 'vuforiaTrackablesAccess';\n" +
      "function createAccelerationSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createAnalogInputDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createAnalogOutputDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createBNO055IMUSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return new Blockly.FieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createColorSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createCompassSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createCRServoDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createDcMotorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['left_drive', 'left_drive'],\n" +
      "      ['right_drive', 'right_drive'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createDigitalChannelDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createGyroSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createIrSeekerSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createLedDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createLightSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createLynxI2cColorRangeSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createMrI2cCompassSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createMrI2cRangeSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createOpticalDistanceSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['sensor_EOPD', 'sensor_EOPD'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createServoDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['arm', 'arm'],\n" +
      "      ['claw', 'claw'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createServoControllerDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['servo_controller', 'servo_controller'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createTouchSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "      ['touch_sensor', 'touch_sensor'],\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createUltrasonicSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function createVoltageSensorDropdown() {\n" +
      "  var CHOICES = [\n" +
      "  ];\n" +
      "  return createFieldDropdown(CHOICES);\n" +
      "}\n" +
      "function addReservedWordsForHardware() {\n" +
      "  Blockly.JavaScript.addReservedWords('accelerationAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('androidAccelerometerAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('androidGyroscopeAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('androidOrientationAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('androidSoundPoolAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('androidTextToSpeechAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('angularVelocityAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('blocksOpMode');\n" +
      "  Blockly.JavaScript.addReservedWords('colorAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('dbgLogAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('elapsedTimeAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('gamepad1');\n" +
      "  Blockly.JavaScript.addReservedWords('gamepad2');\n" +
      "  Blockly.JavaScript.addReservedWords('linearOpMode');\n" +
      "  Blockly.JavaScript.addReservedWords('magneticFluxAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('matrixFAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('miscAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('openGLMatrixAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('orientationAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('positionAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('quaternionAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('systemAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('telemetry');\n" +
      "  Blockly.JavaScript.addReservedWords('vectorFAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('velocityAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaLocalizerAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaLocalizerParametersAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaTrackableAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaTrackableDefaultListenerAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('vuforiaTrackablesAccess');\n" +
      "  Blockly.JavaScript.addReservedWords('left_drive');\n" +
      "  Blockly.JavaScript.addReservedWords('right_drive');\n" +
      "  Blockly.JavaScript.addReservedWords('arm');\n" +
      "  Blockly.JavaScript.addReservedWords('claw');\n" +
      "  Blockly.JavaScript.addReservedWords('servo_controller');\n" +
      "  Blockly.JavaScript.addReservedWords('touch_sensor');\n" +
      "  Blockly.JavaScript.addReservedWords('sensor_EOPD');\n" +
      "}\n";
  callback(jsHardware, '');
}
