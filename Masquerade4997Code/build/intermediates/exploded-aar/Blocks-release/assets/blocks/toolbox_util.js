/**
 * @fileoverview Toolbox utilities.
 * @author lizlooney@google.com (Liz Looney)
 */

/**
 * Fetches the toolbox (as xml) and calls the callback.
 */
function fetchToolbox(callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    fetchToolboxViaBlocksIO(callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    fetchToolboxViaHttp(callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    fetchToolboxViaFile(callback);
  }
}

function addToolboxIcons(workspace) {
  addToolboxIconsForChildren(workspace.toolbox_.tree_.getChildren());
}

function addToolboxIconsForChildren(children) {
  for (var i = 0, child; child = children[i]; i++) {
    if (child.getChildCount() > 0) {
      addToolboxIconsForChildren(child.getChildren());
    } else {
      child.setIconClass('toolbox-node-icon ' + child.getText() + '-icon');
    }
  }
}

//..........................................................................
// Code used when html/js is within the WebView component within the
// Android app.

function fetchToolboxViaBlocksIO(callback) {
  var xmlToolbox = blocksIO.fetchToolbox();
  if (xmlToolbox) {
    callback(xmlToolbox, '');
  } else {
    callback(null, 'Fetch toolbox failed.');
  }
}

//..........................................................................
// Code used when html/js is in a browser, loaded as an http:// URL.

// The following are generated dynamically in ProgrammingModeServer.fetchJavaScriptForServer():
// URI_TOOLBOX

function fetchToolboxViaHttp(callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', URI_TOOLBOX, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var xmlToolbox = xhr.responseText;
        callback(xmlToolbox, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch toolbox failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send();
}

//..........................................................................
// Code used when html/js is in a browser, loaded as a file:// URL.

function fetchToolboxViaFile(callback) {
  var xmlToolbox =
      '<xml id="toolbox" style="display: none">' +
      '<category name="LinearOpMode">' +
      '  <block type="linearOpMode_waitForStart"></block>' +
      '  <block type="linearOpMode_idle"></block>' +
      '  <block type="linearOpMode_sleep_Number">' +
      '    <value name="MILLISECONDS"><shadow type="math_number">' +
      '      <field name="NUM">1000</field>' +
      '    </shadow></value>' +
      '  </block>' +
      '  <block type="linearOpMode_opModeIsActive"></block>' +
      '  <block type="linearOpMode_isStarted"></block>' +
      '  <block type="linearOpMode_isStopRequested"></block>' +
      '  <block type="linearOpMode_getRuntime_Number"></block>' +
      '</category>' +
      '<category name="Telemetry">' +
      '  <block type="telemetry_addNumericData">' +
      '    <value name="KEY"><shadow type="text"><field name="TEXT">key</field></shadow></value>' +
      '    <value name="NUMBER"><shadow type="math_number"><field name="NUM">123</field></shadow></value>' +
      '  </block>' +
      '  <block type="telemetry_addTextData">' +
      '    <value name="KEY"><shadow type="text"><field name="TEXT">key</field></shadow></value>' +
      '    <value name="TEXT"><shadow type="text"><field name="TEXT">text</field></shadow></value>' +
      '  </block>' +
      '  <block type="telemetry_update">' +
      '  </block>' +
      '</category>' +
      '<category name="Miscellaneous" colour="200">' +
      '  <block type="comment">' +
      '    <field name="COMMENT">Enter your comment here!</field>' +
      '  </block>' +
      '</category>' +
      '</xml>';
  callback(xmlToolbox, '');
}
