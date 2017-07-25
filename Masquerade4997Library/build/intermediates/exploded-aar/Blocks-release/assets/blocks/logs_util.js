/**
 * @fileoverview Logs utilities.
 * @author lizlooney@google.com (Liz Looney)
 */

/**
 * Fetches the log and calls the callback
 */
function fetchLogContent(callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    fetchLogContentViaBlocksIO(callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    fetchLogContentViaHttp(callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    fetchLogContentViaFile(callback);
  }
}

//..........................................................................
// Code used when html/js is within the WebView component within the
// Android app.

function fetchLogContentViaBlocksIO(callback) {
  var logContent = blocksIO.fetchLogContent();
  if (logContent) {
    callback(logContent, '');
  } else {
    callback(null, 'Fetch log failed.');
  }
}

//..........................................................................
// Code used when html/js is in a browser, loaded as an http:// URL.
// The following are generated dynamically in ProgrammingModeServer.fetchJavaScriptForServer():
// URI_FETCH_LOG

function fetchLogContentViaHttp(callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('POST', URI_FETCH_LOG, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var logContent = xhr.responseText;
        callback(logContent, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch log failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send();
}

//..........................................................................
// Code used when html/js is in a browser, loaded as a file:// URL.

function fetchLogContentViaFile(callback) {
  var fakeLogContent =
      '01-29 23:36:22.611  3840  3924 I RobotCore: BlocksOpMode -  "CountToTen" - main/LinearOpMode main - runOpMode - start\n' +
      '01-29 23:36:34.191  3840  3924 I RobotCore: BlocksOpMode -  "CountToTen" - main/LinearOpMode main - runOpMode - end - no InterruptedException';
  callback(fakelogContent, '');
}

