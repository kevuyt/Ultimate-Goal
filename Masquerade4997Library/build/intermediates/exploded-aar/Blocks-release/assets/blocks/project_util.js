/**
 * @fileoverview Project utilities.
 * @author lizlooney@google.com (Liz Looney)
 */

/**
 * Fetches the list of projects (as json) and calls the callback.
 */
function fetchProjects(callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    fetchProjectsViaBlocksIO(callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    fetchProjectsViaHttp(callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    fetchProjectsViaFile(callback);
  }
}

/**
 * Opens the project with the given name.
 */
function openProjectBlocks(projectName) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    openProjectBlocksViaBlocksIO(projectName);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    openProjectBlocksViaHttp(projectName);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    openProjectBlocksViaFile(projectName);
  }
}

/**
 * Creates the blocks for a new project.
 */
function createBlocks() {
  var blocksContent =
      '<xml xmlns="http://www.w3.org/1999/xhtml">\n' +
      '  <block type="procedures_defnoreturn" deletable="false" editable="false" x="50" y="50">\n' +
      '    <field name="NAME">runOpMode</field>\n' +
      '    <comment pinned="true" h="80" w="160">This function is executed when this Op Mode is selected from the Driver Station.</comment>\n' +
      '    <statement name="STACK">\n' +
      '      <block type="comment">\n' +
      '        <field name="COMMENT">Put initialization blocks here.</field>\n' +
      '        <next>\n' +
      '          <block type="linearOpMode_waitForStart">\n' +
      '            <next>\n' +
      '              <block type="comment">\n' +
      '                <field name="COMMENT">Put run blocks here.</field>\n' +
      '                <next>\n' +
      '                  <block type="controls_whileUntil">\n' +
      '                    <field name="MODE">WHILE</field>\n' +
      '                    <value name="BOOL">\n' +
      '                      <block type="linearOpMode_opModeIsActive">\n' +
      '                      </block>\n' +
      '                    </value>\n' +
      '                    <statement name="DO">\n' +
      '                      <block type="comment">\n' +
      '                        <field name="COMMENT">Put loop blocks here.</field>\n' +
      '                        <next>\n' +
      '                          <block type="telemetry_update">\n' +
      '                          </block>\n' +
      '                        </next>\n' +
      '                      </block>\n' +
      '                    </statement>\n' +
      '                  </block>\n' +
      '                </next>\n' +
      '              </block>\n' +
      '            </next>\n' +
      '          </block>\n' +
      '        </next>\n' +
      '      </block>\n' +
      '    </statement>\n' +
      '  </block>\n' +
      '</xml>\n';
  return blocksContent;
}

/**
 * Creates the javascript for a new project.
 */
function createJavaScript() {
  var jsFileContent =
      'function runOpMode() {\n' +
      '  linearOpMode.waitForStart();\n' +
      '  while (linearOpMode.opModeIsActive()) {\n' +
      '    telemetry.update();\n' +
      '  }\n' +
      '}\n';
  return jsFileContent;
}

/**
 * Fetches the blocks of an existing project and calls the callback
 */
function fetchBlkFileContent(projectName, callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    fetchBlkFileContentViaBlocksIO(projectName, callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    fetchBlkFileContentViaHttp(projectName, callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    fetchBlkFileContentViaFile(projectName, callback);
  }
}

function saveProject(projectName, blkContent, jsFileContent, flavor, group, callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    saveProjectViaBlocksIO(projectName, blkContent, jsFileContent, flavor, group, callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    saveProjectViaHttp(projectName, blkContent, jsFileContent, flavor, group, callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    saveProjectViaFile(projectName, blkContent, jsFileContent, flavor, group, callback);
  }
}

function renameProject(oldProjectName, newProjectName, callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    renameProjectViaBlocksIO(oldProjectName, newProjectName, callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    renameProjectViaHttp(oldProjectName, newProjectName, callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    renameProjectViaFile(oldProjectName, newProjectName, callback);
  }
}

function copyProject(oldProjectName, newProjectName, callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    copyProjectViaBlocksIO(oldProjectName, newProjectName, callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    copyProjectViaHttp(oldProjectName, newProjectName, callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    copyProjectViaFile(oldProjectName, newProjectName, callback);
  }
}

function deleteProjects(starDelimitedProjectNames, callback) {
  if (typeof blocksIO !== 'undefined') {
    // html/js is within the WebView component within the Android app.
    deleteProjectsViaBlocksIO(starDelimitedProjectNames, callback);
  } else if (window.location.protocol === 'http:') {
    // html/js is in a browser, loaded as an http:// URL.
    deleteProjectsViaHttp(starDelimitedProjectNames, callback);
  } else if (window.location.protocol === 'file:') {
    // html/js is in a browser, loaded as a file:// URL.
    deleteProjectsViaFile(starDelimitedProjectNames, callback);
  }
}

//..........................................................................
// Code used when html/js is within the WebView component within the
// Android app.

function fetchProjectsViaBlocksIO(callback) {
  var jsonProjects = blocksIO.fetchProjects();
  if (jsonProjects) {
    callback(jsonProjects, '');
  } else {
    callback(null, 'Fetch projects failed.');
  }
}

function openProjectBlocksViaBlocksIO(projectName) {
  blocksIO.openProjectBlocks(projectName);
}

function fetchBlkFileContentViaBlocksIO(projectName, callback) {
  var blkFileContent = blocksIO.fetchBlkFileContent(projectName);
  if (blkFileContent) {
    callback(blkFileContent, '');
  } else {
    callback(null, 'Fetch blocks failed.');
  }
}

function saveProjectViaBlocksIO(projectName, blkContent, jsFileContent, flavor, group, callback) {
  var success = blocksIO.saveProject(projectName, blkContent, jsFileContent, flavor, group);
  if (success) {
    callback(true, '');
  } else {
    // TODO(lizlooney): Provide more information about the error.
    callback(false, 'Save project failed.');
  }
}

function renameProjectViaBlocksIO(oldProjectName, newProjectName, callback) {
  var success = blocksIO.renameProject(oldProjectName, newProjectName);
  if (success) {
    callback(true, '');
  } else {
    // TODO(lizlooney): Provide more information about the error.
    callback(false, 'Rename project failed.');
  }
}

function copyProjectViaBlocksIO(oldProjectName, newProjectName, callback) {
  var success = blocksIO.copyProject(oldProjectName, newProjectName);
  if (success) {
    callback(true, '');
  } else {
    // TODO(lizlooney): Provide more information about the error.
    callback(false, 'Copy project failed.');
  }
}

function deleteProjectsViaBlocksIO(starDelimitedProjectNames, callback) {
  var success = blocksIO.deleteProjects(starDelimitedProjectNames);
  if (success) {
    callback(true, '');
  } else {
    // TODO(lizlooney): Provide more information about the error.
    callback(false, 'Delete projects failed.');
  }
}

//..........................................................................
// Code used when html/js is in a browser, loaded as an http:// URL.

// The following are generated dynamically in ProgrammingModeServer.fetchJavaScriptForServer():
// URI_LIST_PROJECTS
// URI_FETCH_BLK
// URI_SAVE_PROJECT
// URI_RENAME_PROJECT
// URI_COPY_PROJECT
// URI_DELETE_PROJECTS
// PARAM_NAME
// PARAM_NEW_NAME
// PARAM_BLK
// PARAM_JS
// PARAM_FLAVOR
// PARAM_GROUP
// PARAM_CONTENT

function fetchProjectsViaHttp(callback) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', URI_LIST_PROJECTS, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var jsonProjects = xhr.responseText;
        callback(jsonProjects, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch projects failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send();
}

function openProjectBlocksViaHttp(projectName) {
  // Go to FtcBlocks.html?project=<projectName>.
  window.location.href = 'FtcBlocks.html?project=' + encodeURIComponent(projectName);
}

function fetchBlkFileContentViaHttp(projectName, callback) {
  var xhr = new XMLHttpRequest();
  var params = PARAM_NAME + '=' + encodeURIComponent(projectName);
  xhr.open('POST', URI_FETCH_BLK, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        var blkFileContent = xhr.responseText;
        callback(blkFileContent, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(null, 'Fetch blocks failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send(params);
}

function saveProjectViaHttp(projectName, blkContent, jsFileContent, flavor, group, callback) {
  var xhr = new XMLHttpRequest();
  var params = PARAM_NAME + '=' + encodeURIComponent(projectName) +
      '&' + PARAM_BLK + '=' + encodeURIComponent(blkContent) +
      '&' + PARAM_JS + '=' + encodeURIComponent(jsFileContent);
  if (flavor != null) {
    params += '&' + PARAM_FLAVOR + '=' + encodeURIComponent(flavor);
  }
  if (group != null) {
    params += '&' + PARAM_GROUP + '=' + encodeURIComponent(group);
  }
  xhr.open('POST', URI_SAVE_PROJECT, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        callback(true, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(false, 'Save project failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send(params);
}

function renameProjectViaHttp(oldProjectName, newProjectName, callback) {
  var xhr = new XMLHttpRequest();
  var params = PARAM_NAME + '=' + encodeURIComponent(oldProjectName) +
      '&' + PARAM_NEW_NAME + '=' + encodeURIComponent(newProjectName);
  xhr.open('POST', URI_RENAME_PROJECT, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        callback(true, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(false, 'Rename project failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send(params);
}

function copyProjectViaHttp(oldProjectName, newProjectName, callback) {
  var xhr = new XMLHttpRequest();
  var params = PARAM_NAME + '=' + encodeURIComponent(oldProjectName) +
      '&' + PARAM_NEW_NAME + '=' + encodeURIComponent(newProjectName);
  xhr.open('POST', URI_COPY_PROJECT, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        callback(true, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(false, 'Copy project failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send(params);
}

function deleteProjectsViaHttp(starDelimitedProjectNames, callback) {
  var xhr = new XMLHttpRequest();
  var params = PARAM_NAME + '=' + encodeURIComponent(starDelimitedProjectNames);
  xhr.open('POST', URI_DELETE_PROJECTS, true);
  xhr.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        callback(true, '');
      } else {
        // TODO(lizlooney): Use specific error messages for various xhr.status values.
        callback(false, 'Delete projects failed. Error code ' + xhr.status + '. ' + xhr.statusText);
      }
    }
  };
  xhr.send(params);
}

//..........................................................................
// Code used when html/js is in a browser, loaded as a file:// URL.

function fetchProjectsViaFile(callback) {
  var jsonProjects = '[' +
      '{ "name": "Autonomous", "escapedName": "Autonomous", "dateModifiedMillis": 1464391007000},' +
      '{ "name": "LineFollow", "escapedName": "LineFollow", "dateModifiedMillis":1464591007000},' +
      '{ "name": "TeleBot", "escapedName": "TeleBot", "dateModifiedMillis":1464191007000},' +
      '{ "name": "Test&reg;", "escapedName": "Test&amp;reg;", "dateModifiedMillis":1464191007000}' +
      ']';
  callback(jsonProjects, '');
}

function openProjectBlocksViaFile(projectName) {
  // Go to FtcBlocks.html?project=<projectName>.
  window.location.href = 'FtcBlocks.html?project=' + encodeURIComponent(projectName);
}

function fetchBlkFileContentViaFile(projectName, callback) {
  var fakeBlkFileContent = createBlocks();
  fakeBlkFileContent += '<xml><OpModeMeta flavor="autonomous" group="abc"></OpModeMeta></xml>';
  callback(fakeBlkFileContent, '');
}

function saveProjectViaFile(projectName, blkContent, jsFileContent, flavor, group, callback) {
  console.log('saveProjectViaFile');
  console.log('projectName is ' + projectName);
  console.log('blkContent is ' + blkContent);
  console.log('jsFileContent is ' + jsFileContent);
  console.log('flavor is ' + flavor);
  console.log('group is ' + group);
  callback(true, '');
  //callback(false, 'Save project failed.');
}

function renameProjectViaFile(oldProjectName, newProjectName, callback) {
  callback(true, '');
  //callback(false, 'Rename project failed.');
}

function copyProjectViaFile(oldProjectName, newProjectName, callback) {
  callback(true, '');
  //callback(false, 'Copy project failed.');
}

function deleteProjectsViaFile(starDelimitedProjectNames, callback) {
  callback(true, '');
  //callback(false, 'Delete projects failed.');
}
