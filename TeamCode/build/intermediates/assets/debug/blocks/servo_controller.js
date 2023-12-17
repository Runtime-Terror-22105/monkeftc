<<<<<<< HEAD
/**
 * @license
 * Copyright 2016 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview FTC robot blocks related to servo controller.
 * @author lizlooney@google.com (Liz Looney)
 */

// The following are generated dynamically in HardwareUtil.fetchJavaScriptForHardware():
// createServoControllerDropdown
// The following are defined in vars.js:
// createNonEditableField
// functionColor
// getPropertyColor

Blockly.Blocks['servoController_getProperty'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['PwmStatus', 'PwmStatus'],
    ];
    this.setOutput(true); // no type, for compatibility
    this.appendDummyInput()
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['PwmStatus', 'Returns the PwmStatus status of the set of servos connected to this ' +
            'controller: ENABLED, DISABLED, or MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_getProperty'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  var property = block.getFieldValue('PROP');
  var code = identifier + '.get' + property + '()';
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

Blockly.FtcJava['servoController_getProperty'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  var property = block.getFieldValue('PROP');
  var code = identifier + '.get' + property + '()';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
};

Blockly.Blocks['servoController_getProperty_PwmStatus'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['PwmStatus', 'PwmStatus'],
    ];
    this.setOutput(true, 'ServoController.PwmStatus');
    this.appendDummyInput()
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['PwmStatus', 'Returns the PwmStatus status of the set of servos connected to this ' +
            'controller: ENABLED, DISABLED, or MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_getProperty_PwmStatus'] =
    Blockly.JavaScript['servoController_getProperty'];

Blockly.FtcJava['servoController_getProperty_PwmStatus'] =
    Blockly.FtcJava['servoController_getProperty'];

// Enums

Blockly.Blocks['servoController_enum_pwmStatus'] = {
  init: function() {
    var PWM_STATUS_CHOICES = [
        ['ENABLED', 'ENABLED'],
        ['DISABLED', 'DISABLED'],
        ['MIXED', 'MIXED'],
    ];
    this.setOutput(true); // no type, for compatibility
    this.appendDummyInput()
        .appendField(createNonEditableField('PwmStatus'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PWM_STATUS_CHOICES), 'PWM_STATUS');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['ENABLED', 'The PwmStatus value ENABLED.'],
        ['DISABLED', 'The PwmStatus value DISABLED.'],
        ['MIXED', 'The PwmStatus value MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PWM_STATUS');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_enum_pwmStatus'] = function(block) {
  var code = '"' + block.getFieldValue('PWM_STATUS') + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.FtcJava['servoController_enum_pwmStatus'] = function(block) {
  var code = 'ServoController.PwmStatus.' + block.getFieldValue('PWM_STATUS');
  Blockly.FtcJava.generateImport_('ServoController');
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['servoController_typedEnum_pwmStatus'] = {
  init: function() {
    var PWM_STATUS_CHOICES = [
        ['ENABLED', 'ENABLED'],
        ['DISABLED', 'DISABLED'],
        ['MIXED', 'MIXED'],
    ];
    this.setOutput(true, 'ServoController.PwmStatus');
    this.appendDummyInput()
        .appendField(createNonEditableField('PwmStatus'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PWM_STATUS_CHOICES), 'PWM_STATUS');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['ENABLED', 'The PwmStatus value ENABLED.'],
        ['DISABLED', 'The PwmStatus value DISABLED.'],
        ['MIXED', 'The PwmStatus value MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PWM_STATUS');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_typedEnum_pwmStatus'] =
    Blockly.JavaScript['servoController_enum_pwmStatus'];

Blockly.FtcJava['servoController_typedEnum_pwmStatus'] =
    Blockly.FtcJava['servoController_enum_pwmStatus'];

// Functions

Blockly.Blocks['servoController_pwmEnable'] = {
  init: function() {
    this.appendDummyInput()
        .appendField('call')
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(createNonEditableField('pwmEnable'));
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(functionColor);
    this.setTooltip('Enables all of the servos connected to this controller.');
  }
};

Blockly.JavaScript['servoController_pwmEnable'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  return identifier + '.pwmEnable();\n';
};

Blockly.FtcJava['servoController_pwmEnable'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  return identifier + '.pwmEnable();\n';
};

Blockly.Blocks['servoController_pwmDisable'] = {
  init: function() {
    this.appendDummyInput()
        .appendField('call')
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(createNonEditableField('pwmDisable'));
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(functionColor);
    this.setTooltip('Disables all of the servos connected to this controller.');
  }
};

Blockly.JavaScript['servoController_pwmDisable'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  return identifier + '.pwmDisable();\n';
};

Blockly.FtcJava['servoController_pwmDisable'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  return identifier + '.pwmDisable();\n';
};
=======
/**
 * @license
 * Copyright 2016 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview FTC robot blocks related to servo controller.
 * @author lizlooney@google.com (Liz Looney)
 */

// The following are generated dynamically in HardwareUtil.fetchJavaScriptForHardware():
// createServoControllerDropdown
// The following are defined in vars.js:
// createNonEditableField
// functionColor
// getPropertyColor

Blockly.Blocks['servoController_getProperty'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['PwmStatus', 'PwmStatus'],
    ];
    this.setOutput(true); // no type, for compatibility
    this.appendDummyInput()
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['PwmStatus', 'Returns the PwmStatus status of the set of servos connected to this ' +
            'controller: ENABLED, DISABLED, or MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_getProperty'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  var property = block.getFieldValue('PROP');
  var code = identifier + '.get' + property + '()';
  return [code, Blockly.JavaScript.ORDER_FUNCTION_CALL];
};

Blockly.FtcJava['servoController_getProperty'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  var property = block.getFieldValue('PROP');
  var code = identifier + '.get' + property + '()';
  return [code, Blockly.FtcJava.ORDER_FUNCTION_CALL];
};

Blockly.Blocks['servoController_getProperty_PwmStatus'] = {
  init: function() {
    var PROPERTY_CHOICES = [
        ['PwmStatus', 'PwmStatus'],
    ];
    this.setOutput(true, 'ServoController.PwmStatus');
    this.appendDummyInput()
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PROPERTY_CHOICES), 'PROP');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['PwmStatus', 'Returns the PwmStatus status of the set of servos connected to this ' +
            'controller: ENABLED, DISABLED, or MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PROP');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_getProperty_PwmStatus'] =
    Blockly.JavaScript['servoController_getProperty'];

Blockly.FtcJava['servoController_getProperty_PwmStatus'] =
    Blockly.FtcJava['servoController_getProperty'];

// Enums

Blockly.Blocks['servoController_enum_pwmStatus'] = {
  init: function() {
    var PWM_STATUS_CHOICES = [
        ['ENABLED', 'ENABLED'],
        ['DISABLED', 'DISABLED'],
        ['MIXED', 'MIXED'],
    ];
    this.setOutput(true); // no type, for compatibility
    this.appendDummyInput()
        .appendField(createNonEditableField('PwmStatus'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PWM_STATUS_CHOICES), 'PWM_STATUS');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['ENABLED', 'The PwmStatus value ENABLED.'],
        ['DISABLED', 'The PwmStatus value DISABLED.'],
        ['MIXED', 'The PwmStatus value MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PWM_STATUS');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_enum_pwmStatus'] = function(block) {
  var code = '"' + block.getFieldValue('PWM_STATUS') + '"';
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.FtcJava['servoController_enum_pwmStatus'] = function(block) {
  var code = 'ServoController.PwmStatus.' + block.getFieldValue('PWM_STATUS');
  Blockly.FtcJava.generateImport_('ServoController');
  return [code, Blockly.FtcJava.ORDER_MEMBER];
};

Blockly.Blocks['servoController_typedEnum_pwmStatus'] = {
  init: function() {
    var PWM_STATUS_CHOICES = [
        ['ENABLED', 'ENABLED'],
        ['DISABLED', 'DISABLED'],
        ['MIXED', 'MIXED'],
    ];
    this.setOutput(true, 'ServoController.PwmStatus');
    this.appendDummyInput()
        .appendField(createNonEditableField('PwmStatus'))
        .appendField('.')
        .appendField(new Blockly.FieldDropdown(PWM_STATUS_CHOICES), 'PWM_STATUS');
    this.setColour(getPropertyColor);
    // Assign 'this' to a variable for use in the tooltip closure below.
    var thisBlock = this;
    var TOOLTIPS = [
        ['ENABLED', 'The PwmStatus value ENABLED.'],
        ['DISABLED', 'The PwmStatus value DISABLED.'],
        ['MIXED', 'The PwmStatus value MIXED.'],
    ];
    this.setTooltip(function() {
      var key = thisBlock.getFieldValue('PWM_STATUS');
      for (var i = 0; i < TOOLTIPS.length; i++) {
        if (TOOLTIPS[i][0] == key) {
          return TOOLTIPS[i][1];
        }
      }
      return '';
    });
  }
};

Blockly.JavaScript['servoController_typedEnum_pwmStatus'] =
    Blockly.JavaScript['servoController_enum_pwmStatus'];

Blockly.FtcJava['servoController_typedEnum_pwmStatus'] =
    Blockly.FtcJava['servoController_enum_pwmStatus'];

// Functions

Blockly.Blocks['servoController_pwmEnable'] = {
  init: function() {
    this.appendDummyInput()
        .appendField('call')
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(createNonEditableField('pwmEnable'));
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(functionColor);
    this.setTooltip('Enables all of the servos connected to this controller.');
  }
};

Blockly.JavaScript['servoController_pwmEnable'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  return identifier + '.pwmEnable();\n';
};

Blockly.FtcJava['servoController_pwmEnable'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  return identifier + '.pwmEnable();\n';
};

Blockly.Blocks['servoController_pwmDisable'] = {
  init: function() {
    this.appendDummyInput()
        .appendField('call')
        .appendField(createServoControllerDropdown(), 'IDENTIFIER')
        .appendField('.')
        .appendField(createNonEditableField('pwmDisable'));
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(functionColor);
    this.setTooltip('Disables all of the servos connected to this controller.');
  }
};

Blockly.JavaScript['servoController_pwmDisable'] = function(block) {
  var identifier = block.getFieldValue('IDENTIFIER');
  return identifier + '.pwmDisable();\n';
};

Blockly.FtcJava['servoController_pwmDisable'] = function(block) {
  var identifier = Blockly.FtcJava.importDeclareAssign_(block, 'IDENTIFIER', 'ServoController');
  return identifier + '.pwmDisable();\n';
};
>>>>>>> f2a5e20575313289d016c6692bd65fe9145d1880
