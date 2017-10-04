package Library4997.MasqWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;
import Library4997.MasqExternal.MasqHardware;

/**
 * Custom gamepad
 */

public class MasqController implements MasqHardware {
    private String name;
    private Gamepad gamepad;

    public MasqController(Gamepad g, String name){
        this.name = name;
        this.gamepad = g;
    }

    public boolean a() {return gamepad.a;}
    public boolean x() {return gamepad.x;}
    public boolean y() {return gamepad.y;}
    public boolean b() {return gamepad.b;}

    public boolean aOnPress() {
        boolean pressed = false, released = false;
        while (a()) {
            released = false;
            pressed = true;
        }
        while (!a()) {
            released = true;
        }
        return pressed && released;
    }
    public boolean bOnPress() {
        boolean pressed = false, realeaed = false;
        while (b()) {
            realeaed = false;
            pressed = true;
        }
        while (!b()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }
    public boolean yOnPress() {
        boolean pressed = false, realeaed = false;
        while (b()) {
            realeaed = false;
            pressed = true;
        }
        while (!b()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }
    public boolean xOnPress() {
        boolean pressed = false, realeaed = false;
        while (x()) {
            realeaed = false;
            pressed = true;
        }
        while (!x()) {
            realeaed = true;
        }
        return pressed && realeaed;
    }

    public float leftStickX(){
        return gamepad.left_stick_x;
    }
    public float leftStickY() {
        return gamepad.left_stick_y;
    }
    public float rightStickX() {
        return gamepad.right_stick_x;
    }
    public float rightStickY() {
        return gamepad.right_stick_y;
    }

    public boolean dpad_up() {
        return gamepad.dpad_up;
    }
    public boolean dpad_down() {
        return gamepad.dpad_down;
    }
    public boolean dpad_left() {
        return gamepad.dpad_left;
    }
    public boolean dpad_right() {
        return gamepad.dpad_right;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean leftBumperOnPress () {return onPress(rightBumper());}
    public boolean rightBumperOnPress () {return onPress(rightBumper());}

    public boolean leftStickButton() {
        return gamepad.left_stick_button;
    }
    public boolean rightStickButton() {
        return gamepad.right_stick_button;
    }

    public boolean leftTriggerPressed() {
        return leftTrigger() > 0;
    }
    public boolean rightTriggerPressed() {
        return rightTrigger() > 0;
    }

    public boolean leftTriggerOnPress(){return onPress(leftTriggerPressed());}
    public boolean rightTriggerOnPress(){return onPress(leftTriggerPressed());}

    public float leftTrigger() {return gamepad.left_trigger;}
    public float rightTrigger() {return gamepad.right_trigger;}

    private boolean onPress(boolean b){
        boolean pressed = false, released = false;
        while (b) {
            released = false;
            pressed = true;
        }
        while (!b) {
            released = true;
        }
        return pressed && released;
    }

    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                "A: " + Boolean.toString(a()),
                "aPress: " + Boolean.toString(aOnPress()),
                "B: " + Boolean.toString(b()),
                "bPress: " + Boolean.toString(bOnPress()),
                "X: " + Boolean.toString(x()),
                "xPress: " + Boolean.toString(xOnPress()),
                "Y: " + Boolean.toString(y()),
                "yPress: " + Boolean.toString(yOnPress()),
                "RB: " + Boolean.toString(rightBumper()),
                "rightBumperPressed: " + Boolean.toString(rightBumperOnPress()),
                "LB: " + Boolean.toString(leftBumper()),
                "leftBumperPressed: " + Boolean.toString(leftBumperOnPress()),
                "leftTrigger: " + Float.toString(leftTrigger()),
                "leftTriggerPress: " + Boolean.toString(leftTriggerPressed()),
                "rightTrigger: " + Float.toString(rightTrigger()),
                "rightTriggerPress: " + Boolean.toString(rightTriggerPressed()),
                "rightStickY: " + Double.toString(rightStickX()),
                "rightStickX: " + Double.toString(rightStickX()),
                "leftStickY: " + Double.toString(leftStickY()),
                "leftStickX: " + Double.toString(leftStickX()),
                "D_UP: " + Boolean.toString(dpad_up()),
                "D_DOWN: " + Boolean.toString(dpad_down()),
                "D_LEFT: " + Boolean.toString(dpad_left()),
                "D_RIGHT: " + Boolean.toString(dpad_right()),
        };
    }
}
