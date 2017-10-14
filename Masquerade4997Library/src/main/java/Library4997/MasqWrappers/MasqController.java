package Library4997.MasqWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Archish on 10/12/17.
 */

public class MasqController {
    private String name;
    private Gamepad gamepad;
    private boolean
            aPrev = false, bPrev = false, xPrev = false, yPrev = false,
            leftBumperPrev = false, rightBumperPrev = false;

    public MasqController(Gamepad g, String name){
        this.name = name;
        this.gamepad = g;
    }

    public boolean a() {return gamepad.a;}
    public boolean x() {return gamepad.x;}
    public boolean y() {return gamepad.y;}
    public boolean b() {return gamepad.b;}

    public boolean aOnPress() {return a() && !aPrev;}
    public boolean bOnPress() {return b() && !bPrev;}
    public boolean yOnPress() {return y() && !yPrev;}
    public boolean xOnPress() {return x() && !xPrev;}

    public float leftStickX(){return gamepad.left_stick_x;}
    public float leftStickY() {
        return gamepad.left_stick_y;
    }
    public float rightStickX() {
        return gamepad.right_stick_x;
    }
    public float rightStickY() {
        return gamepad.right_stick_y;
    }

    public boolean dPadUp() {return gamepad.dpad_up;}
    public boolean dPadDown() {
        return gamepad.dpad_down;
    }
    public boolean dPadLeft() {
        return gamepad.dpad_left;
    }
    public boolean dPadRight() {
        return gamepad.dpad_right;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean leftBumperOnPress () {return leftBumper() && !leftBumperPrev;}
    public boolean rightBumperOnPress () {return rightBumper() && !rightBumperPrev;}

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

    public float leftTrigger() {return gamepad.left_trigger;}
    public float rightTrigger() {return gamepad.right_trigger;}

    public void update(){
        aPrev = gamepad.a;
        bPrev = gamepad.b;
        xPrev = gamepad.x;
        yPrev = gamepad.y;
        leftBumperPrev = gamepad.left_bumper;
        rightBumperPrev = gamepad.right_bumper;
    }

    public String getName() {return name;}
    public String[] getDash() {
        return new String[]{
                name,
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
                "D_UP: " + Boolean.toString(dPadUp()),
                "D_DOWN: " + Boolean.toString(dPadDown()),
                "D_LEFT: " + Boolean.toString(dPadLeft()),
                "D_RIGHT: " + Boolean.toString(dPadRight()),
        };
    }
}
