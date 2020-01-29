package Library4997.MasqWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import Library4997.MasqResources.MasqUtils;

/**
 * Created by Archish on 10/12/17.
 */

public class MasqController implements Runnable{
    private String name;
    private Gamepad gamepad;
    private boolean close = false;
    private boolean
            aPrev = false, bPrev = false, xPrev = false, yPrev = false,
            leftBumperPrev = false, rightBumperPrev = false, rightTriggerPrev = false,
            leftTriggerPrev = false, dPadUpPrev = false, dPadDownPrev = false, dPadRightPrev = false, dPadLeftPrev = false;

    public MasqController(Gamepad g, String name){
        this.name = name;
        this.gamepad = g;
    }

    public enum Button {
      A, B, X, Y,
      D_UP, D_DOWN, D_LEFT, D_RIGHT
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

    public boolean dPadUpOnPress() {return dPadUp() && !dPadUpPrev;}
    public boolean dPadDownOnPress() {return dPadUp() && !dPadDownPrev;}
    public boolean dPadUpRightPress() {return dPadUp() && !dPadRightPrev;}
    public boolean dPadUpLeftPress() {return dPadUp() && !dPadLeftPrev;}

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean leftBumperOnPress () {
        return leftBumper() && !leftBumperPrev;
    }
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
    public boolean leftTriggerOnPress() {
        return leftTriggerPressed() && !leftTriggerPrev;
    }
    public boolean rightTriggerOnPress() {return rightTriggerPressed() && !rightTriggerPrev;}

    public double getLeftStickAngle () {
        return Math.toDegrees(Math.atan2(leftStickY(), -leftStickX()));
    }
    public double getRightStickAngle () {
        return Math.toDegrees(Math.atan2(rightStickY(), -rightStickX()));
    }

    public double getTan (double x, double y) {
        double tan = -y/x;
        if (tan == Double.NEGATIVE_INFINITY || tan == Double.POSITIVE_INFINITY || tan != tan) tan = 0;
        return tan;
    }

    public boolean isJoysticksActive() {
        return Math.hypot(leftStickY(), leftStickX()) + Math.hypot(rightStickX(), rightStickY()) < 0.1;
    }

    public float leftTrigger() {return gamepad.left_trigger;}
    public float rightTrigger() {return gamepad.right_trigger;}

    public boolean start() {return gamepad.start;}

    public synchronized void update(){
        aPrev = gamepad.a;
        bPrev = gamepad.b;
        xPrev = gamepad.x;
        yPrev = gamepad.y;
        leftBumperPrev = gamepad.left_bumper;
        rightBumperPrev = gamepad.right_bumper;
        rightTriggerPrev = rightTriggerPressed();
        leftTriggerPrev = leftTriggerPressed();
        dPadDownPrev = dPadDown();
        dPadLeftPrev = dPadLeft();
        dPadRightPrev = dPadRight();
        dPadUpPrev = dPadUp();
    }

    public String getName() {return name;}
    public String[] getDash() {
        return new String[]{
                name,
                "A: " + a(),
                "aPress: " + aOnPress(),
                "B: " + b(),
                "bPress: " + bOnPress(),
                "X: " + x(),
                "xPress: " + xOnPress(),
                "Y: " + y(),
                "yPress: " + yOnPress(),
                "RB: " + rightBumper(),
                "rightBumperPressed: " + rightBumperOnPress(),
                "LB: " + leftBumper(),
                "leftBumperPressed: " + leftBumperOnPress(),
                "leftTrigger: " + leftTrigger(),
                "leftTriggerPress: " + leftTriggerPressed(),
                "rightTrigger: " + rightTrigger(),
                "rightTriggerPress: " + rightTriggerPressed(),
                "rightStickY: " + rightStickX(),
                "rightStickX: " + rightStickX(),
                "leftStickY: " + leftStickY(),
                "leftStickX: " + leftStickX(),
                "D_UP: " + dPadUp(),
                "D_DOWN: " + dPadDown(),
                "D_LEFT: " + dPadLeft(),
                "D_RIGHT: " + dPadRight(),
        };
    }

    public boolean getButton(Button b) {
        switch (b) {
            case X:
                return x();
            case A:
                return a();
            case B:
                return b();
            case Y:
                return y();
            case D_UP:
                return dPadUp();
            case D_DOWN:
                return dPadDown();
            case D_LEFT:
                return dPadLeft();
            case D_RIGHT:
                return dPadRight();
        }
        return false;
    }

    @Override
    public void run() {
        boolean close = false;
        while (!close) {
            System.out.println(aPrev);
            System.out.println(a());
            update();
            close = this.close;
            MasqUtils.sleep(100);
        }
    }
    public void close() {close = true;}
    public void startUpdate (){new Thread(this).start();}
}
