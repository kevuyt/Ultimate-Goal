package org.firstinspires.ftc.teamcode.Autonomus;

/**
 * Constants in autonomous
 */

public interface Constants extends org.firstinspires.ftc.teamcode.TeleOp.Constants {
    double POWER_LOW = 0.3,
           POWER_OPTIMAL = 0.5,
           POWER_HIGH = 0.7;
    double SLEEPTIME_HIGH = 1000,
           SLEEPTIME_OPTIMAL = 500,
           SLEEPTIME_LOW = 250;
    String LEFT = "LEFT",
           RIGHT = "RIGHT",
           CENTER = "CENTER";
}
