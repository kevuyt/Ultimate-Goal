package org.firstinspires.ftc.teamcode.TeleOp;

/**
 * This is an area you can add all your TeleOp constants for example the closed position of a certain servo etc.
 */
public interface Constants {
    int LIFT_MAX_ROTATIONS = 10;
    int TICKS_PER_ROTATION = 1120;
    int LIFT_MIN = 0,
        LIFT_MAX = LIFT_MAX_ROTATIONS * TICKS_PER_ROTATION;
    String INIT_MESSAGE = ">>> Press Play to Start.";
    double STONE_PUSHER_DOWN = 0,
           STONE_PUSHER_UP = 0.7;
    double GLYPH_TOP_CLOSED = 1,
           GLYPH_TOP_OPENED = 0,
           GLYPH_BOTTOM_CLOSED = 0,
           GLYPH_BOTTOM_OPENED = 0.8;
    double LIFT_DOWN = -1,
           LIFT_UP = 1;
    double JEWEL_BLUE_IN = 0.3,
           JEWEL_BLUE_OUT = 1,
           JEWEL_RED_IN = 0.6,
           JEWEL_RED_OUT = 0;
    double CLAW_CLOSED = 0.3,
           CLAW_OPENED = 0.7;
}
