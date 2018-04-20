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
           GLYPH_BOTTOM_CLOSED = .55,
           GLYPH_BOTTOM_INTAKE = .6,
           GLYPH_BOTTOM_OPENED = 1;
    double INTAKE = 1,
           OUTAKE = -1;
    double FLIPPER_OUT_RIGHT = .05,
           FLIPPER_MID_RIGHT = .5,
           FLIPPER_RIGHT_RIGHT = .27,
           FLIPPER_DOWN_RIGHT = 0.62,
           FLIPPER_OUT_LEFT = .13,
           FLIPPER_MID_LEFT = .4,
           FLIPPER_RIGHT_LEFT= .27,
            FLIPPER_DOWN_LEFT = 0.46;
    double LIFT_DOWN = -1,
           LIFT_UP = 1;
    double JEWEL_BLUE_IN = 0.6,
           JEWEL_BLUE_OUT = 0,
           JEWEL_BLUE_HOVER = .75,
           JEWEL_RED_IN = 0.25,
           JEWEL_RED_OUT = 1,
           JEWEL_RED_HOVER = JEWEL_RED_OUT - .0125;
    double CLAW_CLOSED = 0,
           CLAW_OPENED = 1;
}
