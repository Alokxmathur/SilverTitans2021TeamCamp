package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 9/16/17.
 */

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;

public class Field {
    public static final float MM_PER_INCH        = 25.4f;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    public static final double FIELD_WIDTH  = 6 * TILE_WIDTH;

    public static final double STONE_LENGTH = 8 * Field.MM_PER_INCH;
    public static final double STONE_WIDTH = 4 * MM_PER_INCH;
    public static final double STONE_HEIGHT = 5 * MM_PER_INCH;

    public static final double QUARRY_FROM_WALL = 2*TILE_WIDTH + STONE_WIDTH/2;

    public enum SkyStonePosition {
        TOP, MIDDLE, BOTTOM
    }
    public Field() {
    }

}