package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StrafeUntilXYOperation extends Operation {
    private double speed;
    private double minX, maxX, minY, maxY;

    public StrafeUntilXYOperation(double speed, String title) {
        this.speed = speed;
        this.minX = Double.MIN_VALUE;
        this.maxX = Double.MAX_VALUE;
        this.minY = Double.MIN_VALUE;
        this.maxY = Double.MAX_VALUE;
        this.type = TYPE.STRAFE_UNTIL_XY;
        this.title = title;
    }

    public StrafeUntilXYOperation(double speed, double minX, double maxX, double minY, double maxY, String title) {
        this.speed = speed;
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
        this.type = TYPE.STRAFE_UNTIL_XY;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Strafe: @%.2f until %.2f<x>%.2f, %.2f<y>%.2f--%s",
                this.speed,
                minX, maxX, minY, maxY,
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain, Robot robot) {
        String image = robot.findTarget();
        double x = robot.getCurrentX(), y = robot.getCurrentY();
        if (image != null && x >= minX && x <=maxX && y >= minY && y <= maxY) {
            driveTrain.stop();
            return true;
        }
        Match.log(image + "x=" + robot.getCurrentX() + ",y=" + robot.getCurrentY());
        return false;
    }

    public double getSpeed() {
        return speed;
    }
}

