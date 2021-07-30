package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class TurnOperation extends Operation {

    public enum Direction {
        LEFT, RIGHT
    }

    public double getDegrees() {
        return degrees;
    }

    public void setDegrees(double degrees) {
        this.degrees = degrees;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    protected double degrees;
    private double speed;
    private Direction direction;

    public TurnOperation(double degrees, double speed, Direction direction, String title) {
        this.type = TYPE.TURN;
        this.title = title;
        this.degrees = degrees;
        this.speed = speed;
        this.direction = direction;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Turn degrees: %.2f@%.2f,%s --%s",
                this.degrees, this.speed, this.direction, this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        if (direction == Direction.RIGHT) {
            return driveTrain.rightWithinRange();
        }
        else {
            return driveTrain.leftWithinRange();
        }
    }
}
