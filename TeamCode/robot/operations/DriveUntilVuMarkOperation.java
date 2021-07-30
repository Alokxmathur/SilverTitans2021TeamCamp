package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveUntilVuMarkOperation extends Operation {
    private double speed, direction;

    public DriveUntilVuMarkOperation(double speed, double direction, String title) {
        this.speed = speed;
        this.direction = direction;
        this.type = TYPE.DRIVE_UNTIL_VUMARK;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Drive Until VuMark: @%.2f --%s",
                this.speed,
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain, Robot robot) {
        String image = robot.findTarget();
        double x = robot.getCurrentX(), y = robot.getCurrentY();
        if (image != null) {
            driveTrain.stop();
            return true;
        }
        // adjust relative SPEED based on desiredHeading error.
        double bearingError = AngleUnit.normalizeDegrees(direction - driveTrain.getIMU().getBearing());
        double steer = MecanumDriveTrain.getSteer(bearingError, MecanumDriveTrain.P_DRIVE_COEFF);

        // if driving in reverse, the motor correction also needs to be reversed
        if (speed < 0)
            steer *= -1.0;
        double leftSpeed = speed - steer;
        double rightSpeed = speed + steer;

        // Normalize speeds if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        driveTrain.setLeftFrontPower(leftSpeed);
        driveTrain.setLeftRearPower(leftSpeed);
        driveTrain.setRightFrontPower(rightSpeed);
        driveTrain.setRightRearPower(rightSpeed);

        //Match.log("Error: " + bearingError + ", steer:" + steer);
        return false;
    }

    public double getSpeed() {
        return this.speed;
    }
}

