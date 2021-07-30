package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StrafeLeftForDistanceWithHeadingOperation extends Operation {
    private double distance;
    private double speed;
    private double heading;

    public StrafeLeftForDistanceWithHeadingOperation(double distance, double heading, double speed, String title) {
        this.distance = distance;
        this.heading = heading;
        this.speed = speed;
        this.type = TYPE.STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StrafeLeft: %.2f\"@%.2f --%s",
                this.distance/ Field.MM_PER_INCH,
                this.speed,
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        }
        else {
            // adjust relative SPEED based on desiredHeading error.
            double bearingError = AngleUnit.normalizeDegrees(heading - driveTrain.getIMU().getBearing());
            double steer = MecanumDriveTrain.getSteer(bearingError, MecanumDriveTrain.P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;
            double speedToUse = speed;//new Date().getTime() - this.getStartTime().getTime() < 400 ? 0.1 : speed;
            double leftSpeed = speedToUse - steer;
            double rightSpeed = speedToUse + steer;

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
            Match.log(String.format(Locale.getDefault(), "Left speed: %.2f, right: %.2f", leftSpeed, rightSpeed));

            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getDistance() {
        return this.distance;
    }
}

