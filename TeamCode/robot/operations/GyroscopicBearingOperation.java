package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.PIDController;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class GyroscopicBearingOperation extends Operation {
    public static final double MIN_SPEED = 0.1;
    protected double desiredBearing;

    public GyroscopicBearingOperation(double desiredBearing, String title) {
        this.type = TYPE.BEARING;
        this.title = title;
        this.desiredBearing = desiredBearing;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"GyroscopicBearing: %.2f --%s",
                this.desiredBearing, this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        // determine turn power based on +/- error
        double currentBearing = driveTrain.getIMU().getRawBearing();
        double error = AngleUnit.normalizeDegrees(desiredBearing - currentBearing);

        if (Math.abs(error) <= MecanumDriveTrain.HEADING_THRESHOLD) {
            driveTrain.stop();
            Match.log("Completed with error: " + AngleUnit.normalizeDegrees(desiredBearing - driveTrain.getIMU().getRawBearing()));
            return true;
        }
        else {
            double rotation = Math.max(Math.abs(error/180* MecanumDriveTrain.P_TURN_COEFF), MIN_SPEED);
            rotation *= -Math.signum(error);
            driveTrain.drive(0, 0, rotation, false);
            Match.log(String.format(Locale.getDefault(),
                    "Gyroscopic bearing rotation: %.2f, error: %.2f",
                    rotation, error));
            return false;
        }
    }

    /*
    public boolean isComplete(MecanumDriveTrain driveTrain) {
        // determine turn power based on +/- error
        double currentBearing = driveTrain.getIMU().getRawBearing();
        double error = AngleUnit.normalizeDegrees(degrees - currentBearing);

        if (Math.abs(error) <= MecanumDriveTrain.HEADING_THRESHOLD) {
            driveTrain.stop();
            Match.log("Done with error: " + error);
            return true;
        }
        else {
            pidController.setInput(error/180);
            double rotation = pidController.performPID();
            driveTrain.drive(0, 0, rotation, false);
            Match.log(String.format(Locale.getDefault(),
                    "Gyroscopic bearing rotation: %.2f, error: %.2f",
                    rotation, error));
            return false;
        }
    }

     */
}
