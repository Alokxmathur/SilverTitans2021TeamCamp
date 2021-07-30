package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.PhoebeColorSensor;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveUntilColorOperation extends Operation {
    private double speed;
    private Alliance.Color color;

    public DriveUntilColorOperation(Alliance.Color color, double speed, String title) {
        this.speed = speed;
        this.color = color;
        this.type = TYPE.DRIVE_UNTIL_COLOR;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveUntilColor: @%.2f --%s",
                this.speed,
                this.title);
    }

    public boolean isComplete(PhoebeColorSensor phoebeColorSensor, MecanumDriveTrain driveTrain) {
        Match.log("Color=b:" + phoebeColorSensor.getBlue() + ",r:" + phoebeColorSensor.getRed());
        switch (this.color) {
            case BLUE: {
                if (phoebeColorSensor.getBlue() > 100) {
                    driveTrain.stop();
                    return true;
                }
            }
            case RED: {
                if (phoebeColorSensor.getRed() > 100) {
                    driveTrain.stop();
                    return true;
                }
            }
        }
        return false;
    }

    public double getSpeed() {
        return this.speed;
    }
}

