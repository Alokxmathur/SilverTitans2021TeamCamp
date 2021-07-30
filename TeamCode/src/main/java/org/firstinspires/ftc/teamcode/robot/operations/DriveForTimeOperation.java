package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveForTimeOperation extends Operation {
    private long time;
    private double desiredHeading;
    private double speed;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public long getTime() {
        return time;
    }

    public DriveForTimeOperation(long time, double heading, double speed, String title) {
        this.time = time;
        this.type = TYPE.DRIVE_FOR_TIME;
        this.desiredHeading = heading;
        this.speed = speed;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StraightLine: %d@%.2f --%s",
                this.time, this.desiredHeading,
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        if (new Date().getTime() > (this.getStartTime().getTime() + getTime())) {
            driveTrain.stop();
            return true;
        } else {
            driveTrain.drive(this.desiredHeading, this.getSpeed(), 0);
            return false;
        }
    }

    public double getSpeed() {
        return this.speed;
    }
}

