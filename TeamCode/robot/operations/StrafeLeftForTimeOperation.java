package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StrafeLeftForTimeOperation extends Operation {
    private long time;
    private double speed;
    private boolean goRight;

    public StrafeLeftForTimeOperation(long time, double speed, boolean goRight, String title) {
        this.time = time;
        this.speed = speed;
        this.goRight = goRight;
        this.type = TYPE.STRAFE_LEFT_FOR_TIME;
        this.title = title;
    }

    public String toString() {
        return String.format("Strafe " + (goRight ? "right" : "left") + " for " + time + " msecs, speed: " + speed);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        if (new Date().getTime() - this.getStartTime().getTime() > this.time) {
            driveTrain.stop();
            return true;
        }
        return false;
    }

    public double getSpeed() {
        return goRight ? -this.speed: speed;
    }

    public double getTime() {
        return this.time;
    }
}

