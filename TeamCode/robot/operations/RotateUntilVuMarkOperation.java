package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class RotateUntilVuMarkOperation extends Operation {
    private double speed;

    public RotateUntilVuMarkOperation(double speed, String title) {
        this.speed = speed;
        this.type = TYPE.ROTATE_UNTIL_VUMARK;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Rotate Until VuMark: @%.2f --%s",
                this.speed,
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain, Robot robot) {
        String image = robot.findTarget();
        double x = robot.getCurrentX(), y = robot.getCurrentY();
        if (robot.findTarget() != null) {
            driveTrain.stop();
            return true;
        }
        Match.log(image + "x=" + robot.getCurrentX() + ",y=" + robot.getCurrentY());
        return false;
    }

    public double getSpeed() {
        return this.speed;
    }
}

