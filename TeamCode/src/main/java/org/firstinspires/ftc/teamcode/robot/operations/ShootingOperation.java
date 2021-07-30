package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class ShootingOperation extends Operation {
    public enum ShootingOperationType {
        TURN_ON_HIGH_GOAL, TURN_ON_MID_GOAL, TURN_ON_LEFT_POWER_TARGET, TURN_ON_CENTER_POWER_TARGET, TURN_ON_RIGHT_POWER_TARGET, TURN_OFF, SHOOT, SHOOTING_POSITION, INTAKE_POSITION
    }

    public ShootingOperationType getShootingOperationType() {
        return shootingOperationType;
    }

    private ShootingOperationType shootingOperationType;

    public ShootingOperation(ShootingOperationType type, String title) {
        this.shootingOperationType = type;
        this.type = TYPE.SHOOTER;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Shooting operation: %s--%s",
                this.getShootingOperationType(), this.title);
    }
}
