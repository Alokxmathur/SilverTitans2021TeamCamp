package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class IntakeOperation extends Operation {
    public enum IntakeOperationType {
        LIFT_PLATFORM, FLATTEN_PLATFORM, WIGGLE, STOP_INTAKE, START_INTAKE
    }

    public IntakeOperationType getIntakeOperationType() {
        return intakeOperationType;
    }

    private IntakeOperationType intakeOperationType;

    public IntakeOperation(IntakeOperationType type, String title) {
        this.intakeOperationType = type;
        this.type = TYPE.INTAKE;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"Intake operation: %s--%s",
                this.getIntakeOperationType(), this.title);
    }
}
