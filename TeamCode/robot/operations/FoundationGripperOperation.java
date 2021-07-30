package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.FoundationGripper;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class FoundationGripperOperation extends Operation {
    public enum OperationType {
        RAISE, LOWER
    }

    public OperationType getOperationType() {
        return operationType;
    }

    OperationType operationType;

    public FoundationGripperOperation(OperationType operationType, String title) {
        this.type = TYPE.FOUNDATION_GRIPPER;
        this.operationType = operationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "FoundationGripper: %s --%s",
                this.operationType,
                this.title);
    }


    public boolean isComplete(FoundationGripper trap) {
        return trap.isComplete(this);
    }
}

