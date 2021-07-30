package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.SideGrippers;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class SideGrippersOperation extends Operation {
    public enum OperationType {
        RAISE_LEFT, LOWER_LEFT, RAISE_RIGHT, LOWER_RIGHT
    }

    public OperationType getOperationType() {
        return operationType;
    }

    OperationType operationType;

    public SideGrippersOperation(OperationType operationType, String title) {
        this.type = TYPE.SIDE_GRIPPER;
        this.operationType = operationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "FoundationGripper: %s --%s",
                this.operationType,
                this.title);
    }


    public boolean isComplete(SideGrippers trap) {
        return trap.isComplete(this);
    }
}

