package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;

/**
 * Created by Silver Titans on 10/29/17.
 *
 * This class defines operations that the robot can perform.
 *
 * An enumeration called TYPE lists the types of operations that can be used.
 */

public abstract class Operation {
    public enum TYPE {
        DRIVE_FOR_TIME, DRIVE_FOR_DISTANCE, DRIVE_IN_DIRECTION, DRIVE_TO_POSITION,
        STRAFE_LEFT_FOR_DISTANCE, STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING, ROTATION,
        WAIT_TIME, BEARING,
        CAMERA, PICKER_OPERATION, TURN, FOLLOW_TRAJECTORY, SHOOTER, INTAKE
    }
    protected boolean operationIsBeingProcessed = false;

    public boolean isAborted() {
        return isAborted;
    }

    public void setAborted(boolean aborted) {
        isAborted = aborted;
    }

    protected boolean isAborted = false;

    private Date startTime;
    TYPE type;
    String title;
    public String getTitle() {
        return title;
    }
    public TYPE getType() {
        return this.type;
    }
    public boolean getOperationIsBeingProcessed() {
        return this.operationIsBeingProcessed;
    }

    public void setOperationBeingProcessed() {
        this.operationIsBeingProcessed = true;
        this.startTime = new Date();
    }

    public Date getStartTime() {
        return this.startTime;
    }
}
