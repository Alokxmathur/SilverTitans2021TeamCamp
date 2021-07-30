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
        DRIVE_FOR_TIME, DRIVE_FOR_DISTANCE, GYROSCOPIC_DRIVE, DRIVE_UNTIL_VUMARK, DRIVE_UNTIL_COLOR, ROTATE_UNTIL_VUMARK,
        STRAFE_LEFT_FOR_DISTANCE, STRAFE_LEFT_FOR_TIME, STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING, STRAFE_UNTIL_XY, ROTATION, WAIT_TIME, WAIT_UNTIL_VUMARK, BEARING,
        CAMERA, PICKER_OPERATION, FOUNDATION_GRIPPER, SIDE_GRIPPER, TURN
    }
    protected boolean operationIsBeingProcessed = false;

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
