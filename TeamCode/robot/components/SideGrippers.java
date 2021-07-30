package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.operations.SideGrippersOperation;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class SideGrippers {
    public static final String RIGHT_GRIPPER_SERVO = "rightGripperServo";
    public static final String LEFT_GRIPPER_SERVO = "leftGripperServo";
    //our servos
    Servo rightGripperServo;
    Servo leftGripperServo;

    public static final double RIGHT_GRIPPER_UP_POSITION = 1.0;
    public static final double RIGHT_GRIPPER_DOWN_POSITION = 0.5;
    public static final double LEFT_GRIPPER_UP_POSITION = 0.0;
    public static final double LEFT_GRIPPER_DOWN_POSITION = 0.5;

    boolean rightGripperIsDown = false;
    boolean leftGripperIsDown = false;

    public SideGrippers(HardwareMap hardwareMap, Telemetry telemetry) {
        // Define and Initialize Motors
        rightGripperServo = hardwareMap.get(Servo.class, RIGHT_GRIPPER_SERVO);
        leftGripperServo = hardwareMap.get(Servo.class, LEFT_GRIPPER_SERVO);

        raiseLeftGripper();
        raiseRightGripper();
    }


    public String getStatus() {
        return String.format(Locale.getDefault(),
                "Left:%s,Right:%s",
                leftGripperIsDown ? "Down" : "Up",
                rightGripperIsDown ? "Down" : "Up"
                );
    }

    public void raiseLeftGripper() {
        this.leftGripperServo.setPosition(LEFT_GRIPPER_UP_POSITION);
        this.leftGripperIsDown = false;
    }
    public void lowerLeftGripper() {
        this.leftGripperServo.setPosition(LEFT_GRIPPER_DOWN_POSITION);
        this.leftGripperIsDown = true;
    }
    public void raiseRightGripper() {
        this.rightGripperServo.setPosition(RIGHT_GRIPPER_UP_POSITION);
        this.rightGripperIsDown = false;
    }
    public void lowerRightGripper() {
        this.rightGripperServo.setPosition(RIGHT_GRIPPER_DOWN_POSITION);
        this.rightGripperIsDown = true;
    }
    public boolean isComplete(SideGrippersOperation operation) {
        switch (operation.getOperationType()) {
            case RAISE_LEFT:
            case LOWER_LEFT:
            case RAISE_RIGHT:
            case LOWER_RIGHT:                 {
                return (new Date().getTime() - operation.getStartTime().getTime() > 400);
            }
        }
        return false;
    }

    public void handleOperation(SideGrippersOperation operation) {
        switch (operation.getOperationType()) {
            case RAISE_LEFT: {
                this.raiseLeftGripper();
                break;
            }
            case LOWER_LEFT: {
                this.lowerLeftGripper();
                break;
            }
            case RAISE_RIGHT: {
                this.raiseRightGripper();
                break;
            }
            case LOWER_RIGHT: {
                this.lowerRightGripper();
                break;
            }
        }
    }
}
