package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class FoundationGripper {
    public static final String GRIPPER_SERVO = "foundationServo";
    //our foundation servo
    private Servo gripperServo = null;

    public static final double GRIPPER_DOWN_POSITION = 0.15;
    public static final double GRIPPER_UP_POSITION = 0.5;

    boolean gripperIsDown = false;

    public FoundationGripper(HardwareMap hardwareMap, Telemetry telemetry) {
        // Define and Initialize Motors
        gripperServo = hardwareMap.get(Servo.class, GRIPPER_SERVO);

        raiseGripper();
    }


    public String getStatus() {
        return String.format(Locale.getDefault(),
                "%s",
                gripperIsDown ? "Down" : "Up");
    }

    public boolean isLowered() {
        return this.gripperIsDown;
    }

    public void raiseGripper() {
        this.gripperServo.setPosition(GRIPPER_UP_POSITION);
        this.gripperIsDown = false;
    }
    public void lowerGripper() {
        this.gripperServo.setPosition(GRIPPER_DOWN_POSITION);
        this.gripperIsDown = true;
    }

    public boolean isComplete(FoundationGripperOperation operation) {
        switch (operation.getOperationType()) {
            case RAISE:
            case LOWER: {
                return (new Date().getTime() - operation.getStartTime().getTime() > 400);
            }
        }
        return false;
    }

    public void handleOperation(FoundationGripperOperation operation) {
        switch (operation.getOperationType()) {
            case RAISE: {
                this.raiseGripper();
                break;
            }
            case LOWER: {
                this.lowerGripper();
                break;
            }
        }
    }
}
