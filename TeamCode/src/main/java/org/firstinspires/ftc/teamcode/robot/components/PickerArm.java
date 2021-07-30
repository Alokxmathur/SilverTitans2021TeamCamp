package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

import java.util.Date;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class PickerArm {
    public static final int TIME_FOR_GRIPPER_TO_OPERATE = 200;
    //our shoulder motor
    DcMotor shoulderMotor;
    //and the winch
    DcMotor winchMotor;
    //and the gripper servo
    Servo gripperServo;


//    public static final double SHOULDER_GEAR_REDUCTION = (125f/30f) * (40f/15f);
    public static final double SHOULDER_GEAR_REDUCTION = (125f/30f) * (120f/40f) * (125f/45f);
    public static final double HD_HEX_20_TO_1_ENCODER_COUNT_PER_REV    = 560 ;    // eg: Rev 20:1 Motor Encoder
    public static final int CORE_HEX_ENCODER_COUNT_PER_REV = 288;

    public static final double SHOULDER_ENCODER_COUNT_PER_RADIAN =
            SHOULDER_GEAR_REDUCTION * CORE_HEX_ENCODER_COUNT_PER_REV / (Math.PI*2);

    public static final double SPOOL_RADIUS = 13.425;//mm
    public static final double EXTENSION_ENCODER_COUNT_PER_MM =
            HD_HEX_20_TO_1_ENCODER_COUNT_PER_REV/2/Math.PI/SPOOL_RADIUS;

    public static final double  GRIPPER_CLOSED_POSITION= 0;
    public static final double  GRIPPER_OPEN_POSITION= 0.4;

    public static final double WINCH_SPEED = 1;
    public static final double SHOULDER_SPEED = 1;

    public static final int SHOULDER_INCREMENT = 10;
    public static final int WINCH_INCREMENT = 20;

    public static final int SHOULDER_INIT_POSITION = 255;
    public static final int SHOULDER_RELEASE_POSITION = 475;
    public static final int SHOULDER_LIFT_POSITION = 600;
    public static final int SHOULDER_VERTICAL_POSITION = (int) (Math.toRadians(50) * SHOULDER_ENCODER_COUNT_PER_RADIAN);
    public static final int SHOULDER_LEVEL_POSITION = 0;
    public static final int SHOULDER_GRAB_POSITION = 195;
    public static final int SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR = 290;
    public static final int SHOULDER_POSITION_TO_CLEAR_WOBBLE = 625;

    public static final int HOVER_EXTENSION = 1500;
    public static final int COMPACT_EXTENSION = 640;

    public static final int WITHIN_REACH_INTERVAL_SHOULDER = 100;
    public static final int WITHIN_REACH_INTERVAL_WINCH = 100;


    //These numbers are for the first set of protrusions
    public static final int LEVEL_1_SHOULDER = 612;
    public static final int LEVEL_1_WINCH = 1610;

    public static final int LEVEL_2_SHOULDER = 192;
    public static final int LEVEL_2_WINCH = 1290;

    public static final int LEVEL_3_SHOULDER = 1043;
    public static final int LEVEL_3_WINCH = 2470;

    public static final int LEVEL_4_SHOULDER = 502;
    public static final int LEVEL_4_WINCH = 2470;

    public static final int LEVEL_5_SHOULDER = -190;
    public static final int LEVEL_5_WINCH = 1470;

    public static final int LEVEL_6_SHOULDER = 972;
    public static final int LEVEL_6_WINCH = 2820;

    public static final int WOBBLE_PICKUP_LEVEL = 140;
    public static final int WOBBLE_PICKUP_EXTENSION = 2000;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    boolean gripperIsOpen = false;

    //we start off with the gripper 10.25 inches from the base of the arm
    private double GRIPPER_OFFSET_FROM_SHOULDER = 10.25f*Field.MM_PER_INCH;

    int currentShoulderPosition = 0;
    int currentWinchPosition = 0;

    public PickerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        // Define and Initialize Motors
        winchMotor = hardwareMap.get(DcMotor.class, RobotConfig.WINCH_MOTOR);
        shoulderMotor = hardwareMap.get(DcMotor.class, RobotConfig.SHOULDER_MOTOR);
        gripperServo = hardwareMap.get(Servo.class, RobotConfig.GRIPPER_SERVO);

        // Set shoulder motor to run with encoders.
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setZeroPowerBehavior(BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set winch motor to run with encoders.
        this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.winchMotor.setZeroPowerBehavior(BRAKE);
        this.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closeGripper();
    }

    public void stop() {
        //Stop our motors
        shoulderMotor.setPower(0);
        winchMotor.setPower(0);
        //close gripper to make it easy for next run
        //closeGripper();
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "S:%.2f(%d->%d),W:%.2f(%d->%d),%s,x:%.2f,y:%.2f,a:%.2f,e:%.2f",
                this.shoulderMotor.getPower(), this.shoulderMotor.getCurrentPosition(), this.shoulderMotor.getTargetPosition(),
                this.winchMotor.getPower(), this.winchMotor.getCurrentPosition(), this.winchMotor.getTargetPosition(),
                gripperIsOpen ? "Open" : "Closed",
                getCurrentX(), getCurrentY(),
                Math.toDegrees(getCurrentAngle()), getCurrentExtension());
    }

    public void openGripper() {
        //Match.log("Opening gripper");
        this.gripperServo.setPosition(GRIPPER_OPEN_POSITION);
        this.gripperIsOpen = true;
    }
    public void closeGripper() {
        //Match.log("Closing gripper");
        this.gripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        this.gripperIsOpen = false;
    }

    public int getShoulderTarget() {
        return this.shoulderMotor.getTargetPosition();
    }

    public void setShoulderPosition(int position) {
        //Match.log("Set shoulder position to " + position);
        this.shoulderMotor.setTargetPosition(position);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulderMotor.setPower(SHOULDER_SPEED);
        this.currentShoulderPosition = position;
    }

    public void setWinchPosition(int position) {
        //Match.log("Set winch position to " + position);
        this.winchMotor.setTargetPosition(Math.max(0, position));
        this.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.winchMotor.setPower(WINCH_SPEED);
        this.currentWinchPosition = position;
    }
    public void incrementShoulderPosition() {
        this.setShoulderPosition(currentShoulderPosition + SHOULDER_INCREMENT);
    }

    public void decrementShoulderPosition() {
        this.setShoulderPosition(currentShoulderPosition - SHOULDER_INCREMENT);
    }

    public void incrementWinchPosition() {
        this.setWinchPosition(currentWinchPosition + WINCH_INCREMENT);
    }

    public void decrementWinchPosition() {
        this.setWinchPosition(currentWinchPosition - WINCH_INCREMENT);
    }

    public void setWinchPower(float winchPower) {
        this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.winchMotor.setPower(winchPower * WINCH_SPEED);
    }

    private int getCurrentWinchPosition() {
        if (this.winchMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return this.winchMotor.getTargetPosition();
        }
        else {
            return this.winchMotor.getCurrentPosition();
        }
    }

    public boolean isComplete(PickerOperation operation) {
        switch (operation.getPickerOperationType()) {
            case ARM_EXTENSION: {
                return winchWithinReach();
            }
            case SHOULDER_RELEASE:
            case SHOULDER_POSITION:
            case SHOULDER_LIFT:
            case VERTICAL:
            case SHOULDER_LEVEL: {
                return shoulderWithinReach();
            }
            case OPEN_GRIPPER:
            case CLOSE_GRIPPER: {
                return (new Date().getTime() - operation.getStartTime().getTime() > TIME_FOR_GRIPPER_TO_OPERATE);
            }
            case INITIAL:
            case HOVER:
            case EXTENSION_AND_SHOULDER_POSITION:
            case LEVEL_1:
            case LEVEL_2:
            case LEVEL_3:
            case LEVEL_4:
            case LEVEL_5:
            case LEVEL_6:
            case WOBBLE_PICKUP:
            case OUT:
            case COMPACT:
            case UP: {
                return shoulderWithinReach() && winchWithinReach();
            }
        }
        return false;
    }

    public boolean shoulderWithinReach() {
        return Math.abs(shoulderMotor.getTargetPosition() - shoulderMotor.getCurrentPosition()) < WITHIN_REACH_INTERVAL_SHOULDER;
    }

    public boolean winchWithinReach() {
        return Math.abs(winchMotor.getTargetPosition() - winchMotor.getCurrentPosition()) < WITHIN_REACH_INTERVAL_WINCH;
    }
    public void handleOperation(PickerOperation operation) {
        switch (operation.getPickerOperationType()) {
            case INITIAL: {
                this.setWinchPosition(0);
                this.setShoulderPosition(SHOULDER_INIT_POSITION);
                break;
            }
            case ARM_EXTENSION: {
                this.setWinchPosition((int) (operation.getWinchExtension()));
                break;
            }
            case SHOULDER_RELEASE: {
                this.setShoulderPosition(SHOULDER_RELEASE_POSITION);
                break;
            }
            case SHOULDER_LEVEL: {
                this.setShoulderPosition(SHOULDER_LEVEL_POSITION);
                break;
            }
            case SHOULDER_LIFT: {
                this.setShoulderPosition(SHOULDER_LIFT_POSITION);
                break;
            }
            case COMPACT: {
                this.setShoulderPosition(SHOULDER_VERTICAL_POSITION);
                this.setWinchPosition(COMPACT_EXTENSION);
                this.closeGripper();
                break;
            }
            case SHOULDER_POSITION: {
                this.setShoulderPosition(operation.getShoulderPosition());
                break;
            }
            case VERTICAL: {
                this.setShoulderPosition(SHOULDER_VERTICAL_POSITION);
                break;
            }
            case OPEN_GRIPPER: {
                this.openGripper();
                break;
            }
            case CLOSE_GRIPPER: {
                this.closeGripper();
                break;
            }
            case HOVER: {
                this.setShoulderPosition(SHOULDER_RELEASE_POSITION);
                this.setWinchPosition((int) (HOVER_EXTENSION));
                break;
            }
            case EXTENSION_AND_SHOULDER_POSITION: {
                this.setShoulderPosition(operation.getShoulderPosition());
                this.setWinchPosition((int) (operation.getWinchExtension()));
                break;
            }
            case LEVEL_1: {
                this.setShoulderPosition(LEVEL_1_SHOULDER);
                this.setWinchPosition(LEVEL_1_WINCH);
                break;
            }
            case LEVEL_2: {
                this.openGripper();
                this.setShoulderPosition(LEVEL_2_SHOULDER);
                this.setWinchPosition(LEVEL_2_WINCH);
                break;
            }
            case LEVEL_3: {
                this.closeGripper();
                this.setShoulderPosition(LEVEL_3_SHOULDER);
                this.setWinchPosition(LEVEL_3_WINCH);
                break;
            }
            case LEVEL_4: {
                this.setShoulderPosition(LEVEL_4_SHOULDER);
                this.setWinchPosition(LEVEL_4_WINCH);
                break;
            }
            case LEVEL_5: {
                this.setShoulderPosition(LEVEL_5_SHOULDER);
                this.setWinchPosition(LEVEL_5_WINCH);
                break;
            }
            case LEVEL_6: {
                this.setShoulderPosition(LEVEL_6_SHOULDER);
                this.setWinchPosition(LEVEL_6_WINCH);
                break;
            }
            case WOBBLE_PICKUP: {
                this.openGripper();
                this.setShoulderPosition(WOBBLE_PICKUP_LEVEL);
                this.setWinchPosition(WOBBLE_PICKUP_EXTENSION);
                break;
            }
            case UP:
                break;
            case OUT: {
                this.setWinchPosition(HOVER_EXTENSION);
                this.setShoulderPosition(SHOULDER_LIFT_POSITION);
                break;
            }
        }
    }

    /**
     * Get how far the gripper is from the base of the arm
     * @return
     */
    public double getCurrentExtension() {
        return (this.getCurrentWinchPosition() / EXTENSION_ENCODER_COUNT_PER_MM) + GRIPPER_OFFSET_FROM_SHOULDER;
    }

    /**
     * Get the angle of the arm (in radians)
     * @return
     */
    public double getCurrentAngle() {
        return this.shoulderMotor.getTargetPosition() / SHOULDER_ENCODER_COUNT_PER_RADIAN;
    }

    /**
     * Get how far the gripper extends beyond the base on the x axis
     * @return
     */
    public double getCurrentX() {
        return getCurrentExtension() * Math.cos(getCurrentAngle());
    }

    /**
     * Get how far above the starting position, the top of the gripper is
     * @return
     */
    public double getCurrentY() {
        return getCurrentExtension() * Math.sin(getCurrentAngle());
    }

    /**
     * Extend the arm horizontally keeping it at the same vertical level
     * @param mms - how far to extend horizontally
     */
    public void extendHorizontally(double mms) {
        double currentX = getCurrentX();
        double newX = currentX + mms;
        double currentY = getCurrentY();
        extend(currentX, currentY, newX, currentY);
    }


    /**
     * Raise the arm vertically keeping it at the same horizontal position
     * @param mms - how far to raise vertically
     */
    public void raiseVertically(double mms) {
        double currentX = getCurrentX();
        double currentY = getCurrentY();
        double newY = currentY + mms;
        extend(currentX, currentY, currentX, newY);
    }

    private void extend(double currentX, double currentY, double newX, double newY) {
        double currentHypotenuse = Math.hypot(currentX, currentY);
        double currentAngle = Math.asin(currentY/currentHypotenuse);
        double newHypotenuse = Math.hypot(newX, newY);
        double newAngle = Math.asin(newY/newHypotenuse);

        this.setShoulderPosition((int) (newAngle * SHOULDER_ENCODER_COUNT_PER_RADIAN));
        this.setWinchPosition((int) ((newHypotenuse - GRIPPER_OFFSET_FROM_SHOULDER) * EXTENSION_ENCODER_COUNT_PER_MM));
        Match.log(String.format(Locale.getDefault(),"Extended from (%.2f,%.2f)->(%.2f,%.2f), angle: (%.2f->%.2f), extension: (%.2f->%.2f)",
                currentX/Field.MM_PER_INCH, currentY/ Field.MM_PER_INCH,
                newX/Field.MM_PER_INCH, newY/Field.MM_PER_INCH,
                Math.toDegrees(currentAngle), Math.toDegrees(newAngle),
                currentHypotenuse/Field.MM_PER_INCH, newHypotenuse/Field.MM_PER_INCH));
    }

    public void ensureMotorDirection() {
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}