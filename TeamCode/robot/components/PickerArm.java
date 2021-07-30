package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

import java.util.Date;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class PickerArm {
    private static final String SHOULDER_MOTOR = "shoulderMotor";
    private static final String SHOULDER_CORRECTION_MOTOR = "shoulderCorrectionMotor";
    private static final String WINCH_MOTOR = "winchMotor";
    private static final String GRIPPER_SERVO = "gripperServo";
    private static final String CAPSTONE_SERVO = "capstoneServo";
    //our shoulder motor
    DcMotor shoulderMotor;
    //our shoulder correction motor
    DcMotor shoulderCorrectionMotor;
    //and the winch
    DcMotor winchMotor;
    //and the gripper servo
    Servo gripperServo;
    //and the capstone servo
    Servo capStoneServo;

    public static final double SHOULDER_GEAR_REDUCTION = (125f/30f) * (40f/15f);
    public static final double HD_HEX_20_TO_1_ENCODER_COUNT_PER_REV    = 560 ;    // eg: Rev 20:1 Motor Encoder
    public static final int CORE_HEX_ENCODER_COUNT_PER_REV = 288;

    public static final double SHOULDER_ENCODER_COUNT_PER_RADIAN =
            SHOULDER_GEAR_REDUCTION * CORE_HEX_ENCODER_COUNT_PER_REV / (Math.PI*2);

    public static final double SPOOL_RADIUS = 13.425;
    public static final double EXTENSION_ENCODER_COUNT_PER_MM =
            HD_HEX_20_TO_1_ENCODER_COUNT_PER_REV/2/Math.PI/SPOOL_RADIUS;

    public static final double  GRIPPER_CLOSED_POSITION= 0;
    public static final double  GRIPPER_OPEN_POSITION= 0.4;

    public static final double  CAPSTONE_HELD_POSITION= .1;
    public static final double  CAPSTONE_RELEASED_POSITION= 0.6;

    public static final double WINCH_SPEED = 1;
    public static final double SHOULDER_SPEED = .6;

    public static final int SHOULDER_INCREMENT = 10;

    public static final int SHOULDER_RELEASE_POSITION = (int) (Math.toRadians(10) * SHOULDER_ENCODER_COUNT_PER_RADIAN);
    public static final int SHOULDER_LIFT_POSITION = (int) (Math.toRadians(20) * SHOULDER_ENCODER_COUNT_PER_RADIAN);
    public static final int SHOULDER_VERTICAL_POSITION = (int) (Math.toRadians(70) * SHOULDER_ENCODER_COUNT_PER_RADIAN);
    public static final int SHOULDER_LEVEL_POSITION = 0;
    public static final int SHOULDER_GRAB_POSITION = -120;
    public static final double GRAB_EXTENSION = 8f*Field.MM_PER_INCH;
    public static final double CARRY_EXTENSION = 9f*Field.MM_PER_INCH;
    public static final double HOVER_EXTENSION = 12f*Field.MM_PER_INCH;
    public static final double PLACE_EXTENSION = 15f*Field.MM_PER_INCH;

    public static final int WITHIN_REACH_INTERVAL_SHOULDER = 40;
    public static final int WITHIN_REACH_INTERVAL_WINCH = 60;


 //These numbers are for the first set of protrusions
    public static final int LEVEL_1_SHOULDER = -10;
    public static final int LEVEL_1_WINCH = 1407;

    public static final int LEVEL_2_SHOULDER = 80;
    public static final int LEVEL_2_WINCH = 1544;

    public static final int LEVEL_3_SHOULDER = 220;
    public static final int LEVEL_3_WINCH = 1760;

    public static final int LEVEL_4_SHOULDER = 270;
    public static final int LEVEL_4_WINCH = 1880;

    public static final int LEVEL_5_SHOULDER = 310;
    public static final int LEVEL_5_WINCH = 2272;

    public static final int LEVEL_6_SHOULDER = 400;
    public static final int LEVEL_6_WINCH = 2820;

    public static final int LEVEL_7_SHOULDER = 450;
    public static final int LEVEL_7_WINCH = 3300;

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
        shoulderMotor = hardwareMap.get(DcMotor.class, SHOULDER_MOTOR);
        shoulderCorrectionMotor = hardwareMap.get(DcMotor.class, SHOULDER_CORRECTION_MOTOR);
        winchMotor = hardwareMap.get(DcMotor.class, WINCH_MOTOR);

        gripperServo = hardwareMap.get(Servo.class, GRIPPER_SERVO);
        capStoneServo = hardwareMap.get(Servo.class, CAPSTONE_SERVO);

        // Set shoulder motor to run with encoders.
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setZeroPowerBehavior(BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set shoulder correction motor to run with encoders.
        this.shoulderCorrectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderCorrectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set winch motor to run with encoders.
        this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.winchMotor.setZeroPowerBehavior(BRAKE);
        this.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closeGripper();
        holdCapStone();
    }

    public void stop() {
        //Stop our motors
        shoulderMotor.setPower(0);
        winchMotor.setPower(0);
        //close gripper to make it easy for next run
        closeGripper();
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "S:%.2f(%d->%d, c:%d(%d),e:%d),W:%.2f(%d->%d),%s,x:%.2f,y:%.2f,a:%.2f,e:%.2f",
                this.shoulderMotor.getPower(), this.shoulderMotor.getCurrentPosition(), this.shoulderMotor.getTargetPosition(),
                this.shoulderCorrectionMotor.getCurrentPosition(), (int) (SHOULDER_GEAR_REDUCTION*this.shoulderCorrectionMotor.getCurrentPosition()), this.getShoulderError(),
                this.winchMotor.getPower(), this.winchMotor.getCurrentPosition(), this.winchMotor.getTargetPosition(),
                gripperIsOpen ? "Open" : "Closed",
                getCurrentX(), getCurrentY(),
                Math.toDegrees(getCurrentAngle()), getCurrentExtension());
    }

    public void openGripper() {
        this.gripperServo.setPosition(GRIPPER_OPEN_POSITION);
        this.gripperIsOpen = true;
    }
    public void closeGripper() {
        this.gripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        this.gripperIsOpen = false;
    }

    public void holdCapStone() {
        this.capStoneServo.setPosition(CAPSTONE_HELD_POSITION);
    }

    public void releaseCapstone() {
        this.capStoneServo.setPosition(CAPSTONE_RELEASED_POSITION);
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

    /** Returns the difference between what the shoulder motor encoders report and what the shoulder correction motor
     * encoders report while factoring in the gearing involved
     * @return
     */
    public int getShoulderError() {
        return 0;
        /*
        return (this.shoulderMotor.getCurrentPosition()
            + (int) (SHOULDER_GEAR_REDUCTION * -this.shoulderCorrectionMotor.getCurrentPosition()));

         */
    }

    public boolean isComplete(PickerOperation operation) {
        switch (operation.getPickerOperationType()) {
            case ARM_EXTENSION:
                return winchWithinReach();
            case SHOULDER_GRAB:
            case SHOULDER_RELEASE:
            case SHOULDER_VERTICAL:
            case SHOULDER_POSITION:
            case SHOULDER_LIFT:
            case SHOULDER_LEVEL: {
                return shoulderWithinReach();
            }
            case OPEN_GRIPPER: {
                return (new Date().getTime() - operation.getStartTime().getTime() > 500);
            }
            case CLOSE_GRIPPER: {
                return (new Date().getTime() - operation.getStartTime().getTime() > 300);
            }
            case INITIAL:
            case CARRY:
            case HOVER:
            case PLACE:
            case EXTENSION_AND_SHOULDER_POSITION:
            case LEVEL_1:
            case LEVEL_2:
            case LEVEL_3:
            case LEVEL_4:
            case LEVEL_5:
            case LEVEL_6:
            case LEVEL_7:
                case UP:
            case OUT: {
                return shoulderWithinReach() && winchWithinReach();
            }
            case GRAB: {
                // The winch should have already been set to go to the grab position.
                // If the winch has reached its desired position, we ask the shoulder to get to the grab position
                // if it wasn't already asked to get there.
                //We return true if both winch and shoulder have reached their position.
                if (winchWithinReach()) {
                    if (getShoulderTarget() != SHOULDER_GRAB_POSITION) {
                        setShoulderPosition(SHOULDER_GRAB_POSITION);
                    }
                    return shoulderWithinReach();
                }
                else {
                    return false;
                }
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
                this.setShoulderPosition(0);
                break;
            }
            case ARM_EXTENSION: {
                this.setWinchPosition((int) (operation.getWinchExtension() * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case SHOULDER_RELEASE: {
                this.setShoulderPosition(SHOULDER_RELEASE_POSITION + this.getShoulderError());
                break;
            }
            case SHOULDER_LEVEL: {
                this.setShoulderPosition(SHOULDER_LEVEL_POSITION + this.getShoulderError());
                break;
            }
            case SHOULDER_LIFT: {
                this.setShoulderPosition(SHOULDER_LIFT_POSITION + this.getShoulderError());
                break;
            }
            case SHOULDER_GRAB: {
                this.setShoulderPosition(SHOULDER_GRAB_POSITION + this.getShoulderError());
                break;
            }
            case SHOULDER_VERTICAL: {
                this.setShoulderPosition(SHOULDER_VERTICAL_POSITION + this.getShoulderError());
                break;
            }
            case SHOULDER_POSITION: {
                this.setShoulderPosition(operation.getShoulderPosition());
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
            case CARRY: {
                this.setShoulderPosition(SHOULDER_LEVEL_POSITION + this.getShoulderError());
                this.setWinchPosition((int) (CARRY_EXTENSION * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case HOVER: {
                this.setShoulderPosition(SHOULDER_RELEASE_POSITION + this.getShoulderError());
                this.setWinchPosition((int) (HOVER_EXTENSION * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case PLACE: {
                this.setShoulderPosition(SHOULDER_RELEASE_POSITION + this.getShoulderError());
                this.setWinchPosition((int) (PLACE_EXTENSION * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case EXTENSION_AND_SHOULDER_POSITION: {
                this.setShoulderPosition(operation.getShoulderPosition());
                this.setWinchPosition((int) (operation.getWinchExtension() * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case LEVEL_1: {
                this.setShoulderPosition(LEVEL_1_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_1_WINCH);
                break;
            }
            case LEVEL_2: {
                this.setShoulderPosition(LEVEL_2_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_2_WINCH);
                break;
            }
            case LEVEL_3: {
                this.setShoulderPosition(LEVEL_3_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_3_WINCH);
                break;
            }
            case LEVEL_4: {
                this.setShoulderPosition(LEVEL_4_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_4_WINCH);
                break;
            }
            case LEVEL_5: {
                this.setShoulderPosition(LEVEL_5_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_5_WINCH);
                break;
            }
            case LEVEL_6: {
                this.setShoulderPosition(LEVEL_6_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_6_WINCH);
                break;
            }
            case LEVEL_7: {
                this.setShoulderPosition(LEVEL_6_SHOULDER + this.getShoulderError());
                this.setWinchPosition(LEVEL_6_WINCH);
                break;
            }
            case GRAB: {
                this.setWinchPosition((int) (GRAB_EXTENSION * EXTENSION_ENCODER_COUNT_PER_MM));
                break;
            }
            case UP: {
                this.raiseVertically(Field.STONE_HEIGHT);
                break;
            }
            case OUT: {
                this.extendHorizontally(Field.STONE_WIDTH);
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
        return (this.shoulderMotor.getTargetPosition() - this.getShoulderError()) / SHOULDER_ENCODER_COUNT_PER_RADIAN;
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
}