package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;

import java.util.Date;
import java.util.Locale;

public class Intake {

    //TODO - find right positions
    public double PLATFORM_WIGGLE_BOTTOM_POSITION = .24;
    public double PLATFORM_INTAKE_POSITION = .263;
    public double PLATFORM_SHOOTING_POSITION = 0.084;
    public static final long WIGGLE_MSECS = 150;
    public static final int WIGGLE_REPS = 5;

    public static final double SERVO_INCREMENT = 0.001;

    private Servo platformServo;
    private CRServo intakeServo;
    private DcMotor intakeMotor;

    int wiggleCount;
    private long lastWiggleTime;

    public Intake(HardwareMap hardwareMap) {
        // Define and Initialize Motors
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        platformServo = hardwareMap.get(Servo.class, RobotConfig.PLATFORM_SERVO);
        intakeServo = hardwareMap.get(CRServo.class, RobotConfig.INTAKE_SERVO);

        assumeShootingPosition();
    }

    public void wiggleBottomPlatform() {
        this.platformServo.setPosition(PLATFORM_WIGGLE_BOTTOM_POSITION);
    }
    public void assumeIntakePosition() {
        this.platformServo.setPosition(PLATFORM_INTAKE_POSITION);
    }
    public void assumeShootingPosition() {
        this.platformServo.setPosition(PLATFORM_SHOOTING_POSITION);
    }

    public void startIntake() {
        this.assumeIntakePosition();
        this.intakeServo.setPower(-1);
        this.intakeMotor.setPower(RobotConfig.INTAKE_MOTOR_SPEED);
    }
    public void stopIntake() {
        this.assumeShootingPosition();
        this.intakeServo.setPower(0);
        this.intakeMotor.setPower(0);
    }

    public void ensureMotorDirection() {
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void increaseIntakeSpeed() {
        this.intakeMotor.setPower(Math.min(this.intakeMotor.getPower() + RobotConfig.INTAKE_SPEED_INCREMENT, 1));
    }

    public void decreaseIntakeSpeed() {
        this.intakeMotor.setPower(Math.max(this.intakeMotor.getPower() - RobotConfig.INTAKE_SPEED_INCREMENT, -1));
    }

    public void raisePlatform() {
        this.platformServo.setPosition(this.platformServo.getPosition() + SERVO_INCREMENT);
    }
    public void lowerPlatform() {
        this.platformServo.setPosition(this.platformServo.getPosition() - SERVO_INCREMENT);
    }
    public String getStatus() {
        return String.format(Locale.getDefault(),
                "S:%.3fA:%.3f",
                this.intakeMotor.getPower(),
                this.platformServo.getPosition());
    }

    public void handleOperation(IntakeOperation operation) {
        if (operation.getIntakeOperationType() == IntakeOperation.IntakeOperationType.LIFT_PLATFORM) {
            this.assumeShootingPosition();
        }
        else if (operation.getIntakeOperationType() == IntakeOperation.IntakeOperationType.FLATTEN_PLATFORM) {
            this.assumeIntakePosition();
        }
        else if (operation.getIntakeOperationType() == IntakeOperation.IntakeOperationType.START_INTAKE) {
            startIntake();
        }
        else if (operation.getIntakeOperationType() == IntakeOperation.IntakeOperationType.STOP_INTAKE) {
            stopIntake();
        }
        else if (operation.getIntakeOperationType() == IntakeOperation.IntakeOperationType.WIGGLE) {
            this.assumeShootingPosition();
            lastWiggleTime = new Date().getTime();
            wiggleCount = 1;
        }
    }

    public boolean isComplete(IntakeOperation operation) {
        switch (operation.getIntakeOperationType()) {
            case STOP_INTAKE:
            case START_INTAKE:
            case LIFT_PLATFORM:
            case FLATTEN_PLATFORM:
                return true;
            case WIGGLE: {
                if (wiggleCount < WIGGLE_REPS*2) {
                    long nowTime = new Date().getTime();
                    if (nowTime - lastWiggleTime > WIGGLE_MSECS) {
                        if (wiggleCount % 2 == 0) {
                            this.assumeShootingPosition();
                        } else {
                            this.wiggleBottomPlatform();
                        }
                        wiggleCount++;
                        lastWiggleTime = nowTime;
                    }
                    return false;
                }
                else {
                    //this.liftPlatform();
                    return  true;
                }
            }
        }
        return false;
    }
}
