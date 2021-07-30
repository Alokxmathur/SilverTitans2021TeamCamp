package org.firstinspires.ftc.teamcode.robot.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;

import java.util.Date;
import java.util.Locale;

public class Shooter {

    public static final double NEW_P = 50;
    public static final double NEW_I = 0;
    public static final double NEW_D = 10;
    public static final double NEW_F = 13.1;
    //TODO - find right positions for shooting
    public static final double SQUEEZED_TRIGGER_POSITION = 0.35;
    public static final double OPEN_TRIGGER_POSITION = .7;
    public static final double INTAKE_POSITION = .684;
    public static final double SHOOTING_POSITION = .813;

    public static final double SERVO_INCREMENT = 0.001;
    //TODO - calibrate speeds

    public static final double HIGH_GOAL_SPEED = .74;
    public static final double MID_GOAL_SPEED = .5;
    public static final double LEFT_POWER_TARGET_SPEED = .654;
    public static final double CENTER_POWER_TARGET_SPEED = .654;
    public static final double RIGHT_POWER_TARGET_SPEED = .654;

    public static long TIME_TO_SHOOT = 900;

    private DcMotorEx shooterMotor;
    private Servo triggerServo;
    private Servo angleServo;

    private double lastSpeed = LEFT_POWER_TARGET_SPEED;
    private VoltageSensor batteryVoltageSensor;
    private boolean inShootingPosition;


    public Shooter(HardwareMap hardwareMap) {
        // Define and Initialize Motors
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        triggerServo = hardwareMap.get(Servo.class, RobotConfig.SHOOTER_TRIGGER_SERVO);
        angleServo = hardwareMap.get(Servo.class, RobotConfig.SHOOTER_ANGLE_SERVO);
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.SHOOTER_MOTOR);
        setPIDF();
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        assumeShootingPosition();
        unshoot();
    }

    public void setPIDF() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F*12/batteryVoltageSensor.getVoltage());
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Match.log("Set shooter pidf to " + pidfCoefficients.toString());
    }

    public void setShooterSpeed(double speed) {
        double batteryBasedSpeed = speed;
        Match.log("set shooter speed to " + batteryBasedSpeed);
        this.shooterMotor.setPower(batteryBasedSpeed);
        if (speed != 0) {
            lastSpeed = speed;
        }
    }

    public void shootHighGoal() {
        //this.shooterMotor.setPower(HIGH_GOAL_SPEED);
        //lastSpeed = HIGH_GOAL_SPEED;
        shoot();
    }

    public void shootMidGoal() {
        shoot();
    }

    public void shootPowerTarget() {
        //this.shooterMotor.setPower(LEFT_POWER_TARGET_SPEED);
        //lastSpeed = LEFT_POWER_TARGET_SPEED;
        shoot();
    }
    public void assumeIntakePosition() {
        this.angleServo.setPosition(INTAKE_POSITION);
        this.unshoot();
        this.inShootingPosition = false;
    }
    public void assumeShootingPosition() {
        this.setPIDF();
        this.angleServo.setPosition(SHOOTING_POSITION);
        inShootingPosition = true;
    }
    public void raiseShooter() {
        this.angleServo.setPosition(this.angleServo.getPosition() + SERVO_INCREMENT);
    }
    public void lowerShooter() {
        this.angleServo.setPosition(this.angleServo.getPosition() - SERVO_INCREMENT);
    }

    public void shoot() {
        this.resumeShooterSpeed();
        if (inShootingPosition) {
            this.triggerServo.setPosition(SQUEEZED_TRIGGER_POSITION);
        }
        else {
            Match.log("Didn't shoot because we were not in shooting position");
            this.assumeShootingPosition();
        }
    }

    public void unshoot() {
        this.triggerServo.setPosition(OPEN_TRIGGER_POSITION);
    }

    public void incrementTriggerServo() {
        this.triggerServo.setPosition(this.triggerServo.getPosition() + SERVO_INCREMENT);
    }
    public void decrementTriggerServo() {
        this.triggerServo.setPosition(this.triggerServo.getPosition() - SERVO_INCREMENT);
    }
    public void resumeShooterSpeed() {
        Match.log("Resuming speed of " + lastSpeed);
        this.setShooterSpeed(lastSpeed);
    }
    public void ensureMotorDirection() {
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void increaseSpeed() {
        this.shooterMotor.setPower(Math.min(this.shooterMotor.getPower() + RobotConfig.SHOOTER_SPEED_INCREMENT, 1));
        this.lastSpeed = this.shooterMotor.getPower();
    }

    public void decreaseSpeed() {
        this.shooterMotor.setPower(Math.max(this.shooterMotor.getPower() - RobotConfig.SHOOTER_SPEED_INCREMENT, -1));
        this.lastSpeed = this.shooterMotor.getPower();
    }

    public void handleOperation(ShootingOperation operation, Pose2d location) {
        if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_ON_HIGH_GOAL) {
            this.setShooterSpeed(HIGH_GOAL_SPEED);
            this.lastSpeed = HIGH_GOAL_SPEED;
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_ON_MID_GOAL) {
            this.setShooterSpeed(MID_GOAL_SPEED);
            this.lastSpeed = MID_GOAL_SPEED;
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_ON_LEFT_POWER_TARGET) {
            this.setShooterSpeed(LEFT_POWER_TARGET_SPEED);
            this.lastSpeed = LEFT_POWER_TARGET_SPEED;
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_ON_CENTER_POWER_TARGET) {
            this.setShooterSpeed(CENTER_POWER_TARGET_SPEED);
            this.lastSpeed = CENTER_POWER_TARGET_SPEED;
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_ON_RIGHT_POWER_TARGET) {
            this.setShooterSpeed(RIGHT_POWER_TARGET_SPEED);
            this.lastSpeed = RIGHT_POWER_TARGET_SPEED;
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.TURN_OFF) {
            this.shooterMotor.setPower(0);
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.SHOOT) {
            this.shoot();
            Match.log("Shot at " + location.toString());
        } else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.SHOOTING_POSITION) {
            this.assumeShootingPosition();
        }
        else if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.INTAKE_POSITION) {
            this.assumeIntakePosition();
        }
    }

    public boolean isComplete(ShootingOperation operation) {
        long timeSpent = new Date().getTime() - operation.getStartTime().getTime();
        if (operation.getShootingOperationType() == ShootingOperation.ShootingOperationType.SHOOT) {
            if (timeSpent > TIME_TO_SHOOT) {
                return true;
            }
            else if (timeSpent > TIME_TO_SHOOT*.6) {
                unshoot();
                return false;
            }
            else {
                return false;
            }
        }
        else {
            return true;
        }
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "S:%.3f(%.3f),A:%.3f, Tr:%.3f, Se: %d->%d",
                this.lastSpeed,
                this.shooterMotor.getPower(),
                this.angleServo.getPosition(),
                this.triggerServo.getPosition(),
                this.shooterMotor.getCurrentPosition(),
                this.shooterMotor.getTargetPosition());
    }
}
