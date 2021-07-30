package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.Intake;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.Shooter;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ClockwiseRotationOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToPositionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

/**
 * This class represents our robot.
 * The config on the robot needs to have the following entries defined:
 * *
 * rightDrive: the right motor of the drive train
 * leftDrive: the left motor of the drive tpenrain
 */

public class Robot {

    public static final double LENGTH = 22*Field.MM_PER_INCH; // 13 1/8 inches
    public static final double WIDTH = 17.5*Field.MM_PER_INCH;

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;
    OperationThread operationThreadTertiary;

    MecanumDriveTrain mecanumDriveTrain;
    PickerArm pickerArm;
    Intake intake;
    Shooter shooter;
    //WebCam webcam;
    OpenCVWebcam webcam;
    VslamCamera vslamCamera;

    boolean everythingButCamerasInitialized = false;
    boolean shootingEndGameRings;
    boolean firstRingShot, firstRingQueued, secondRingShot, secondRingQueued, thirdRingShot, thirdRingQueued;

    //Our sensors etc.

    //our state
    String state = "pre-initialized";
    private boolean shootingPowerTarget;
    private boolean shootingHighGoal;
    private boolean intakeIsOn;

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    /**
     * Initialize our robot
     * We set our alliance and our starting position based on finding a VuMark
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        intakeIsOn = shootingHighGoal = shootingPowerTarget = false;

        //initialize our components
        initCameras(match.getAllianceColor(), match.getStartingPosition());
        initDriveTrain();
        initPickerArm(telemetry);
        initLiftMechanism(telemetry);
        initShooter(telemetry);

        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();

        Match.log("Started operations threads");
        this.operationThreadPrimary = new OperationThread(this, "Primary");
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary");
        operationThreadSecondary.start();
        this.operationThreadTertiary = new OperationThread(this, "Tertiary");
        operationThreadTertiary.start();

        this.everythingButCamerasInitialized = true;
    }

    public void initDriveTrain() {
        //Create our drive train
        telemetry.addData("Status", "Initializing drive train, please wait");
        telemetry.update();
        this.mecanumDriveTrain = new MecanumDriveTrain(hardwareMap, telemetry, vslamCamera);
    }

    public void initPickerArm(Telemetry telemetry) {
        telemetry.addData("Status", "Initializing picker arm, please wait");
        telemetry.update();
        this.pickerArm = new PickerArm(hardwareMap, telemetry);
    }

    public void initLiftMechanism(Telemetry telemetry) {
        telemetry.addData("Status", "Initializing lift mechanism, please wait");
        telemetry.update();
        this.intake = new Intake(hardwareMap);
    }

    public void initShooter(Telemetry telemetry) {
        telemetry.addData("Status", "Initializing shooter, please wait");
        telemetry.update();
        this.shooter = new Shooter(hardwareMap);
    }

    public void initCameras(Alliance.Color allianceColor, Field.StartingPosition startingPosition) {
        //initialize webcam
        Match.log("Initializing Webcam");
        telemetry.addData("Status", "Initializing Webcam, please wait");
        telemetry.update();
        this.webcam = new OpenCVWebcam();
        this.webcam.init(hardwareMap, telemetry, OpenCVWebcam.RING_COLOR_MIN, OpenCVWebcam.RING_COLOR_MAX);

        //initialize Vslam camera
        Match.log("Initializing VSLAM");
        telemetry.addData("Status", "Initializing VSLAM, please wait");
        telemetry.update();
        this.vslamCamera = new VslamCamera(hardwareMap);
    }


    /**
     * Stop the robot
     */
    public void stop() {
        //Stop all of our motors
        Match.log("Stopping robot");
        this.operationThreadPrimary.abort();
        this.operationThreadSecondary.abort();
        this.operationThreadTertiary.abort();
        this.mecanumDriveTrain.stop();
        this.pickerArm.stop();

        //stop our camera
        this.vslamCamera.stop();
        Match.log(("Robot stopped"));
    }

    /**
     * Returns a string representing the status of the motors of the robot
     *
     * @return Motor Status
     */
    public String getMotorStatus() {
        if (this.mecanumDriveTrain == null) {
            return "Drivetrain not initialized";
        }
        else {
            return this.mecanumDriveTrain.getStatus();
        }
    }

    /**
     * Check if an operation has been completed
     *
     * @param operation - the operation to see if it is completed
     * @return
     */
    public boolean operationCompleted(Operation operation) {
        if (!operation.isAborted()) {
            switch (operation.getType()) {
                case FOLLOW_TRAJECTORY: {
                    FollowTrajectory followTrajectoryOperation = (FollowTrajectory) operation;
                    return followTrajectoryOperation.isComplete(mecanumDriveTrain, vslamCamera);
                }
                case DRIVE_TO_POSITION: {
                    DriveToPositionOperation driveToPositionOperation = (DriveToPositionOperation) operation;
                    return driveToPositionOperation.isComplete(mecanumDriveTrain, vslamCamera);
                }
                case DRIVE_FOR_DISTANCE: {
                    DistanceOperation distanceOperation = (DistanceOperation) operation;
                    return distanceOperation.isComplete(mecanumDriveTrain);
                }
                case DRIVE_IN_DIRECTION: {
                    DistanceInDirectionOperation distanceInDirectionOperation = (DistanceInDirectionOperation) operation;
                    return distanceInDirectionOperation.isComplete(this.mecanumDriveTrain, Math.toDegrees(getCurrentTheta()));
                }
                case STRAFE_LEFT_FOR_DISTANCE: {
                    StrafeLeftForDistanceOperation strafeForDistanceOperation = (StrafeLeftForDistanceOperation) operation;
                    return strafeForDistanceOperation.isComplete(mecanumDriveTrain);
                }
                case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING: {
                    StrafeLeftForDistanceWithHeadingOperation gyroscopicStrafeForDistanceOperation = (StrafeLeftForDistanceWithHeadingOperation) operation;
                    return gyroscopicStrafeForDistanceOperation.isComplete(mecanumDriveTrain, Math.toDegrees(getCurrentTheta()));
                }
                case ROTATION: {
                    ClockwiseRotationOperation clockwiseRotationOperation = (ClockwiseRotationOperation) operation;
                    return clockwiseRotationOperation.isComplete(mecanumDriveTrain);
                }
                case DRIVE_FOR_TIME: {
                    DriveForTimeOperation driveForTimeOperation = (DriveForTimeOperation) operation;
                    return driveForTimeOperation.isComplete(mecanumDriveTrain);
                }
                case BEARING: {
                    BearingOperation bearingOperation = (BearingOperation) operation;
                    return bearingOperation.isComplete(this.mecanumDriveTrain, Math.toDegrees(getCurrentTheta()));
                }
                case WAIT_TIME: {
                    WaitOperation waitTimeOperation = (WaitOperation) operation;
                    return waitTimeOperation.isComplete();
                }
                case PICKER_OPERATION: {
                    PickerOperation pickerOperation = (PickerOperation) operation;
                    return pickerOperation.isComplete(this.pickerArm);
                }
                case SHOOTER: {
                    ShootingOperation shootingOperation = (ShootingOperation) operation;
                    return shooter.isComplete(shootingOperation);
                }
                case INTAKE: {
                    IntakeOperation intakeOperation = (IntakeOperation) operation;
                    return intake.isComplete(intakeOperation);
                }
            }
        }
        return true;
    }

    /**
     * execute an operation
     *
     * @param operation - the operation to execute
     */
    public void executeOperation(Operation operation) {
        if (!operation.isAborted()) {
            switch (operation.getType()) {
                case FOLLOW_TRAJECTORY: {
                    this.mecanumDriveTrain.handleOperation((FollowTrajectory) operation);
                    break;
                }
                case DRIVE_TO_POSITION: {
                    this.mecanumDriveTrain.handleOperation((DriveToPositionOperation) operation);
                    break;
                }
                case DRIVE_FOR_DISTANCE: {
                    this.mecanumDriveTrain.handleOperation((DistanceOperation) operation);
                    break;
                }
                case DRIVE_IN_DIRECTION: {
                    this.mecanumDriveTrain.handleOperation((DistanceInDirectionOperation) operation);
                    break;
                }
                case STRAFE_LEFT_FOR_DISTANCE: {
                    this.mecanumDriveTrain.handleOperation((StrafeLeftForDistanceOperation) operation);
                    break;
                }
                case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING: {
                    this.mecanumDriveTrain.handleOperation((StrafeLeftForDistanceWithHeadingOperation) operation);
                    break;
                }
                case DRIVE_FOR_TIME: {
                    this.mecanumDriveTrain.handleOperation((DriveForTimeOperation) operation);
                    break;
                }
                case ROTATION: {
                    this.mecanumDriveTrain.handleOperation((ClockwiseRotationOperation) operation);
                    break;
                }
                case BEARING: {
                    this.mecanumDriveTrain.handleOperation((BearingOperation) operation);
                    break;
                }
                case WAIT_TIME: {
                    //don't have to do anything to execute the wait operation
                    break;
                }
                case PICKER_OPERATION: {
                    PickerOperation pickerOperation = (PickerOperation) operation;
                    this.pickerArm.handleOperation(pickerOperation);
                    break;
                }
                case SHOOTER: {
                    ShootingOperation shootingOperation = (ShootingOperation) operation;
                    this.shooter.handleOperation(shootingOperation, getPose());
                    break;
                }
                case INTAKE: {
                    IntakeOperation intakeOperation = (IntakeOperation) operation;
                    this.intake.handleOperation(intakeOperation);
                    break;
                }
            }
        }
    }

    public void abortOperation(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_FOR_DISTANCE:
            case DRIVE_IN_DIRECTION:
            case FOLLOW_TRAJECTORY:
            case DRIVE_TO_POSITION:
            case TURN:
            case STRAFE_LEFT_FOR_DISTANCE:
            case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING:
            case ROTATION:
            case BEARING:
            case DRIVE_FOR_TIME: {
                this.mecanumDriveTrain.stop();
                break;
            }
            case PICKER_OPERATION: {
                this.pickerArm.stop();
                break;
            }
            case SHOOTER:
            case INTAKE:
            case WAIT_TIME:
            case CAMERA: {
                break;
            }
        }
    }

    public void queuePrimaryOperation(Operation operation) {
        this.operationThreadPrimary.queueUpOperation(operation);
    }
    public void queueSecondaryOperation(Operation operation) {
        this.operationThreadSecondary.queueUpOperation(operation);
    }
    public void queueTertiaryOperation(Operation operation) {
        this.operationThreadTertiary.queueUpOperation(operation);
    }

    /**
     * Returns the current x value of robot's center in mms
     * @return the current x position in mms
     */
    public double getCurrentX() {
        return this.vslamCamera.getPoseEstimate().getX()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current y value of robot's center in mms
     * @return the current y position in mms
     */
    public double getCurrentY() {
        return this.vslamCamera.getPoseEstimate().getY()*Field.MM_PER_INCH;
    }

    /**
     * Returns the current heading of the robot in radians
     * @return
     */
    public double getCurrentTheta() {
        return this.vslamCamera.getPoseEstimate().getHeading();}

    public boolean allOperationsCompleted() {
        return primaryOperationsCompleted() && secondaryOperationsCompleted() && tertiaryOperationsCompleted();
    }

    public boolean primaryOperationsCompleted() {
        return !this.operationThreadPrimary.hasEntries();
    }

    public boolean secondaryOperationsCompleted() {
        return !this.operationThreadSecondary.hasEntries();
    }

    public boolean tertiaryOperationsCompleted() {
        return !this.operationThreadTertiary.hasEntries();
    }

    public String getPosition() {
        return this.vslamCamera.getPoseEstimate().toString();
    }

    public boolean havePosition() {
        return vslamCamera.havePosition();
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean fullyInitialized() {
        return this.everythingButCamerasInitialized && this.vslamCamera.isInitialized();
    }

    public String getPickerArmStatus() {
        if (this.pickerArm == null) {
            return "Not init";
        }
        return this.pickerArm.getStatus();
    }

    public void handleDriveTrain(Gamepad gamePad1) {
        if (this.primaryOperationsCompleted()) {
            if (shootingEndGameRings) {
                if (!firstRingShot) {
                    if (!firstRingQueued) {
                        AutonomousHelper.shootRingAtPosition(this, Field.POWER_SHOT_SHOOTING_POSE_RIGHT);
                        firstRingQueued = true;
                    }
                    else {
                        firstRingShot = this.primaryOperationsCompleted();
                    }
                }
                else if (!secondRingShot) {
                    if (!secondRingQueued) {
                        AutonomousHelper.shootRingAtPosition(this, Field.POWER_SHOT_SHOOTING_POSE_CENTER);
                        secondRingQueued = true;
                    }
                    else {
                        secondRingShot = this.primaryOperationsCompleted();
                    }
                }
                else if (!thirdRingShot) {
                    if (!thirdRingQueued) {
                        AutonomousHelper.shootRingAtPosition(this, Field.POWER_SHOT_SHOOTING_POSE_LEFT);
                        thirdRingQueued = true;
                    }
                    else {
                        thirdRingShot = this.primaryOperationsCompleted();
                    }
                }
            }
            if (gamePad1.left_stick_button) {
                this.shooter.setShooterSpeed(Shooter.RIGHT_POWER_TARGET_SPEED);
                this.queuePrimaryOperation(new DriveToPositionOperation(Field.POWER_SHOT_SHOOTING_POSE_RIGHT, 1, "Get to Power shot position"));
                this.queuePrimaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.STOP_INTAKE, "Stop intake"));
                this.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOTING_POSITION, "Shooting position for shooter"));
                shootingEndGameRings = true;
                firstRingShot = firstRingQueued = secondRingShot = secondRingQueued = thirdRingShot = thirdRingQueued = false;
            }
            else if (gamePad1.right_stick_button) {
                this.shooter.setShooterSpeed(Shooter.HIGH_GOAL_SPEED);
                this.queuePrimaryOperation(new DriveToPositionOperation(Field.TOWER_SHOOTING_POSE, 0,"Get to tower position"));
                this.queuePrimaryOperation(new BearingOperation(0, "Align"));
                //this.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOTING_POSITION, "Shooting position for shooter"));
                //this.queuePrimaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.STOP_INTAKE, "Stop intake"));
            }
            else if (gamePad1.y) {
                this.queuePrimaryOperation(new DriveToPositionOperation(Field.DROP_ZONE_POSE, 0, "Get to drop zone"));
            }
            else if (gamePad1.right_trigger != 0) {
                this.queuePrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(-Field.DISTANCE_BETWEEN_POWER_SHOTS, 0, AutonomousHelper.SUPER_CAUTIOUS_SPEED, "Move left"));
                this.queuePrimaryOperation(new BearingOperation(0, "Align"));
            }
            else if (gamePad1.a) {
                this.queuePrimaryOperation(new BearingOperation(0, "Align"));
            }
            else if (gamePad1.start) {
                this.queuePrimaryOperation(new DistanceOperation(Field.DISTANCE_BETWEEN_POWER_SHOTS, AutonomousHelper.SUPER_CAUTIOUS_SPEED, "Move forward a bit"));
            }
            else if (gamePad1.back) {
                this.queuePrimaryOperation(new DistanceOperation(-Field.DISTANCE_BETWEEN_POWER_SHOTS, AutonomousHelper.SUPER_CAUTIOUS_SPEED, "Move forward a bit"));
            }
        }
        if (this.primaryOperationsCompleted()) {
            double multiplier = 0.6;
            double x = Math.pow(gamePad1.left_stick_x, 3) * multiplier; // Get left joystick's x-axis value.
            double y = -Math.pow(gamePad1.left_stick_y, 3) * multiplier; // Get left joystick's y-axis value.

            double rotation = Math.pow(gamePad1.right_stick_x, 3) * 0.5; // Get right joystick's x-axis value for rotation

            this.mecanumDriveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation);
        }
    }

    public void handlePicker(Gamepad gamePad2) {
        if (this.secondaryOperationsCompleted()) {
            if (gamePad2.a) {
                this.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_1, "Level1"));
            }
            if (gamePad2.b) {
                this.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.WOBBLE_PICKUP, "Wobble pickup"));
            }
            if (gamePad2.y) {
                this.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_3, "Level3"));
            }
            if (gamePad2.x) {
                this.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_4, "Level4"));
            }
            if (gamePad2.left_stick_button) {
                this.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.COMPACT, "Compact"));
            }
        }

        float shoulderPower = -gamePad2.left_stick_y; // Get left joystick's y-axis value.
        float winchPower = -gamePad2.right_stick_y; // Get right joystick's y-axis value.
        //only change shoulder or winch position if queued operations are all done
        if (this.primaryOperationsCompleted()) {
            //handle angle of arm
            if (Math.abs(shoulderPower) > 0.1) {
                if (gamePad2.left_trigger > 0) {
                    this.pickerArm.raiseVertically(10*shoulderPower);
                }
                else {
                    if (shoulderPower > 0) {
                        this.pickerArm.incrementShoulderPosition();
                    } else {
                        this.pickerArm.decrementShoulderPosition();
                    }
                }
            }
            //handle extension of arm
            if (gamePad2.left_trigger > 0) {
                if (Math.abs(winchPower) > 0.1) {
                    this.pickerArm.extendHorizontally(10*winchPower);
                }
             } else {
                if (winchPower > 0) {
                    this.pickerArm.incrementWinchPosition();
                } else if (winchPower < 0) {
                    this.pickerArm.decrementWinchPosition();
                }
                else {
                    //do nothing if the winch power is zero
                    //this helps us maintain the winch position
                }
            }
        }

        if (gamePad2.right_trigger > 0) {
            this.pickerArm.closeGripper();
        }
        if (gamePad2.right_bumper) {
            this.pickerArm.openGripper();
        }
        if (tertiaryOperationsCompleted() && gamePad2.left_bumper) {
            this.queueTertiaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.WIGGLE, "Wiggle platform"));
        }
    }

    public void handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            this.operationThreadTertiary.abort();
        }
        this.handleDriveTrain(gamePad1);
        this.handlePicker(gamePad2);
        this.handleIntake(gamePad1, gamePad2);
        this.handleShooter(gamePad1);
    }

    public void handleIntake(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.dpad_right) {
            turnIntakeOn();
        }
        if (gamePad1.dpad_left) {
            turnIntakeOff();
        }

        if (gamePad2.dpad_up) {
            if (gamePad2.left_trigger != 0) {
                intake.raisePlatform();
            }
            else {
                shooter.lowerShooter();
            }
        }
        else if (gamePad2.dpad_down) {
            if (gamePad2.left_trigger != 0) {
                intake.lowerPlatform();
            }
            else {
                shooter.raiseShooter();
            }
        }

        if (gamePad2.dpad_right) {
            intake.assumeShootingPosition();
        }
        else if (gamePad2.dpad_left) {
            intake.assumeIntakePosition();
        }
    }

    public void turnIntakeOff() {
        this.shooter.resumeShooterSpeed();
        this.intake.stopIntake();
        this.shooter.assumeShootingPosition();
    }


    public void handleShooter(Gamepad gamepad1) {
        if (primaryOperationsCompleted() && gamepad1.right_bumper) {
            queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_HIGH_GOAL, "Power for high goal"));
            queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT, "Shoot ring 1"));
            queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT, "Shoot ring 2"));
            queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT, "Shoot ring 3"));
        }
        if (shootingHighGoal || shootingPowerTarget) {
            //if we are in the process of shooting, stop shooting if neither bumper or trigger is pressed
            if (!gamepad1.left_bumper && gamepad1.left_trigger == 0) {
                shootingHighGoal = shootingPowerTarget = false;
                shooter.unshoot();
            }
        }
        //shoot high goal if the left bumper is pressed
        else if (gamepad1.left_bumper) {
            shooter.shootHighGoal();
            //mark shooting high goal in progress
            shootingHighGoal = true;
        }
        //shoot power target if the left trigger is pressed
        else if (gamepad1.left_trigger != 0) {
            shooter.shootPowerTarget();
            //mark shooting power target in progress
            shootingPowerTarget = true;
        }

        if (gamepad1.dpad_down) {
            shooter.decreaseSpeed();
        }
        else if (gamepad1.dpad_up) {
            shooter.increaseSpeed();
        }
        if (gamepad1.a) {
            shooter.decrementTriggerServo();
        }
        else if (gamepad1.b) {
            shooter.incrementTriggerServo();
        }
    }

    public void turnIntakeOn() {
        this.intake.startIntake();
        this.shooter.assumeIntakePosition();
    }
    public Field.RingCount getNumberOfRings() {
        return this.webcam.getNumberOfRings();
    }

    public void setPose(Pose2d pose) {
        this.mecanumDriveTrain.setLocalizer(vslamCamera);
        this.vslamCamera.stop();
        this.vslamCamera.start();
        this.vslamCamera.setStartingPose(pose);
    }

    public void reset() {
        if (this.mecanumDriveTrain != null) {
            this.mecanumDriveTrain.ensureWheelDirection();
        }
        if (this.pickerArm != null) {
            pickerArm.ensureMotorDirection();
        }
        if (this.intake != null) {
            intake.ensureMotorDirection();
        }
        if (this.shooter != null) {
            shooter.ensureMotorDirection();
        }
        intakeIsOn = shootingHighGoal = shootingPowerTarget = false;
        startVSLAM();
        this.webcam.start();
    }

    public String getShooterStatus() {
        return this.shooter.getStatus();
    }

    public String getIntakeStatus() {
        return this.intake.getStatus();
    }

    public Pose2d getPose() {
        return this.vslamCamera.getPoseEstimate();
    }

    public void startVSLAM() {
        this.vslamCamera.start();
    }

    public String getLastTrajectoryError() {
        return this.vslamCamera.getLastError();
    }

    public boolean ableToProcessImages() {
        return true;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return this.mecanumDriveTrain.trajectoryBuilder(startPose);
    }
    public TrajectoryBuilder accurateTrajectoryBuilder(Pose2d startPose) {
        return this.mecanumDriveTrain.accurateTrajectoryBuilder(startPose);
    }

    public OpenCVWebcam getWebcam() {
        return this.webcam;
    }

    public void setShooterPIDF() {
        this.shooter.setPIDF();
    }
}
