package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.FoundationGripper;
import org.firstinspires.ftc.teamcode.robot.components.PhoebeColorSensor;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.SideGrippers;
import org.firstinspires.ftc.teamcode.robot.components.camera.WebCam;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilColorOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilVuMarkOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.RotateUntilVuMarkOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ClockwiseRotationOperation;
import org.firstinspires.ftc.teamcode.robot.operations.SideGrippersOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitUntilVuMarkOperation;

/**
 * This class represents our robot.
 * The config on the robot needs to have the following entries defined:
 * *
 * rightDrive: the right motor of the drive train
 * leftDrive: the left motor of the drive tpenrain
 */

public class Robot {

    public static final double GRIPPER_LEFT_DISPLACEMENT = .5 * Field.MM_PER_INCH;
    public static final double GRIPPER_FORWARD_DISPLACEMENT = 6f * Field.MM_PER_INCH;
    public static final double FOUNDATION_GRIPPER_FORWARD_DISPLACEMENT = GRIPPER_FORWARD_DISPLACEMENT
            + 5.375 * Field.MM_PER_INCH;

    public static final double LENGTH = 22*Field.MM_PER_INCH;

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;
    OperationThread operationThreadTertiary;

    MecanumDriveTrain mecanumDriveTrain = null;
    PickerArm pickerArm = null;
    FoundationGripper foundationGripper;
    SideGrippers sideGrippers;
    PhoebeColorSensor phoebeColorSensor;

    boolean everythingButVuforiaInitialized = false;

    //Our sensors etc.

    WebCam camera;
    //RealSenseCamera realSenseCamera;

    //our state
    String state = "pre-initialized";

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    /**
     * Initialize our robot
     * We set our alliance and our starting position based on finding a VuMark
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match, Alliance.Color allianceColor) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();
        this.operationThreadPrimary = new OperationThread(this, "Primary");
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary");
        operationThreadSecondary.start();
        this.operationThreadTertiary = new OperationThread(this, "Tertiary");
        operationThreadTertiary.start();
        Match.log("Started operations threads");

        //initialize our components

        initFoundatonGripper();
        initCamera(allianceColor);
        this.camera.turnLedOn();
        initDriveTrain();
        this.camera.turnLedOff();
        initPickerArm();
        this.camera.turnLedOn();
        initSideGrippers();
        this.camera.turnLedOff();
        //initColorSensor();
        //initRealSenseCamera();

        this.everythingButVuforiaInitialized = true;
        telemetry.update();
    }

    public void initDriveTrain() {
        //Create our drive train
        telemetry.addData("Status", "Initializing drive train, please wait");
        telemetry.update();
        this.mecanumDriveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
    }

    public void initPickerArm() {
        telemetry.addData("Status", "Initializing picker arm, please wait");
        telemetry.update();
        this.pickerArm = new PickerArm(hardwareMap, telemetry);
    }

    public void initFoundatonGripper() {
        telemetry.addData("Status","Initializing foundation gripper, please wait");
        telemetry.update();
        this.foundationGripper = new FoundationGripper(hardwareMap, telemetry);
    }

    public void initSideGrippers() {
        telemetry.addData("Status","Initializing side grippers, please wait");
        telemetry.update();
        this.sideGrippers = new SideGrippers(hardwareMap, telemetry);
        this.sideGrippers.raiseLeftGripper();
        this.sideGrippers.raiseRightGripper();
    }

    public void initColorSensor() {
        telemetry.addData("Status","Initializing phoebeColorSensor, please wait");
        telemetry.update();
        this.phoebeColorSensor = new PhoebeColorSensor(hardwareMap, telemetry);
    }

    public void initCamera(Alliance.Color allianceColor) {
        //initialize camera
        telemetry.addData("Status", "Initializing VuForia, please wait");
        telemetry.update();
        this.camera = new WebCam();
        this.camera.init(hardwareMap, telemetry, allianceColor);
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
        Match.log(("Robot stopped"));
    }

    /**
     * Returns a string representing the status of the motors of the robot
     *
     * @return Motor Status
     */
    public String getMotorStatus() {
        return this.mecanumDriveTrain.getStatus();
    }

    /**
     * Check if an operation has been completed
     *
     * @param operation - the operation to see if it is completed
     * @return
     */
    public boolean operationCompleted(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_FOR_DISTANCE: {
                DriveForDistanceOperation driveForDistanceOperation = (DriveForDistanceOperation) operation;
                return driveForDistanceOperation.isComplete(mecanumDriveTrain);
            }
            case DRIVE_UNTIL_VUMARK: {
                DriveUntilVuMarkOperation driveUntilVuMarkOperation = (DriveUntilVuMarkOperation) operation;
                return driveUntilVuMarkOperation.isComplete(mecanumDriveTrain, this);
            }
            case ROTATE_UNTIL_VUMARK: {
                RotateUntilVuMarkOperation rotateUntilVuMarkOperation = (RotateUntilVuMarkOperation) operation;
                return rotateUntilVuMarkOperation.isComplete(mecanumDriveTrain, this);
            }
            case DRIVE_UNTIL_COLOR: {
                DriveUntilColorOperation driveUntilColorOperation = (DriveUntilColorOperation) operation;
                return driveUntilColorOperation.isComplete(this.phoebeColorSensor, mecanumDriveTrain);
            }
            case GYROSCOPIC_DRIVE: {
                DriveForDistanceInDirectionOperation driveForDistanceInDirectionOperation = (DriveForDistanceInDirectionOperation) operation;
                return driveForDistanceInDirectionOperation.isComplete(this.mecanumDriveTrain);
            }
            case STRAFE_LEFT_FOR_DISTANCE: {
                StrafeLeftForDistanceOperation strafeForDistanceOperation = (StrafeLeftForDistanceOperation) operation;
                return strafeForDistanceOperation.isComplete(mecanumDriveTrain);
            }
            case STRAFE_LEFT_FOR_TIME: {
                return ((StrafeLeftForTimeOperation) operation).isComplete(mecanumDriveTrain);
            }
            case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING: {
                StrafeLeftForDistanceWithHeadingOperation gyroscopicStrafeForDistanceOperation = (StrafeLeftForDistanceWithHeadingOperation) operation;
                return gyroscopicStrafeForDistanceOperation.isComplete(mecanumDriveTrain);
            }
            case STRAFE_UNTIL_XY: {
                StrafeUntilXYOperation strafeUntilXYOperation = (StrafeUntilXYOperation) operation;
                return strafeUntilXYOperation.isComplete(mecanumDriveTrain, this);
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
                GyroscopicBearingOperation bearingOperation = (GyroscopicBearingOperation) operation;
                return bearingOperation.isComplete(this.mecanumDriveTrain);
            }
            case WAIT_TIME: {
                WaitOperation waitTimeOperation = (WaitOperation) operation;
                return waitTimeOperation.isComplete();
            }
            case WAIT_UNTIL_VUMARK: {
                WaitUntilVuMarkOperation waitUntilVuMarkOperation = (WaitUntilVuMarkOperation) operation;
                return waitUntilVuMarkOperation.isComplete(this);
            }
            case CAMERA: {
                CameraOperation cameraOperation = (CameraOperation) operation;
                return cameraOperation.isComplete();
            }
            case PICKER_OPERATION: {
                PickerOperation pickerOperation = (PickerOperation) operation;
                return pickerOperation.isComplete(this.pickerArm);
            }
            case FOUNDATION_GRIPPER: {
                FoundationGripperOperation foundationGripperOperation = (FoundationGripperOperation) operation;
                return foundationGripper.isComplete(foundationGripperOperation);
            }
            case SIDE_GRIPPER: {
                SideGrippersOperation sideGrippersOperation = (SideGrippersOperation) operation;
                return sideGrippers.isComplete(sideGrippersOperation);
            }
        }
        return false;
    }

    public String findTarget() {
        return this.camera.findTarget();
    }

    /**
     * execute an operation
     *
     * @param operation - the operation to execute
     */
    public void executeOperation(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_FOR_DISTANCE: {
                this.mecanumDriveTrain.handleOperation((DriveForDistanceOperation) operation);
                break;
            }
            case GYROSCOPIC_DRIVE: {
                this.mecanumDriveTrain.handleOperation((DriveForDistanceInDirectionOperation) operation);
                break;
            }
            case DRIVE_UNTIL_VUMARK: {
                this.mecanumDriveTrain.handleOperation((DriveUntilVuMarkOperation) operation);
                break;
            }
            case ROTATE_UNTIL_VUMARK: {
                this.mecanumDriveTrain.handleOperation((RotateUntilVuMarkOperation) operation);
            }
            case DRIVE_UNTIL_COLOR: {
                this.mecanumDriveTrain.handleOperation((DriveUntilColorOperation) operation);
                break;
            }
            case STRAFE_LEFT_FOR_DISTANCE: {
                this.mecanumDriveTrain.handleOperation((StrafeLeftForDistanceOperation) operation);
                break;
            }
            case STRAFE_LEFT_FOR_TIME: {
                this.mecanumDriveTrain.handleOperation((StrafeLeftForTimeOperation) operation);
                break;
            }
            case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING: {
                this.mecanumDriveTrain.handleOperation((StrafeLeftForDistanceWithHeadingOperation) operation);
                break;
            }
            case STRAFE_UNTIL_XY: {
                this.mecanumDriveTrain.handleOperation((StrafeUntilXYOperation) operation);
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
                this.mecanumDriveTrain.handleOperation((GyroscopicBearingOperation) operation);
                break;
            }
            case WAIT_TIME: {
                //don't have to do anything to execute the wait operation
                break;
            }
            case WAIT_UNTIL_VUMARK: {
                WaitUntilVuMarkOperation waitUntilVuMarkOperation = (WaitUntilVuMarkOperation) operation;
                waitUntilVuMarkOperation.setStart();
                break;
            }
            case CAMERA: {
                CameraOperation cameraOperation = (CameraOperation) operation;
                this.camera.handleOperation(cameraOperation);
                break;
            }
            case PICKER_OPERATION: {
                PickerOperation pickerOperation = (PickerOperation) operation;
                this.pickerArm.handleOperation(pickerOperation);
                break;
            }
            case FOUNDATION_GRIPPER: {
                FoundationGripperOperation foundationGripperOperation = (FoundationGripperOperation) operation;
                this.foundationGripper.handleOperation(foundationGripperOperation);
                break;
            }
            case SIDE_GRIPPER: {
                SideGrippersOperation sideGrippersOperation = (SideGrippersOperation) operation;
                this.sideGrippers.handleOperation(sideGrippersOperation);
                break;
            }
        }
    }

    public void abortOperation(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_FOR_DISTANCE:
            case GYROSCOPIC_DRIVE:
            case DRIVE_UNTIL_VUMARK:
            case STRAFE_LEFT_FOR_DISTANCE:
            case STRAFE_LEFT_FOR_TIME:
            case STRAFE_LEFT_FOR_DISTANCE_WITH_HEADING:
            case STRAFE_UNTIL_XY:
            case ROTATION:
            case ROTATE_UNTIL_VUMARK:
            case BEARING:
            case DRIVE_UNTIL_COLOR:
            case DRIVE_FOR_TIME: {
                this.mecanumDriveTrain.stop();
                break;
            }
            case PICKER_OPERATION: {
                this.pickerArm.stop();
                break;
            }
            case WAIT_TIME:
            case WAIT_UNTIL_VUMARK:
            case FOUNDATION_GRIPPER:
            case SIDE_GRIPPER:
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
    public float getCurrentX() {
        return this.camera.getCurrentX();
    }

    public float getCurrentY() {
        return this.camera.getCurrentY();
    }

    public float getCurrentTheta() {return this.camera.getCurrentTheta();}

    public double getBearing() {
        //return this.camera.getCurrentBearing();
        return this.mecanumDriveTrain.getIMU().getBearing();
    }

    public boolean operationsCompleted() {
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
        return this.camera.getPosition();
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean fullyInitialized() {
        return this.everythingButVuforiaInitialized && this.camera.isInitialized();
    }

    public boolean isGyroCalibrated() {
        return this.mecanumDriveTrain.getIMU().isCalibrated();
    }

    public String getPickerArmStatus() {
        return this.pickerArm.getStatus();
    }

    public void handleDriveTrain(Gamepad gamePad1) {
        //Arcade drive - gamepad 1 is driver
        double multiplier = 0.6;
        double x = Math.pow(gamePad1.left_stick_x, 3) * multiplier; // Get left joystick's x-axis value.
        double y = -Math.pow(gamePad1.left_stick_y, 3) * multiplier; // Get left joystick's y-axis value.

        double rotation = Math.pow(gamePad1.right_stick_x, 3) * 0.375; // Get right joystick's y-axis value for rotation

        this.mecanumDriveTrain.drive(Math.atan2(x, y), Math.hypot(x, y), rotation, false);

        //if trigger is pressed, if the foundation gripper is not lowered, lower it
        if (gamePad1.right_trigger > 0) {
            if (!this.foundationGripper.isLowered()) {
                this.foundationGripper.lowerGripper();
            }
        }
        else {
            //if trigger is not pressed, if the gripper is lowered, raise it
            if (this.foundationGripper.isLowered()) {
                this.foundationGripper.raiseGripper();
            }
        }
    }

    public void handlePicker(Gamepad controller1, Gamepad controller2) {
        if (this.operationsCompleted()) {
            if (controller2.dpad_up) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level shoulder"));
            }
            else if (controller2.dpad_down) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_GRAB, "Lower shoulder"));
            }
            else if (controller2.dpad_left) {
                PickerOperation pickerOperation = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION, "Release position");
                pickerOperation.setShoulderPosition(PickerArm.SHOULDER_RELEASE_POSITION);
                this.queuePrimaryOperation(pickerOperation);
            }
            else if (controller2.dpad_right) {
                PickerOperation pickerOperation = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION, "90 degrees");
                pickerOperation.setShoulderPosition(PickerArm.SHOULDER_VERTICAL_POSITION);
                this.queuePrimaryOperation(pickerOperation);
            }
            if (controller2.a) {
                if (controller2.left_trigger > 0) {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_5, "Level5"));
                }
                else {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_1, "Level1"));
                }
            }
            if (controller2.b) {
                if (controller2.left_trigger > 0) {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_6, "Level6"));
                }
                else {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_2, "Level2"));
                }
            }
            if (controller2.y) {
                if (controller2.left_trigger > 0) {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_7, "Level7"));
                }
                else {
                    this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_3, "Level3"));
                }
            }
            if (controller2.x) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.LEVEL_4, "Level4"));
            }

            if (controller1.a) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.GRAB, "Position to grab stone"));
            }
            if (controller1.b) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Carry"));
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.GRAB, "Position to grab stone"));
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Grab stone"));
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Pick"));
            }

            //release capstone if controller 2 left bumper is pressed
            if (controller2.left_bumper) {
                this.pickerArm.releaseCapstone();
            }
            //hold capstone if controller 1 left bumper is pressed
            if (controller1.left_bumper) {
                this.pickerArm.holdCapStone();
            }
            //go vertical if controller 1 left trigger is pressed
            if (controller1.left_trigger > 0) {
                this.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_VERTICAL, "Vertical"));
            }
        }
        float shoulderPower = -controller2.left_stick_y; // Get left joystick's y-axis value.
        float winchPower = -controller2.right_stick_y; // Get right joystick's y-axis value.
        //only change shoulder or winch position if queued operations are all done
        if (this.operationsCompleted()) {
            //handle angle of arm
            if (Math.abs(shoulderPower) > 0.1) {
                if (controller2.left_trigger > 0) {
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
            if (controller2.left_trigger > 0) {
                if (Math.abs(winchPower) > 0.1) {
                    this.pickerArm.extendHorizontally(10*winchPower);
                }
             } else {
                this.pickerArm.setWinchPower(winchPower);
            }
        }


        if (controller2.right_trigger > 0) {
            this.pickerArm.closeGripper();
        }
        if (controller2.right_bumper) {
            this.pickerArm.openGripper();
        }
    }

    public void handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            this.operationThreadTertiary.abort();
        }
        this.handleDriveTrain(gamePad1);
        this.handlePicker(gamePad1, gamePad2);
    }

    public void lowerFoundationGripper() {
        this.foundationGripper.lowerGripper();
    }

    public int getNumberOfRings() {
        return this.camera.getNumberOfRings();
    }

}
