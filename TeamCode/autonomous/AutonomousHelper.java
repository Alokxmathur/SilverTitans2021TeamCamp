package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitUntilVuMarkOperation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.game.Alliance.Color.BLUE;
import static org.firstinspires.ftc.teamcode.game.Alliance.Color.RED;

public abstract class AutonomousHelper extends OpMode {

    protected Match match;
    protected Robot robot;
    protected Alliance.Color allianceColor;
    protected double headingToFoundation;

    public static final long VUFORIA_SETTLING_TIME = 2500; //msecs for vuforia to see image
    public static final double STARTING_POSITION = Field.TILE_WIDTH + MecanumDriveTrain.DRIVE_TRAIN_WIDTH / 2;
    public static final double CAUTIOUS_SPEED = 0.6;
    public static final double FAST_SPEED = 1.0;
    public static final double START_TO_CLEAR_TAPE = 1.5 * Field.TILE_WIDTH;
    public static final double RETRACTION_FROM_QUARRY = 0f * Field.MM_PER_INCH;

    protected boolean initialMovementDone, initialMovementQueued;
    protected boolean numberOfRingsDetermined, determinationQueued;
    protected boolean wobbleGoalDeposited, wobbleGoalDepositQueued;
    protected boolean navigated, navigationQueued;

    int numberOfRingsOnStack;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color allianceColor) {
        this.allianceColor = allianceColor;
        this.match = Match.getNewInstance();
        match.init();
        match.setAlliance(allianceColor);
        this.robot = match.getRobot();
        this.robot.init(hardwareMap, telemetry, match, allianceColor);
        headingToFoundation = allianceColor == Alliance.Color.BLUE ? 90 : -90;

        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.fullyInitialized()) {
            //keep looking for the sky stone position in the quarry
            match.setNumberOfRings(robot.getNumberOfRings());
            //update driver station with number of rings
            telemetry.addData("Alliance", allianceColor);
            telemetry.addData("Rings:", match.getNumberOfRings());
            telemetry.addData("Status", "Ready to autonomous");
            telemetry.addData("Motors", robot.getMotorStatus());
            telemetry.addData("Picker", robot.getPickerArmStatus());
            telemetry.update();
        }
        else {
            telemetry.addData("status", "Waiting for vuForia to finish, please wait");
            telemetry.update();
        }
    }

    public void loop() {
        if (!initialMovementDone) {
            //initiate Phoebe
            if (!initialMovementQueued) {
                this.queueInitialOperations();
                initialMovementQueued = true;
            }
            initialMovementDone = robot.primaryOperationsCompleted();
        } else if (!numberOfRingsDetermined) {
            if (!determinationQueued) {
                Match.log("Determining number of rings");
                queueRingDetermination();
                determinationQueued = true;
            }
            numberOfRingsDetermined = robot.operationsCompleted();
        } else if (!wobbleGoalDeposited) {
            if (!wobbleGoalDepositQueued) {
                Match.log("Depositing wobble goal");
                queueWobbleGoalDeposit();
                wobbleGoalDepositQueued = true;
            }
            wobbleGoalDeposited = robot.operationsCompleted();
        } else if (!navigated) {
            if (!navigationQueued) {
                Match.log("Navigating");
                queueNavigation();
                navigationQueued = true;
            }
            navigated = robot.operationsCompleted();
        }
    }

    /**
     * Turn on led lights, lower the foundation gripper on tertiary thread
     * Get gripper to hover position and open it on secondary thread
     * <p>
     * We do all of this in on the secondary thread so that we don't have to wait for them to
     * complete before reaching the bottom sky stone
     */
    protected void queueInitialOperations() {
        //queue following operations on tertiary thread
        //turn on flash
        robot.queueTertiaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //lower foundation gripper so camera can see
        robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower foundation gripper"));

        //queue following operations on secondary thread
        //get gripper to hover position
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Gripper to hover"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper"));
    }

    protected void queueRingDetermination() {
        numberOfRingsOnStack = (int) (Math.random() * 3);
        if (numberOfRingsOnStack == 2) {
            numberOfRingsOnStack = 4;
        }
    }

    protected void queueWobbleGoalDeposit() {
        double forwardMovement=0, rightMovement=0;
        if (numberOfRingsOnStack == 0) {
            forwardMovement  = 2*Field.TILE_WIDTH;
            rightMovement = 1*Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 1) {
            forwardMovement  = 3*Field.TILE_WIDTH;
            rightMovement = 0;
        }
        if (numberOfRingsOnStack == 4) {
            forwardMovement  = 4*Field.TILE_WIDTH;
            rightMovement = 1*Field.TILE_WIDTH;
        }

        robot.queuePrimaryOperation(
                new DriveForDistanceOperation(forwardMovement, CAUTIOUS_SPEED, "Move to the right box"));
        robot.queuePrimaryOperation(
                new StrafeLeftForDistanceOperation(-rightMovement, CAUTIOUS_SPEED, "Strafe to the box")
        );

    }

    protected void queueNavigation() {
        double backwardsMovement=0;
        if (numberOfRingsOnStack == 0) {
            backwardsMovement  = -1*Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 1) {
            backwardsMovement  = 0*Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 4) {
            backwardsMovement  = 1*Field.TILE_WIDTH;
        }
        robot.queuePrimaryOperation(
                new DriveForDistanceOperation(-backwardsMovement, CAUTIOUS_SPEED, "Navigate"));
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        match.setStart();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.robot.stop();
    }

    public Alliance.Color getAlliance() {
        return allianceColor;
    }


    protected void flash() {
        //turn off flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_OFF, "Turn off flash"));
        //wait
        robot.queueSecondaryOperation(new WaitOperation(1000, "Flash"));
        //turn on flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //wait
        robot.queueSecondaryOperation(new WaitOperation(1000, "Flash"));
        //turn off flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_OFF, "Turn off flash"));
    }

}
