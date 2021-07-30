package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveToPositionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;

import java.util.Date;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="PowerShotsV2", group="Phoebe")

public class PowerShotsV2 extends AutonomousHelper {
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!Field.isInitialized()) {
            //RobotLog.i("SilverTitans: Initializing trajectories, please wait");
            telemetry.addData("Status", "Trajectories initializing, please wait. " +
                    (30 - (int)(new Date().getTime() - initStartTime.getTime())/1000));
        }
        else if (robot.fullyInitialized()) {
            if (!wobbleRaisedInitially) {
                robot.setPose(Field.STARTING_POSE);
                //robot.startVSLAM();
                robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Lift wobble goal slightly"));
                wobbleRaisedInitially = true;
                Match.log("Init complete");
            }
            else if (!robot.havePosition()) {
                telemetry.addData("Status", "Waiting for position, please wait");
            }
            else if (robot.allOperationsCompleted()) {
                if (cameraWorks || robot.ableToProcessImages()) {
                    cameraWorks = true;
                    double xError = robot.getCurrentX() / Field.MM_PER_INCH - Field.STARTING_POSE.getX();
                    double yError = robot.getCurrentY() / Field.MM_PER_INCH - Field.STARTING_POSE.getY();
                    double bearingError = AngleUnit.normalizeRadians(robot.getCurrentTheta()) - AngleUnit.normalizeRadians(Field.STARTING_POSE.getHeading());
                    if ((Math.abs(xError) > ALLOWED_POSITIONAL_ERROR)
                            || (Math.abs(yError) > ALLOWED_POSITIONAL_ERROR
                            || (Math.abs(bearingError) > ALLOWED_BEARING_ERROR))) {
                        telemetry.addData("Status", "PositionError:" + Field.STARTING_POSE + " v/s" + robot.getPosition() + ", please init again");
                        robot.setPose(Field.STARTING_POSE);
                    } else {
                        match.setNumberOfRings(robot.getNumberOfRings());
                        match.updateTelemetry(telemetry,"Ready, let's go");
                    }
                } else {
                    telemetry.addData("Status", "Unable to see, please init again");
                }
            }
            else {
                telemetry.addData("Status", "Waiting for wobble to lift, please wait");
            }
        }
        else {
            telemetry.addData("Status", "Cameras initializing, please wait");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        match.setStart();
        match.setNumberOfRings(robot.getNumberOfRings());
    }

    @Override
    public void loop() {
        if (!wobbleLifted) {
            if (!queuedWobbleLift) {
                Match.log("Lifting wobble");
                queuedWobbleLift();
                queuedWobbleLift = true;
            }
            wobbleLifted = robot.allOperationsCompleted();
        } else if (!firstWobbleDeposited) {
            if (!firstWobbleDepositQueued) {
                match.setNumberOfRings(robot.getNumberOfRings());
                Match.log("Depositing wobble goal");
                queueFirstWobbleGoalDeposit();
                firstWobbleDepositQueued = true;
            }
            firstWobbleDeposited = robot.allOperationsCompleted();
        } else if (!ringsShootingPositionReached) {
            if (!ringShootingPositionQueued) {
                queueReachShootingPosition();
            }
            else {
                ringsShootingPositionReached = robot.allOperationsCompleted();
            }
        }
        else if (!firstRingShot) {
            if (!firstRingShootingQueued) {
                queueFirstRingShooting();
                firstRingShootingQueued = true;
            }
            firstRingShot = robot.primaryOperationsCompleted();
        }
        else if (!secondRingShot) {
            if (!secondRingShootingQueued) {
                queueSecondRingShooting();
                secondRingShootingQueued = true;
            }
            secondRingShot = robot.primaryOperationsCompleted();
        }
        else if (!thirdRingShot) {
            if (!thirdRingShootingQueued) {
                queueThirdRingShooting();
                thirdRingShootingQueued = true;
            }
            thirdRingShot = robot.primaryOperationsCompleted();
        } else if (!returnedForSecondWobble) {
            if (!returnForSecondWobbleQueued) {
                Match.log("Grabbing second wobble goal");
                returnForSecondWobble();
            }
            returnedForSecondWobble = robot.primaryOperationsCompleted();
        }
        else if (!collectedSecondWobble) {
            if (!collectionOfSecondWobbleQueued) {
                Match.log("Queuing collection for second wobble");
                queueSecondWobbleCollection();
                collectionOfSecondWobbleQueued = true;
            }
            collectedSecondWobble = robot.allOperationsCompleted();
        }
        else if (!secondWobbleGoalDeposited) {
            if (!secondWobbleGoalDepositQueued) {
                Match.log("Depositing second wobble goal");
                queueDepositSecondWobble();
            }
            secondWobbleGoalDeposited = robot.allOperationsCompleted();
        }
        else if (!navigated) {
            if (!navigationQueued) {
                Match.log("Navigating");
                queueNavigation();
            }
            navigated = robot.allOperationsCompleted();
        }

        match.updateTelemetry(telemetry, "Autonomous");
    }

    protected void queueRingDetermination() {
        Match.log("Finding number of rings");
        Field.RingCount numberOfRings = robot.getNumberOfRings();
        match.setNumberOfRings(numberOfRings);
        //generate the rest of the trajectories - hopefully this completes before we are done depositing first wobble
        match.getField().generateTrajectories(numberOfRings, true);
    }

    protected void queuedWobbleLift() {
        robot.queuePrimaryOperation(new DistanceOperation(INITIAL_FORWARD_MOVEMENT, SUPER_CAUTIOUS_SPEED, "Move up"));

        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OUT, "Out with the wobble"));

        robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOTING_POSITION,"Assume shooting position"));
        robot.queueTertiaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.LIFT_PLATFORM, "Lift platform"));
        robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_RIGHT_POWER_TARGET,"Turn on shooter for power target"));
    }

    protected void queueFirstWobbleGoalDeposit() {
        robot.queuePrimaryOperation(new FollowTrajectory(field.getFirstWobbleGoalDepositTrajectory(), "Get to deposit first wobble"));
        //open gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));

        //lower gripper to position to deposit wobble goal
        PickerOperation operationLower = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Lower gripper to deposit wobble goal");
        operationLower.setShoulderPosition(PickerArm.SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR);
        robot.queueSecondaryOperation(operationLower);
    }

    public void queueReachShootingPosition() {
        Trajectory trajectory = field.getReachPowerShotShootingPose();
        //only queue up operations to reach shooting position if we have a trajectory
        if (trajectory != null) {
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory, "Reach shooting pose"));
            robot.queuePrimaryOperation(new BearingOperation(0, "Align for right power target"));
            //raise gripper to position to clear wobble goal
            robot.queueTertiaryOperation(new PickerOperation(PickerOperation.PickerOperationType.VERTICAL, "Vertical"));
            ringShootingPositionQueued = true;
        }
        else {
            Thread.yield();
        }
    }

    public static void shootRingAtPosition(Robot robot, Pose2d desiredPose) {
        correctPositionTo(robot, desiredPose);
        //fire off ring
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT,"Shoot ring"));
    }

    public static void correctPositionTo(Robot robot, Pose2d desiredPose) {
        Match.log("Shooting: Current position=" + robot.getPosition() + " v/s "
                + desiredPose);
        double xError = desiredPose.getX()* Field.MM_PER_INCH - robot.getCurrentX();
        double yError = desiredPose.getY()*Field.MM_PER_INCH - robot.getCurrentY();
        if (xError != 0 || yError !=0) {
            robot.queuePrimaryOperation(new DriveToPositionOperation(desiredPose, "Correct position"));
        }
    }

    protected void queueFirstRingShooting() {
        shootRingAtPosition(robot, Field.POWER_SHOT_SHOOTING_POSE_RIGHT);
    }

    protected void queueSecondRingShooting() {
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_CENTER_POWER_TARGET,
                "Power for center target"));
        shootRingAtPosition(robot, Field.POWER_SHOT_SHOOTING_POSE_CENTER);
    }

    protected void queueThirdRingShooting() {
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_LEFT_POWER_TARGET,
                "Power for left target"));
        shootRingAtPosition(robot, Field.POWER_SHOT_SHOOTING_POSE_LEFT);
    }

    /**
     * Return close to where we started and grab the second wobble goal
     */
    protected void returnForSecondWobble() {
        Trajectory trajectory = field.getReachSecondWobbleFromPowerShotLeftTrajectory();
        if (trajectory != null) {
            robot.queueSecondaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_OFF, "Turn off shooter"));
            robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.WOBBLE_PICKUP, "Ready for second wobble"));
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory, "Reach second wobble"));
            returnForSecondWobbleQueued = true;
        }
    }

    protected void queueSecondWobbleCollection() {
        correctPositionTo(robot, new Pose2d(Field.SECOND_WOBBLE_PICK_UP.getX(), Field.SECOND_WOBBLE_PICK_UP.getY()));
        robot.queuePrimaryOperation(
                new BearingOperation(SECOND_WOBBLE_PICKUP_HEADING, "Face second wobble"));

        //close gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Close gripper on wobble goal"));
        //robot.queuePrimaryOperation(new WaitOperation(1000, "Wait"));
        //get gripper to position to raise wobble goal
        PickerOperation operationHover = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Get wobble to be just above floor");
        operationHover.setShoulderPosition(PickerArm.SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR);
        robot.queuePrimaryOperation(operationHover);
    }

    /**
     * Deposit second wobble goal exactly where we deposited the first one
     */
    protected void queueDepositSecondWobble() {
        Trajectory trajectory = field.getSecondWobbleGoalDepositTrajectory();
        if (trajectory != null) {
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory, "Deposit second wobble"));
            //robot.queuePrimaryOperation(new WaitOperation(500, "Wait half a sec"));
            //deposit second wobble goal
            robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));
            secondWobbleGoalDepositQueued = true;
        }
    }

    protected void queueNavigation() {
        Trajectory trajectory = field.getNavigationTrajectory();
        if (trajectory != null) {
            robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.VERTICAL, "Go vertical"));
            //robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Close gripper"));
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory, "Navigate"));
            if (match.getNumberOfRings() == Field.RingCount.FOUR) {
                robot.queuePrimaryOperation(new BearingOperation(180, "Face four rings"));
            } else {
                robot.queuePrimaryOperation(new BearingOperation(0, "Face dispenser"));
            }
            navigationQueued = true;
        }
    }

    @Override
    public void stop() {
        this.robot.stop();
    }
}