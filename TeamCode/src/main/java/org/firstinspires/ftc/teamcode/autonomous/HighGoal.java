package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="HighGoal", group="Phoebe")
//@Disabled
public class HighGoal extends PowerShots {
    public HighGoal() {
        super();
    }

    protected void queuedWobbleLift() {
        robot.queuePrimaryOperation(new DistanceOperation(INITIAL_FORWARD_MOVEMENT, SUPER_CAUTIOUS_SPEED, "Move up"));

        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OUT, "Out with the wobble"));

        robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOTING_POSITION,"Assume shooting position"));
        robot.queueTertiaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.LIFT_PLATFORM, "Lift platform"));
        robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_HIGH_GOAL,"Turn on shooter for high goal"));
    }

    protected void queueRingDetermination() {
        Field.RingCount numberOfRings = robot.getNumberOfRings();
        match.setNumberOfRings(numberOfRings);
        //generate the rest of the trajectories - hopefully this completes before we are done depositing first wobble
        match.getField().generateTrajectories(numberOfRings, false);
    }

    @Override
    public void queueReachShootingPosition() {
        Trajectory trajectory = field.getReachHighGoalShootingPose();
        //only queue up operations to reach shooting position if we have a trajectory
        if (trajectory != null) {
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory, "Reach shooting pose"));
            robot.queuePrimaryOperation(new BearingOperation(0, "Align"));
            //raise gripper to position to clear wobble goal
            robot.queueTertiaryOperation(new PickerOperation(PickerOperation.PickerOperationType.VERTICAL, "Vertical"));
            ringShootingPositionQueued = true;
        }
        else {
            Thread.yield();
        }
    }

    protected void queueFirstRingShooting() {
        shootRingAtPosition(robot, Field.TOWER_SHOOTING_POSE);
    }

    protected void queueSecondRingShooting() {
        //fire away
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT,"Shoot ring 2"));
    }

    protected void queueThirdRingShooting() {
        //fire away
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT,"Shoot ring 3"));
    }


    /**
     * Return close to where we started and grab the second wobble goal
     */
    protected void returnForSecondWobble() {
        Trajectory trajectory = field.getReachSecondWobbleFromHighGoalTrajectory();
        if (trajectory != null) {
            robot.queueSecondaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_OFF, "Turn off shooter"));
            robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.WOBBLE_PICKUP, "Ready for second wobble"));
            robot.queuePrimaryOperation(new FollowTrajectory(trajectory,
                    "Reach second wobble"));
            returnForSecondWobbleQueued = true;
        }
    }

}