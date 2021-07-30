package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PhoebeRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

/**
 * Created by Silver Titans on 9/16/17.
 */

public class Field {
    public static double DISTANCE_BETWEEN_POWER_SHOTS = 7.625*Field.MM_PER_INCH;
    public static double RING_WIDTH = 5*Field.MM_PER_INCH;
    public static final float MM_PER_INCH = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    //the width of each tile
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double FIELD_WIDTH = 6*TILE_WIDTH;
    public static final double TAPE_WIDTH = 2*MM_PER_INCH;

    public static volatile boolean initialized = false;
    public static Object mutex = new Object();


    public enum RingCount {
        ONE, NONE, FOUR, UNKNOWN
    }

    public enum StartingPosition {
        LEFT, RIGHT
    }
    public static final double ROBOT_STARTING_X =
            -(Field.FIELD_WIDTH/2 - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 - 2*Field.MM_PER_INCH);
    public static final Pose2d STARTING_POSE = new Pose2d(
            ROBOT_STARTING_X/MM_PER_INCH,
            -(Field.TILE_WIDTH*2 + Field.TAPE_WIDTH - Robot.WIDTH/2)/MM_PER_INCH,
            0);

    public static final Pose2d POWER_SHOT_SHOOTING_POSE_RIGHT = new Pose2d(
            -4,
            -24
    );
    public static final Pose2d POWER_SHOT_SHOOTING_POSE_CENTER = new Pose2d(
            POWER_SHOT_SHOOTING_POSE_RIGHT.getX(),
            POWER_SHOT_SHOOTING_POSE_RIGHT.getY() + Field.DISTANCE_BETWEEN_POWER_SHOTS/Field.MM_PER_INCH
    );
    public static final Pose2d POWER_SHOT_SHOOTING_POSE_LEFT = new Pose2d(
            POWER_SHOT_SHOOTING_POSE_CENTER.getX(),
            POWER_SHOT_SHOOTING_POSE_CENTER.getY() + Field.DISTANCE_BETWEEN_POWER_SHOTS/Field.MM_PER_INCH
    );
    public static final Pose2d TOWER_SHOOTING_POSE = new Pose2d(
            -3.5,
            -34.5
    );
    public static final Pose2d DROP_ZONE_POSE = new Pose2d(
            -60,
            -9,
            Math.toRadians(180)
    );
    public static final Pose2d NAVIGATION_POSE_NOT_FOUR_RINGS = new Pose2d(6, -24);
    public static final Pose2d NAVIGATION_POSE_FOUR_RINGS = new Pose2d(6, -48, Math.toRadians(180));

    public static final Pose2d NONE_DEPOSIT_POSE_FIRST = new Pose2d(-2, -62);
    //the one ring deposit position is one tile width to the left and forward from the audience from the none deposit position
    public static final Pose2d ONE_DEPOSIT_POSE_FIRST = new Pose2d(NONE_DEPOSIT_POSE_FIRST.getX() + Field.TILE_WIDTH/Field.MM_PER_INCH,
            NONE_DEPOSIT_POSE_FIRST.getY() + Field.TILE_WIDTH/Field.MM_PER_INCH);
    //the four ring deposit position is two tile width forward from the audience from the none deposit position
    public static final Pose2d FOUR_DEPOSIT_POSE_FIRST = new Pose2d(NONE_DEPOSIT_POSE_FIRST.getX() + 2*Field.TILE_WIDTH/Field.MM_PER_INCH,
            NONE_DEPOSIT_POSE_FIRST.getY());

    public static final Pose2d NONE_DEPOSIT_POSE_SECOND =
            new Pose2d(8, -40, Math.toRadians(-90));
    public static final Pose2d ONE_DEPOSIT_POSE_SECOND =
            new Pose2d(NONE_DEPOSIT_POSE_SECOND.getX()+Field.TILE_WIDTH/Field.MM_PER_INCH,
                    NONE_DEPOSIT_POSE_SECOND.getY()+Field.TILE_WIDTH/Field.MM_PER_INCH, ONE_DEPOSIT_POSE_FIRST.getHeading());
    public static final Pose2d FOUR_DEPOSIT_POSE_SECOND =
            new Pose2d(NONE_DEPOSIT_POSE_SECOND.getX()+2*Field.TILE_WIDTH/Field.MM_PER_INCH, NONE_DEPOSIT_POSE_SECOND.getY(),
                    NONE_DEPOSIT_POSE_SECOND.getHeading());

    public static final Pose2d SECOND_WOBBLE_PICK_UP = new Pose2d(
            -51,
            -7.5,//-(Field.TILE_WIDTH - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 - PickerArm.HOVER_EXTENSION/PickerArm.EXTENSION_ENCODER_COUNT_PER_MM + 4)/MM_PER_INCH,
            Math.toRadians(-90));

    Trajectory[] dropFirstWobbleTrajectories = new Trajectory[RingCount.values().length];
    Trajectory reachPowerShotShootingSpotTrajectory;
    Trajectory reachHighGoalShootingSpotTrajectory;
    Trajectory secondWobbleFromPowerShotTrajectory = null;
    Trajectory secondWobbleFromRedHighGoalTrajectory;
    Trajectory dropSecondWobbleTrajectory;
    Trajectory navigationTrajectory;
    Trajectory angularPowerShotTrajectory;
    Trajectory secondWobbleFromAngularPowerShotTrajectory;

    public void init() {
        Thread initThread = new Thread(() -> {
            RobotLog.i("SilverTitans: Field initialization started");

            //Red-right first wobble trajectories
            dropFirstWobbleTrajectories[RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(
                            STARTING_POSE,
                            PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(NONE_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            dropFirstWobbleTrajectories[RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(
                            STARTING_POSE, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(ONE_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            dropFirstWobbleTrajectories[RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(
                             STARTING_POSE, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToConstantHeading(new Vector2d(-24, NONE_DEPOSIT_POSE_FIRST.getY()), 0)
                            .splineToConstantHeading(FOUR_DEPOSIT_POSE_FIRST.vec(), 0)
                            .build();
            RobotLog.i("SilverTitans: Created drop first wobble trajectories");
            synchronized (mutex) {
                initialized = true;
                RobotLog.i("SilverTitans: Field initialization completed");
            }
        });
        initThread.start();
    }

    public static boolean isInitialized() {
        synchronized (mutex) {
            return initialized;
        }
    }

    public void generateTrajectories(RingCount ringCount, boolean powershots) {
        Thread initThread = new Thread(() -> {
            if (ringCount == RingCount.NONE) {
                if (powershots) {
                    reachPowerShotShootingSpotTrajectory =
                            new TrajectoryBuilder(NONE_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    //.strafeLeft(24) //avoid hitting rings if we did not see the right number of them
                                    .splineToLinearHeading(POWER_SHOT_SHOOTING_POSE_RIGHT, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created reach power shot shooting trajectory");
                }
                else {
                    reachHighGoalShootingSpotTrajectory =
                            new TrajectoryBuilder(NONE_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    //.strafeLeft(4) //avoid hitting rings if we did not see the right number of them
                                    .splineToLinearHeading(TOWER_SHOOTING_POSE, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created high goal shooting trajectory");
                }
                Thread.yield();
                generateReachSecondWobbleGoalTrajectories(powershots);
                dropSecondWobbleTrajectory =
                        new TrajectoryBuilder(SECOND_WOBBLE_PICK_UP, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .strafeLeft(24)//avoid rings
                                .splineToConstantHeading(NONE_DEPOSIT_POSE_SECOND.vec(), NONE_DEPOSIT_POSE_SECOND.getHeading())
                                .build();
                RobotLog.i("SilverTitans: Created drop second wobble trajectory");
                Thread.yield();
                navigationTrajectory =
                        new TrajectoryBuilder(NONE_DEPOSIT_POSE_SECOND, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .splineToLinearHeading(NAVIGATION_POSE_NOT_FOUR_RINGS, 0)
                                .build();
                RobotLog.i("SilverTitans: Created navigation trajectory");
                Thread.yield();
            }
            else if (ringCount == RingCount.ONE) {
                if (powershots) {
                    reachPowerShotShootingSpotTrajectory =
                            new TrajectoryBuilder(ONE_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    .splineToLinearHeading(POWER_SHOT_SHOOTING_POSE_RIGHT, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created reach power shot shooting trajectory");
                }
                else {
                    reachHighGoalShootingSpotTrajectory =
                            new TrajectoryBuilder(ONE_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    .splineToLinearHeading(TOWER_SHOOTING_POSE, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created reach high goal shooting trajectory");
                }
                Thread.yield();
                generateReachSecondWobbleGoalTrajectories(powershots);
                dropSecondWobbleTrajectory =
                        new TrajectoryBuilder(new Pose2d(SECOND_WOBBLE_PICK_UP.vec(), Math.toRadians(AutonomousHelper.SECOND_WOBBLE_PICKUP_HEADING)), PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .strafeLeft(24)//avoid rings
                                .splineToConstantHeading(ONE_DEPOSIT_POSE_SECOND.vec(), NONE_DEPOSIT_POSE_SECOND.getHeading())
                                .build();
                RobotLog.i("SilverTitans: Created drop second wobble trajectory");
                Thread.yield();
                navigationTrajectory =
                        new TrajectoryBuilder(ONE_DEPOSIT_POSE_SECOND, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .splineToLinearHeading(NAVIGATION_POSE_NOT_FOUR_RINGS, 0)
                                .build();
                RobotLog.i("SilverTitans: Created navigation trajectory");
                Thread.yield();
            }
            else {
                if (powershots) {
                    reachPowerShotShootingSpotTrajectory =
                            new TrajectoryBuilder(FOUR_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    //.strafeLeft(24)     //avoid hitting rings if we did not see the right number of them
                                    .splineToLinearHeading(POWER_SHOT_SHOOTING_POSE_RIGHT, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created reach power shot shooting trajectory");
                } else {
                    reachHighGoalShootingSpotTrajectory =
                            new TrajectoryBuilder(FOUR_DEPOSIT_POSE_FIRST, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                    .strafeLeft(4)     //avoid hitting rings if we did not see the right number of them
                                    .splineToLinearHeading(TOWER_SHOOTING_POSE, 0)
                                    .build();
                    RobotLog.i("SilverTitans: Created reach high goal shooting trajectory");
                }
                Thread.yield();
                generateReachSecondWobbleGoalTrajectories(powershots);
                dropSecondWobbleTrajectory =
                        new TrajectoryBuilder(SECOND_WOBBLE_PICK_UP, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .strafeLeft(24)//avoid rings
                                .splineToConstantHeading(FOUR_DEPOSIT_POSE_SECOND.vec(), NONE_DEPOSIT_POSE_SECOND.getHeading())
                                .build();
                RobotLog.i("SilverTitans: Created drop second wobble trajectory");
                Thread.yield();
                navigationTrajectory =
                        new TrajectoryBuilder(FOUR_DEPOSIT_POSE_SECOND, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                                .splineToLinearHeading(NAVIGATION_POSE_FOUR_RINGS, 0)
                                .build();
                RobotLog.i("SilverTitans: Created navigation trajectory");
                Thread.yield();
            }
        });
        initThread.start();
    }

    private void generateReachSecondWobbleGoalTrajectories(boolean powershots) {
        //Red-right reach second wobble from left power shot trajectories
        if (powershots) {
            secondWobbleFromPowerShotTrajectory =
                    new TrajectoryBuilder(POWER_SHOT_SHOOTING_POSE_LEFT, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            .splineToLinearHeading(new Pose2d(SECOND_WOBBLE_PICK_UP.getX(), SECOND_WOBBLE_PICK_UP.getY(), 0), 0)
                            .build();
            RobotLog.i("SilverTitans: Created second wobble from power shot trajectory");
        }
        else {
            secondWobbleFromRedHighGoalTrajectory =
                    new TrajectoryBuilder(TOWER_SHOOTING_POSE, true, PhoebeRoadRunnerDrive.fastVelocityConstraint, PhoebeRoadRunnerDrive.accelerationConstraint)
                            //.strafeLeft(24)
                            .splineToLinearHeading(new Pose2d(SECOND_WOBBLE_PICK_UP.getX(), SECOND_WOBBLE_PICK_UP.getY(), 0), 0)
                            .build();
            RobotLog.i("SilverTitans: Created reach second wobble from high goal trajectory");
        }
        Thread.yield();
    }

    public static com.arcrobotics.ftclib.geometry.Pose2d roadRunnerToCameraPose(Pose2d pose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX()*M_PER_INCH, pose.getY()*M_PER_INCH,
                new Rotation2d(pose.getHeading()+Math.PI));
    }
    public static Pose2d cameraToRoadRunnerPose(com.arcrobotics.ftclib.geometry.Pose2d pose) {
        return new Pose2d(pose.getTranslation().getX()/M_PER_INCH, pose.getTranslation().getY()/M_PER_INCH,
                pose.getHeading()+Math.PI);
    }

    public Trajectory getFirstWobbleGoalDepositTrajectory() {
        Match match = Match.getInstance();
        int rings = match.getNumberOfRings().ordinal();
        return dropFirstWobbleTrajectories[rings];
    }
    public Trajectory getReachPowerShotShootingPose() {
        return reachPowerShotShootingSpotTrajectory;
    }
    public Trajectory getReachHighGoalShootingPose() {
        return reachHighGoalShootingSpotTrajectory;
    }
    public Trajectory getReachSecondWobbleFromPowerShotLeftTrajectory() {
        return secondWobbleFromPowerShotTrajectory;
    }
    public Trajectory getReachSecondWobbleFromHighGoalTrajectory() {
        return secondWobbleFromRedHighGoalTrajectory;
    }
    public Trajectory getSecondWobbleGoalDepositTrajectory() {
        return dropSecondWobbleTrajectory;
    }
    public Trajectory getNavigationTrajectory() {
        return navigationTrajectory;
    }
}