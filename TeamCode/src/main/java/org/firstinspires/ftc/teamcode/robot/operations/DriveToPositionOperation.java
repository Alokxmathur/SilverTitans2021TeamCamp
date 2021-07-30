package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.vision.VslamCamera;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveToPositionOperation extends FollowTrajectory {
    private Pose2d desiredPose;
    private boolean trajectoryBeingFollowed;


    public boolean isTrajectoryBeingFollowed() {
        return trajectoryBeingFollowed;
    }

    public void setTrajectoryBeingFollowed(boolean trajectoryBeingFollowed) {
        this.trajectoryBeingFollowed = trajectoryBeingFollowed;
    }

    public DriveToPositionOperation(Pose2d desiredPose, int correctionCount, String title) {
        this(desiredPose, title);
        this.correctionCount = correctionCount;
    }
    public DriveToPositionOperation(Pose2d desiredPose, String title) {
        super(title);
        this.desiredPose = desiredPose;
        this.type = TYPE.DRIVE_TO_POSITION;
        this.title = title;
    }

    public Pose2d getDesiredPose() {
        return this.desiredPose;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveTo: %s --%s",
                this.desiredPose.toString(),
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain, VslamCamera camera) {
        //wait for the trajectory to be created
        if (!trajectoryBeingFollowed) {
            return false;
        }
        return super.isComplete(driveTrain, camera);
    }
}

