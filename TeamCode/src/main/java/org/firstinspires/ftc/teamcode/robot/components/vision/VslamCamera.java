package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;

public class VslamCamera implements Localizer, Consumer<T265Camera.CameraUpdate> {
    //Remember all measurements for ftclib geometry are in meters

    //How far camera is in front from the center of the robot
    public static final double T265_OFFSET_FRONT =
            -MecanumDriveTrain.DRIVE_TRAIN_LENGTH / 2 / 1000;
    //How far camera is to the left of center
    public static final double T265_CAMERA_OFFSET_LEFT = -1 * Field.MM_PER_INCH / 1000;
    public static final double T265_ROTATION = 180;

    private static T265Camera t265Camera = null;
    private Pose2d lastPose = new Pose2d();
    private T265Camera.CameraUpdate cameraUpdate;
    private Pose2d pose2dVelocity = new Pose2d();
    private double lastUpdateTime = 0;
    private NanoClock clock = NanoClock.system();
    private final Transform2d t265LocationOnRobot =
            new Transform2d(new Translation2d(T265_OFFSET_FRONT, T265_CAMERA_OFFSET_LEFT),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));
    private static final Object synchronizationObject = new Object();
    private volatile boolean isInitialized = false;
    private Pose2d startingPose;

    public String getLastError() {
        return lastError;
    }

    public void setLastError(String lastError) {
        this.lastError = lastError;
    }

    private String lastError = "";

    public VslamCamera(HardwareMap hardwareMap) {
        Thread cameraInitializationThread = new Thread(() -> {
            synchronized (synchronizationObject) {
                if (t265Camera == null) {
                    Match.log("Creating new T265 camera");
                    t265Camera = new T265Camera(t265LocationOnRobot, 0.95, hardwareMap.appContext);
                    t265Camera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d());
                }
                isInitialized = true;
            }
        });
        cameraInitializationThread.start();
    }

    /**
     * Set robot starting position
     *
     */
    public void setStartingPose() {
        this.startingPose = Field.STARTING_POSE;
        t265Camera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d());
    }


    public void setStartingPose(Pose2d pose) {
        this.startingPose = pose;
    }

    public boolean isInitialized() {
        synchronized (synchronizationObject) {
            return this.isInitialized;
        }
    }

    //Methods to implement roadrunner localizer
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        update();
        return lastPose;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        lastPose = pose2d;
        update();
    }

    @Override
    public Pose2d getPoseVelocity() {
        return pose2dVelocity;
    }

    @Override
    public void update() {
        //find time elapsed
        double currentTime = clock.seconds();
        double timeElapsed = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        T265Camera.CameraUpdate cameraUpdate = getLastCameraUpdate();
        //get latest received update
        if (cameraUpdate == null) {
            return;
        }
        try {
            //find current position in the coordinates and units of measure that roadrunner understands
            double currentX = -cameraUpdate.pose.getTranslation().getX() / Field.M_PER_INCH + startingPose.getX();
            double currentY = -cameraUpdate.pose.getTranslation().getY() / Field.M_PER_INCH + startingPose.getY();
            double currentTheta = cameraUpdate.pose.getHeading() + Math.PI;

            pose2dVelocity = new Pose2d(
                    (currentX - lastPose.getX()) / timeElapsed,
                    (currentY - lastPose.getY()) / timeElapsed,
                    (currentTheta - lastPose.getHeading()) / timeElapsed);
            //set last pose to be the current one
            lastPose = new Pose2d(
                    currentX,
                    currentY,
                    currentTheta);
        } catch (Throwable e) {
            RobotLog.ee("Vslam", e, "Error getting position");
        }
    }

    private T265Camera.CameraUpdate getLastCameraUpdate() {
        synchronized (synchronizationObject) {
            return cameraUpdate;
        }
    }

    public void stop() {
        //Match.log("Stopping T265 camera");
        t265Camera.stop();
    }
    public void start() {
        try {//start our camera
            //Match.log("Starting T265 camera");
            t265Camera.start(this);
        } catch (Throwable e) {
            if (!e.getMessage().equals("Camera is already started")) {
                RobotLog.logStackTrace("SilverTitans: Error starting real sense", e);
                throw e;
            }
        }
    }


    @Override
    public void accept(T265Camera.CameraUpdate cameraUpdate) {
        synchronized (synchronizationObject) {
            this.cameraUpdate = cameraUpdate;
        }
    }

    public boolean havePosition() {
        return this.cameraUpdate != null;
    }
}
