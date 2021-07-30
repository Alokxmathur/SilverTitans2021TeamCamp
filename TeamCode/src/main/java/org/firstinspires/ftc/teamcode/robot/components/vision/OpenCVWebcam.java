/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class OpenCVWebcam {

    public static final double CAMERA_OFFSET_FRONT = 6.5;
    public static final double CAMERA_SERVO_FORWARD_POSITION = 0.51;
    public static final int FOCAL_LENGTH = 1500;

    public static final Scalar RING_COLOR_MIN = new Scalar(12, 50, 50);
    public static final Scalar RING_COLOR_MAX = new Scalar(22, 255, 255);

    public static final Scalar WOBBLE_COLOR_MIN = new Scalar(0, 70, 50);
    public static final Scalar WOBBLE_COLOR_MAX = new Scalar(10, 255, 255);
    public static final int Y_PIXEL_COUNT = 1920;
    public static final int X_PIXEL_COUNT = 1080;

    OpenCvCamera webcam;
    Servo cameraServo;
    Pipeline pipeline;
    Scalar colorMin;
    Scalar colorMax;

    double distanceToRing, angleToRing;

    public Scalar getColorMin() {
        return this.colorMin;
    }

    public void setColorMin(Scalar colorMin) {
        this.colorMin = colorMin;
    }

    public Scalar getColorMax() {
        return colorMax;
    }

    public void setColorMax(Scalar colorMax) {
        this.colorMax = colorMax;
    }

    public static final Object synchronizer = new Object();

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Scalar colorMin, Scalar colorMax) {
        this.colorMin = colorMin;
        this.colorMax = colorMax;
        pipeline = new Pipeline();

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(OpenCVWebcam.CAMERA_SERVO_FORWARD_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline.setTelemetry(telemetry);
        webcam.setPipeline(pipeline);
        start();
    }

    public void start() {
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(Y_PIXEL_COUNT, X_PIXEL_COUNT, OpenCvCameraRotation.SIDEWAYS_LEFT));
    }

    public Field.RingCount getNumberOfRings() {
        synchronized (synchronizer) {
            return pipeline.ringCount;
        }
    }

    /**
     * Return the road runner pose of the ring seen relative to the robot.
     * The robot is assumed to be facing the y axis with positive being in front of the robot.
     * Positive X axis is assumed to be running away to robot's right
     * @return
     */
    public Pose2d getRelativeRingPosition() {
        if (seeingRing()) {
            return new Pose2d(pipeline.objectDetector.getXPosition(),
                    pipeline.objectDetector.getYPosition(), pipeline.objectDetector.getAngleFromCamera());
        }
        return null;
    }

    /**
     * Return the road runner pose of the ring seen relative to the field.
     *
     * @return
     */
    public Pose2d getAbsoluteRingPosition(Pose2d robotPosition) {

        Pose2d relativeRingPosition = getRelativeRingPosition();
        if (relativeRingPosition != null) {
            double heading = AngleUnit.normalizeRadians
                    (robotPosition.getHeading() + Math.toRadians(270) + relativeRingPosition.getHeading());
            double hypotenuse = Math.hypot(relativeRingPosition.getX(), relativeRingPosition.getY());
            double x = Math.cos(heading) * hypotenuse;
            double y = Math.sin(heading) * hypotenuse;
            Vector2d ringAbsoluteVector = robotPosition.vec().plus(new Vector2d(x, y));
            return new Pose2d(ringAbsoluteVector, heading);
        }
        return null;
    }

    public void decrementMinX() {
        pipeline.objectDetector.decrementMinAllowedX();
    }

    public void incrementMinX() {
        pipeline.objectDetector.incrementMinAllowedX();
    }
    public void decrementMaxX() {
        pipeline.objectDetector.decrementMaxAllowedX();
    }
    public void incrementMaxX() {
        pipeline.objectDetector.incrementMaxAllowedX();
    }
    public void decrementMinY() {
        pipeline.objectDetector.decrementMinAllowedY();
    }
    public void incrementMinY() {
        pipeline.objectDetector.incrementMinAllowedY();
    }
    public void decrementMaxY() {
        pipeline.objectDetector.decrementMaxAllowedY();
    }
    public void incrementMaxY() {
        pipeline.objectDetector.incrementMaxAllowedY();
    }
    public void setMinX(int minX) {
        pipeline.objectDetector.setMinAllowedX(minX);
    }
    public void setMaxX(int maxX) {
        pipeline.objectDetector.setMaxAllowedX(maxX);
    }
    public void setMinY(int minY) {
        pipeline.objectDetector.setMinAllowedY(minY);
    }
    public void setMaxY(int maxY) {
        pipeline.objectDetector.setMaxAllowedY(maxY);
    }
    public String getBounds() {
        return (pipeline.objectDetector.getBounds());
    }

    public int getMinX() {
        return pipeline.objectDetector.minX;
    }
    public int getMaxX() {
        return pipeline.objectDetector.maxX;
    }
    public int getMinY() {
        return pipeline.objectDetector.minY;
    }
    public int getMaxY() {
        return pipeline.objectDetector.maxY;
    }

    public double getXPosition() {
        return pipeline.objectDetector.getXPosition();
    }
    public double getYPosition() {
        return pipeline.objectDetector.getYPosition();
    }

    /**
     * Return the distance to object in inches
     * @return
     */
    public double getDistanceToObjectFromCamera() { return pipeline.objectDetector.distanceToObjectFromCamera;}
    public double getDistanceToObjectFromCenter() { return pipeline.objectDetector.distanceFromCenterOfRobot();}
    public double getAngleToObjectFromCamera() { return pipeline.objectDetector.angleToObjectFromCamera;}
    public double getAngleToObjectFromCenter() { return pipeline.objectDetector.angleFromCenterOfRobot();}
    public Scalar getMean() {
        return pipeline.objectDetector.getMean();
    }

    public boolean seeingRing() {
        return getNumberOfRings() == Field.RingCount.FOUR || getNumberOfRings() == Field.RingCount.ONE;
    }
    public Object getWidth() {
        return getMaxY() - getMinY();
    }

    public Object getHeight() {
        return getMaxX() - getMinX();
    }
    public class Pipeline extends OpenCvPipeline {
        Telemetry telemetry;
        public void setTelemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * Red, Blue and Green color constants
         */
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar BLUE = new Scalar(0, 0, 255);

        public ObjectDetector objectDetector = new ObjectDetector(colorMin, colorMax,
                0, 480, 0, 1700);
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Field.RingCount ringCount = Field.RingCount.UNKNOWN;
        private volatile double ringRatio;

        @Override
        public Mat processFrame(Mat input) {
            synchronized (synchronizer) {
                MatOfPoint contour = objectDetector.process(input);
                if (contour != null) {
                    ArrayList<MatOfPoint> contours = new ArrayList<>();
                    contours.add(contour);
                    Imgproc.drawContours(input, contours, -1, GREEN, 5);
                }
                Rect limitsRectangle = objectDetector.getAreaOfInterest();
                Imgproc.rectangle(input, limitsRectangle, RED, 6);

                synchronized (synchronizer) {
                    if (objectDetector.getLargestArea() > 0) {
                        ringRatio = objectDetector.getHeightWidthRatio();
                        if (ringRatio > ObjectDetector.FOUR_RING_RATIO_THRESHOLD) {
                            ringCount = Field.RingCount.FOUR;
                        } else  {
                            ringCount = Field.RingCount.ONE;
                        }
                    }
                    else {
                        ringCount = Field.RingCount.NONE;
                    }
                }
                return input;
            }
        }
    }
}