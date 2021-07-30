package org.firstinspires.ftc.teamcode.robot.components.vision;

import android.util.Log;

import org.firstinspires.ftc.teamcode.game.Field;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetector {

    public static final double FOUR_RING_RATIO_THRESHOLD = .4f;

    private static final String TAG = "ObjectDetector";
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    private Mat mSpectrum = new Mat();
    private MatOfPoint largestContour = null;

    double maxArea = 0;
    int minY = 9999, minX = 9999;
    int maxY = 0, maxX = 0;
    double distanceToObjectFromCamera, angleToObjectFromCamera;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public Scalar getMean() {
        return mean;
    }

    Scalar mean;
    Rect areaOfInterest;

    int minAllowedX;
    int maxAllowedX;
    int minAllowedY;

    public void setMinAllowedX(int minAllowedX) {
        this.minAllowedX = minAllowedX;
    }

    public void setMaxAllowedX(int maxAllowedX) {
        this.maxAllowedX = maxAllowedX;
    }

    public void setMinAllowedY(int minAllowedY) {
        this.minAllowedY = minAllowedY;
    }

    public void setMaxAllowedY(int maxAllowedY) {
        this.maxAllowedY = maxAllowedY;
    }

    int maxAllowedY;

    public ObjectDetector(Scalar minColor, Scalar maxColor,
                          int minAllowedX, int maxAllowedX, int minAllowedY, int maxAllowedY) {
        this.setHsvColorRange(minColor, maxColor);
        this.minAllowedX = minAllowedX;
        this.maxAllowedX = maxAllowedX;
        this.minAllowedY = minAllowedY;
        this.maxAllowedY = maxAllowedY;
        setupAreaOfInterest();
    }

    private void setupAreaOfInterest() {
        areaOfInterest = new Rect(minAllowedX, minAllowedY, maxAllowedX-minAllowedX, maxAllowedY-minAllowedY);
    }

    public void setHsvColorRange(Scalar minHsvColor, Scalar maxHsvColor) {
        mLowerBound.val[0] = minHsvColor.val[0];
        mUpperBound.val[0] = maxHsvColor.val[0];

        mLowerBound.val[1] = minHsvColor.val[1];
        mUpperBound.val[1] = maxHsvColor.val[1];

        mLowerBound.val[2] = minHsvColor.val[2];
        mUpperBound.val[2] = maxHsvColor.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
        Log.e(TAG, getBounds());

        Mat spectrumHsv = new Mat(1, (int)(maxHsvColor.val[0]-minHsvColor.val[0]), CvType.CV_8UC3);

        for (int j = 0; j < maxHsvColor.val[0]-minHsvColor.val[0]; j++) {
            byte[] tmp = {(byte)(maxHsvColor.val[0]+j), (byte)255, (byte)255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public String getBounds() {
        return String.format("%.0f-%.0f,%.0f-%.0f,%.0f-%.0f, x:%d-%d, y:%d-%d",
                mLowerBound.val[0], mUpperBound.val[0], mLowerBound.val[1],
                mUpperBound.val[1], mLowerBound.val[2], mUpperBound.val[2],
                minAllowedX, maxAllowedX, minAllowedY, maxAllowedY);
    }

    public Rect getAreaOfInterest() {
        return areaOfInterest;
    }

    public MatOfPoint process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contoursFound = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mDilatedMask, contoursFound, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        minX = minY = 9999;
        maxX = maxY = 0;
        maxArea = 0;
        for (MatOfPoint contour: contoursFound) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                Rect boundingRectangle = Imgproc.boundingRect(contour);
                if (boundingRectangle.x*4 <= maxAllowedX && boundingRectangle.x*4 >= minAllowedX
                    && boundingRectangle.y*4 <= maxAllowedY && boundingRectangle.y*4 >=minAllowedY ) {
                    maxArea = area;
                    Core.multiply(contour, new Scalar(4,4), contour);
                    largestContour = contour;
                    minX = boundingRectangle.x*4;
                    maxX = boundingRectangle.x*4 + boundingRectangle.width*4;
                    minY = boundingRectangle.y*4;
                    maxY = boundingRectangle.y*4 + boundingRectangle.height*4;
                    updateObjectPosition();
                }
                mean = Core.mean(contour);
            }
        }
        return largestContour;
    }
    void updateObjectPosition() {
        distanceToObjectFromCamera = getDistanceFromCamera();
        angleToObjectFromCamera = getAngleFromCamera();
    }

    public double getLargestArea() {
        return maxArea;
    }

    public double getWidthInPixels() {
        if (getLargestArea() != 0) {
            return maxY - minY;
        }
        else {
            return 0;
        }
    }
    public double getHeightInPixels() {
        if (getLargestArea() != 0) {
            return maxX - minX;
        }
        else {
            return 0;
        }
    }

    public double getHeightWidthRatio() {
        if (maxArea > 0) {
            return (double) getHeightInPixels() / (double) getWidthInPixels();
        }
        return 0;
    }
    public void decrementMinAllowedX() {
        this.minAllowedX = Math.max(minAllowedX - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMinAllowedX() {
        this.minAllowedX = Math.min(minAllowedX + 1, OpenCVWebcam.X_PIXEL_COUNT -1);
        setupAreaOfInterest();
    }
    public void decrementMaxAllowedX() {
        this.maxAllowedX = Math.max(maxAllowedX - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMaxAllowedX() {
        this.maxAllowedX = Math.min(maxAllowedX + 1, OpenCVWebcam.X_PIXEL_COUNT);
        setupAreaOfInterest();
    }

    public void decrementMinAllowedY() {
        this.minAllowedY = Math.max(minAllowedY - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMinAllowedY() {
        this.minAllowedY = Math.min(minAllowedY + 1, OpenCVWebcam.Y_PIXEL_COUNT-1);
        setupAreaOfInterest();
    }
    public void decrementMaxAllowedY() {
        this.maxAllowedY = Math.max(maxAllowedY - 1, 0);
        setupAreaOfInterest();
    }
    public void incrementMaxAllowedY() {
        this.maxAllowedY = Math.min(maxAllowedY + 1, OpenCVWebcam.Y_PIXEL_COUNT);
        setupAreaOfInterest();
    }

    /** Return the distance to the object from the camera in inches
     *
     * @return
     */
    private double getDistanceFromCamera() {
        int pixelWidth = (maxY - minY);
        return Field.RING_WIDTH/Field.MM_PER_INCH * OpenCVWebcam.FOCAL_LENGTH / pixelWidth;
    }

    /** Returns the x position of the object in pixels
     *
     * @return
     */
    private double getXPixelPosition() {
        return (double)(maxY - minY)/2 + minY - OpenCVWebcam.Y_PIXEL_COUNT/2;
    }

    /** Returns the y position of the object seen in pixels
     * @return
     */
    public double getYPixelPosition() {
        return (double)(maxX - minX)/2 + minX;
    }

    /** Returns the x position of the object seen in inches
     * X and y coordinates are reversed from the point of view of the robot from the camera image
     *
     * Also camera's 0 is really at 1920/2
     *
     * @return
     */
    public double getXPosition() {
        return distanceToObjectFromCamera * Math.cos(angleToObjectFromCamera);
    }

    /** Returns the y position of the ring(s) seen in inches
     * X and y coordinates are reversed from the point of view of the robot from the camera image
     *
     * @return
     */
    public double getYPosition() {
        return distanceToObjectFromCamera * Math.sin(angleToObjectFromCamera);
    }

    /**
     * Return the angle of the object from the camera in radians
     * @return
     */
    public double getAngleFromCamera() {
        if (getHeightInPixels() != 0) {
            if (getXPixelPosition() != 0) {
                //x and y are reversed as we are in landscape mode
                ///y goes from left to right of robot, x from back to front
                return Math.atan2(getYPixelPosition()/OpenCVWebcam.X_PIXEL_COUNT,  getXPixelPosition()/OpenCVWebcam.Y_PIXEL_COUNT);
            }
            return Math.toRadians(90);
        }
        return 0;
    }
    /**
     * Returns distance in inches to the object from the center of the robot, given the camera offset from the center of the robot
     * @return
     */
    public double distanceFromCenterOfRobot() {
        if (angleToObjectFromCamera < Math.toRadians(90)) {
                return Math.sqrt(
                     Math.pow(OpenCVWebcam.CAMERA_OFFSET_FRONT, 2)
                            + Math.pow(distanceToObjectFromCamera, 2)
                            - 2*distanceToObjectFromCamera*OpenCVWebcam.CAMERA_OFFSET_FRONT*Math.cos(Math.toRadians(90) + angleToObjectFromCamera));
        }
        else {
            return Math.sqrt(
                    Math.pow(OpenCVWebcam.CAMERA_OFFSET_FRONT, 2)
                            + Math.pow(distanceToObjectFromCamera, 2)
                            - 2*distanceToObjectFromCamera*OpenCVWebcam.CAMERA_OFFSET_FRONT*Math.cos(Math.toRadians(270) - angleToObjectFromCamera));
        }
    }

    /**
     * Returns the angle in radians to the object from the center of the robot
     * @return
     */

    public double angleFromCenterOfRobot() {
        return Math.asin(distanceFromCenterOfRobot()*Math.sin(Math.toRadians(90) + angleToObjectFromCamera)/distanceToObjectFromCamera);
    }

}
