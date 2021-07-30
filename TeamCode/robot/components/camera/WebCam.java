package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;


/**
 * Created by silver titans on 9/19/17.
 */

public class WebCam {

    private volatile boolean isInitialized;

    // IMPORTANT:  For Phone WebCam, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) WebCam Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    public static final float CAMERA_FORWARD_DISPLACEMENT  = 7.25f * Field.MM_PER_INCH;   //WebCam is 7 1/4 Inches in front of robot center
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 4.375f * Field.MM_PER_INCH;   //WebCam is 4 3/8 Inches above ground
    public static final float CAMERA_LEFT_DISPLACEMENT     = 0;     //WebCam is centered left to right

    private VuforiaLocalizer vuforia = null;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpenGLMatrix vuforiaCameraFromRobot;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    VuforiaTrackables targetsUltimateGoal;
    ArrayList<VuforiaTrackable> allTrackables = new ArrayList<>();

    private Servo cameraServo;

    // Constants for perimeter targets
    private static final float halfField = 72f * Field.MM_PER_INCH;
    private static final float quadField  = 36f * Field.MM_PER_INCH;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmTargetHeight   = (float) ((5.75) * Field.MM_PER_INCH);          // the height of the center of the target image above the floor

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    DcMotor ledControl;

    private RingDetector ringDetector;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Alliance.Color allianceColor) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        ledControl = hardwareMap.get(DcMotor.class, "LED");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(0.51);
        this.ringDetector = new RingDetector();
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        Thread vuforiaInitializationThread = new Thread(new Runnable() {
            @Override
            public void run() {
                synchronized (ringDetector) {

                    /*
                     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
                     * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
                     * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
                     */
                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                    parameters.vuforiaLicenseKey = Match.VUFORIA_KEY;
                    parameters.cameraDirection = BACK;
                    parameters.useExtendedTracking = false;
                    /**
                     * We also indicate which camera on the RC we wish to use.
                     */
                    parameters.cameraName = webcamName;

                    //  Instantiate the Vuforia engine
                    vuforia = ClassFactory.getInstance().createVuforia(parameters);
                    vuforia.enableConvertFrameToBitmap();
                    vuforia.setFrameQueueCapacity(1);

                    // Load the data sets for the trackable objects. These particular data
                    // sets are stored in the 'assets' part of our application.
                    targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
                    VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
                    blueTowerGoalTarget.setName("Blue Tower Goal Target");
                    VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
                    redTowerGoalTarget.setName("Red Tower Goal Target");
                    VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
                    redAllianceTarget.setName("Red Alliance Target");
                    VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
                    blueAllianceTarget.setName("Blue Alliance Target");
                    VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
                    frontWallTarget.setName("Front Wall Target");

                    // For convenience, gather together all the trackable objects in one easily-iterable collection */
                    allTrackables.addAll(targetsUltimateGoal);


                    /**
                     * In order for localization to work, we need to tell the system where each target is on the field, and
                     * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
                     * Transformation matrices are a central, important concept in the math here involved in localization.
                     * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
                     * for detailed information. Commonly, you'll encounter transformation matrices as instances
                     * of the {@link OpenGLMatrix} class.
                     *
                     * If you are standing in the Red Alliance Station looking towards the center of the field,
                     *     - The X axis runs from your left to the right. (positive from the center to the right)
                     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
                     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
                     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
                     *
                     * Before being transformed, each target image is conceptually located at the origin of the field's
                     *  coordinate system (the center of the field), facing up.
                     */

                    //Set the position of the perimeter targets with relation to origin (center of field)
                    redAllianceTarget.setLocation(OpenGLMatrix
                            .translation(0, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

                    blueAllianceTarget.setLocation(OpenGLMatrix
                            .translation(0, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
                    frontWallTarget.setLocation(OpenGLMatrix
                            .translation(-halfField, 0, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

                    // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
                    blueTowerGoalTarget.setLocation(OpenGLMatrix
                            .translation(halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
                    redTowerGoalTarget.setLocation(OpenGLMatrix
                            .translation(halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


                    vuforiaCameraFromRobot = OpenGLMatrix
                            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, ZYX, DEGREES, -90, 90, 0)).inverted();

                    /**  Let all the trackable listeners know where the phone is.  */
                    for (VuforiaTrackable trackable : allTrackables) {
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(vuforiaCameraFromRobot, parameters.cameraDirection);
                    }
                    targetsUltimateGoal.activate();
                    isInitialized = true;
                }
            }
        });
        vuforiaInitializationThread.start();
    }

    /**
     * Tries to find one of the vuMarks on the field.
     *
     * @return name of the vumark found, null if none could be found
     */
    public String findTarget() {
        OpenGLMatrix vuforiaCameraFromTarget = null;
        VuforiaTrackable visibleTarget = null;
        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable: allTrackables){
            vuforiaCameraFromTarget = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();
            if (vuforiaCameraFromTarget != null){
                visibleTarget = trackable;
                break;
            }
        }
        if (vuforiaCameraFromTarget != null) {
            OpenGLMatrix targetFromVuforiaCamera = vuforiaCameraFromTarget.inverted();
            lastLocation = visibleTarget.getFtcFieldFromTarget().multiplied(targetFromVuforiaCamera).multiplied(vuforiaCameraFromRobot);
            VectorF translation = lastLocation.getTranslation();
            //Match.log(String.format("Pos (in), {X, Y, Z} = %.1f, %.1f, %.1f",
                    //translation.get(0) / Field.MM_PER_INCH, translation.get(1) / Field.MM_PER_INCH, translation.get(2) / Field.MM_PER_INCH));
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            //Match.log(String.format("Rot (deg), {Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle));
        }
        telemetry.update();
        return visibleTarget != null ? visibleTarget.getName() : null;
    }

    public float getCurrentX() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(0);
        } else {
            return -1;
        }
    }

    public float getCurrentY() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(1);
        } else {
            return -1;
        }
    }

    public float getCurrentZ() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(2);
        } else {
            return -1;
        }
    }

    public float getCurrentTheta() {
        if (lastLocation != null) {
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return
                    rotation.thirdAngle;
        } else {
            return -1;
        }
    }

    public String getPosition() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return String.format("{X, Y, Z} = %.2f, %.2f, %.2f",
                    translation.get(0) / Field.MM_PER_INCH,
                    translation.get(1) / Field.MM_PER_INCH,
                    translation.get(2) / Field.MM_PER_INCH);
        } else {
            return "Target not found";
        }
    }

    public void turnLedOn() {
        ledControl.setPower(1);
    }
    public void turnLedOff() {
        ledControl.setPower(0);
    }

    public void handleOperation(CameraOperation operation) {
        if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_ON) {
            Match.log("Setting led on");
            ledControl.setPower(1);
        }
        else if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_OFF) {
            Match.log("Setting led off");
            ledControl.setPower(0);
        }
        else {
            Match.log("Unknown operation: " + operation.getCameraOperationType());
        }
    }


    /**
     * attempt to find sky stone location
     *
     * @return The position of the sky stone in the quarry
     */
    public int getNumberOfRings() {
        synchronized (ringDetector) {
            /*To access the image: we need to iterate through the images of the frame object:*/
            VuforiaLocalizer.CloseableFrame vuforiaFrame; //takes the frame at the head of the queue
            try {
                //Match.log("Getting frame from vuForia");
                vuforiaFrame = vuforia.getFrameQueue().take();
                //Match.log("Got frame from vuForia");
                Image image = null;
                long numImages = vuforiaFrame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    Image checkedImage = vuforiaFrame.getImage(i);
                    int format = checkedImage.getFormat();
                    if (format == PIXEL_FORMAT.RGB565) {
                        image = checkedImage;
                        break;
                    }//if
                }//for
                if (image == null) {
                    Match.log("Unable to get image from vuForia camera out of " + numImages);
                    return 0;
                }

                Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                //Match.log(image.getWidth() + ": " + image.getHeight());
                bitmap.copyPixelsFromBuffer(image.getPixels());
                return ringDetector.getNumberOfRings(bitmap);
            } catch (Exception e) {
                Match.log("Exception " + e + " in finding number of rings");
            }
            //default to 0
            return 0;
        }
    }

    public boolean isInitialized() {
            return isInitialized;
    }
}
