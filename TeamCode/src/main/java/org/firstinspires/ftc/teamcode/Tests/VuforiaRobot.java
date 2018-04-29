package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by KyleSmith on 2017/10/15.
 */

public class VuforiaRobot {

    //Extra vars
    public static final String TAG = "Vuforia VuMark Compeition";

    public OpenGLMatrix lastLocation = null;

    //Load all pictograms for Vuforia to scan
    private VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

    //Vars to store the robots position relative to the vuMark
    private double tX;
    private double tY;
    private double tZ;

    //TODO: Not sure how this is supposed to work but I think it is to see which template it is looking for
    VuforiaTrackable relicTemplate = relicTrackables.get(0);

    private VuforiaLocalizer vuforia;

    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

    //Default Constructor
    public VuforiaRobot(){}

    //Init method

    /*
    1. Get Access to the camera and assign ID to the camera
    2.Ensure license key is vallid
    3.Set camera direction
    4.Activate the relic trackables
     */

    public void initVuforia(HardwareMap hardwareMap){

         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //TODO: Current setting is saving power due to the fact that camera view is not enabled!
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //Set this to true to see what it does
        parameters.useExtendedTracking = false;

        //License Key **** Obtained from Vuforia's Website ****
        parameters.vuforiaLicenseKey = "Afi7ZJb/////AAAAGSZPQI5OpkKRkWVA/oa7uKCJtmTcZUlDo1ZMlZfZ+aYKl+CTzQvPjhtoLE8n/cNTFSHvyUBnFpv9oPt/70QZlGQvgbnL/3TQ0x9UnTsNnz//HF8ctHrXTUfDNgAjYtxYFGvJMlNjfh8jBxFQKMQYJrLRFhlMcYEshS117KUjqy7Gwp3uCdF+eWdSsuPXinf0fj3DATNDYjdOB5ChcXh1r6DW3RkmIV7Q131gUFJBeVDKjxy0ZBmGBdWX+VhD/81xNpHZHbN8vVikjl7oOWD7HijxaOi8FFOKbNysVyNq0/UujoW6thNXXRo2fTb+1ZnpZ21Gfo2uAnDAQ5/xajmwNxEhCEW19O4DaVDlSRBq3vJi";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */

        //TODO: Change the direction of the camera according to which way the camera is facing
        //Back camera is usually beter for resolution, but varies due to configuration of robot
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */

        //
        relicTrackables.activate();

    }

    //Method for scanning pictogram allowing us to map button to this method
    public void scanPictogram(){

        //Checks if what the camera is seeing is the same as the vuMarks in the array
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){

            //Gets the vuMarks position relative to the robot thriugh the use of the OpenGL library
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                 settX(trans.get(0));
                 settY(trans.get(1));
                 settZ(trans.get(2));

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

            }


        }//End of control structure for checking

    }//End of scan pictogram method

    /* Getters and setters */

    public double gettX() {
        return tX;
    }

    public void settX(double tX) {
        this.tX = tX;
    }

    public double gettY() {
        return tY;
    }

    public void settY(double tY) {
        this.tY = tY;
    }

    public double gettZ() {
        return tZ;
    }

    public void settZ(double tZ) {
        this.tZ = tZ;
    }
}
