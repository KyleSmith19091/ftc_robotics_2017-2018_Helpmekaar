package org.firstinspires.ftc.teamcode.Competition.AutonomousPackage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
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
import org.firstinspires.ftc.teamcode.Competition.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Competition.OpenCVClasses.OpenCVJewelID;

/**
 * Created by KyleSmith on 3/3/18.
 */
@Autonomous(name = "BlueMat", group = "Autonomous")
public class BlueMat extends LinearOpMode {


    /* Declare OpMode members. */
    Robot robot   = new Robot();   // Custom Hardware class hardware
    OpenCVJewelID bluevision = new OpenCVJewelID(); //Jewel Identification using OpenCV

    //Jewel Knocking Variables -->
    private double          clawPosition    = robot.getClawHome(); //Position of Servo for knocking of Jewels
    private double          x              = 0;                   //Position of jewel on the screen of the robot controller -- //No need for y position

    //Used for the timeout parameter in the encoder drive
    private ElapsedTime runtime = new ElapsedTime();

    //----------------------------------------------------------------------------------------------

    /* Vuforia Opmode members */
    VuforiaLocalizer vuforia; //Loads a Vuforia dataset from the indicated application asset
    VuforiaTrackables relicTrackables; //Traceable images, from 2017-2018 asset
    VuforiaTrackable relicTemplate; //An interface which sets an object that will receive notifications
    VuforiaLocalizer.Parameters parameters; //Used for setting parameters of cameraview and ensures that a valid Vuforia License Key is used
    RelicRecoveryVuMark vuMark;

    //----------------------------------------------------------------------------------------------

    //Encoder Vars
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CENTIMETERS  = 10.16 ;     // For figuring circumference
    static final double     COUNTS_PER_CENTIMETERS  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CENTIMETERS * 3.1415); //Allows for easier porgramming as distance can easily be added to the encoder drive method
    static final double     DRIVE_SPEED             = 0.7;  //Speed at which motors should run when moving forward, backward, left or right.
    static final double     TURN_SPEED              = 0.4;  //Speed at which motors should run when turning.


    //----------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() {


        //Initialize the drive system variables and servos
        robot.init(hardwareMap);

        bluevision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        bluevision.setShowBlue(false);
        bluevision.enable();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("",  "Encoders Reset");
        telemetry.update();

        robot.composeTelemetryInit(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //----------------------------------------------------------------------------------------------

        // INITIALISATION COMPLETE OF HARDWARE COMPLETE

        // ## Movement of robot happens next

        //----------------------------------------------------------------------------------------------

        //Telemetry header
        telemetry.addData("", "*******************");

        //STEP 1: Determine the jewel colour
        bluevision.setShowBlue(true);
        sleep(500); //Give OpenCV time to convert image and determine blue color

        //Get the x value of the largest concentration of blue(rectangle is drawn around)
        x = bluevision.getiJewelX();

        if (x >= 630) {

            //Change the claw position to move accordingly later
            clawPosition = 0.15;


        } else if (x <= 630) {

            //Change the claw position to move accordingly later
            clawPosition = 0.7;

        }

        telemetry.addData("", "Jewel Identified and knocked!");
        telemetry.update();

        //Set the new claw position
        robot.claw.setPosition(clawPosition);
        sleep(500);     // pause for servos to move

        //Retract the servo back into the robot
        robot.claw.setPosition(0.44);
        robot.claw2.setPosition(0.23);

        //Close the jewel identification program and camera view instance
        try {
            bluevision.setShowBlue(false);
            bluevision.disable();
        }catch (Exception e){
            telemetry.addData("An Exception has occured", "", e);
        }



            /* Init Vuforia: Starts up robot controller camera and retrieves vuMarks from assets folder in robot controller package */
        initVuforia();
        sleep(750);

        //Step 2 Scan vuMark(pictograph) using vuforia
        vuMark = runVuforia();    //Check if a vuMark is visible and return that vuMark instance and if seen display positional values of robot relative to vuMark
        telemetry.addData("", "VISIBLE: ", vuMark);
        telemetry.update();
        sleep(1500);
        //------------------------------------------------------------------------------------------


        if (vuMark == RelicRecoveryVuMark.LEFT) {

            //Move forward to left column ** Slower speed due to balance plate throwing robot off
            encoderDrive(0.35, -15, -15, -15, -15, 1.25);
            sleep(500);       //Wait for movement to finish

            //Turn to align with left column
            encoderDrive(0.6, -49, 79, -49, 79, 1.7);
            sleep(500);       //Wait for movement to finish

            //Move forward to Cryptobox
            encoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 0.60);
            sleep(500);       //Wait for movement to finish

            encoderDrive(0.5, -49, 79, -49, 79, 1.7);
            sleep(500);       //Wait for movement to finish

            encoderDrive(0.6, 25, 25, 25, 25, 1.5);
            sleep(500);       //Wait for movement to finish

            robot.arm.setPosition(0.67);
            sleep(700);       //Wait for movement to finish
            /*

            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 0.5);
            */

        } else if (vuMark == RelicRecoveryVuMark.CENTER) {

            //Move backward to center column ** Slower speed due to balance plate throwing robot off
            encoderDrive(0.35, -30, -30, -30, -30, 1.75);
            sleep(500);       //Wait for movement to finish

            //Turn to align with Center column
            encoderDrive(0.5, -49, 79, -49, 79, 1.7);
            sleep(750);       //Wait for movement to finish

            //Move forward to Cryptobox
            encoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 0.60);
            sleep(500);       //Wait for movement to finish

            //Turn to align with cryptobox
            encoderDrive(0.5, -49, 79, -49, 79, 1.7);
            sleep(500);       //Wait for movement to finish

            encoderDrive(0.6, 25, 25, 25, 25, 1.5);
            sleep(500);       //Wait for movement to finish

            robot.arm.setPosition(0.67);
            sleep(700);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 0.5);



        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

            //Move backward to Rght column ** Slower speed due to balance plate throwing robot off
            encoderDrive(0.35, -30, -30, -30, -30, 1.5);
            sleep(500);

            //Turn to align with left column
            encoderDrive(0.6, -49, 79, -49, 79, 1.7);
            sleep(500);       //Wait for movement to finish

            //Move forward to Cryptobox
            encoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 0.60);
            sleep(500);       //Wait for movement to finish

            encoderDrive(0.5, -49, 79, -49, 79, 1.7);
            sleep(500);       //Wait for movement to finish

            encoderDrive(0.6, 25, 25, 25, 25, 1.5);
            sleep(500);       //Wait for movement to finish

            robot.arm.setPosition(0.67);
            sleep(700);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 0.5);
            sleep(500);       //Wait for movement to finish

            encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 0.5);

        }

        //------------------------------------------------------------------------------------------

        robot.glyphPickupLiftServo.setPower(0.8);
        sleep(6000);
        robot.glyphPickupLiftServo.setPower(-0.05);
        sleep(500);
        robot.glyphPickupLift.setTargetPosition(-13000);
        robot.glyphPickupLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.glyphPickupLift.setPower(1.0);
        sleep(7000);
        robot.glyphPickupLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        //------------------------------------------------------------------------------------------

    }//End of runOpMode method

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftInchesBack, double rightInchesBack,
                             double timeoutS) {
        //For setting to what point the encoders on the drive motors should run
        int newLeftTarget;
        int newRightTarget;
        int newLefttargetBack;
        int newRightTargetBack;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_CENTIMETERS);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_CENTIMETERS);
            newLefttargetBack  = robot.leftDriveBack.getCurrentPosition() + (int)(leftInchesBack * COUNTS_PER_CENTIMETERS);
            newRightTargetBack = robot.rightDriveBack.getCurrentPosition() + (int)(rightInchesBack * COUNTS_PER_CENTIMETERS);

            //Set the target position
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDriveBack.setTargetPosition(newLefttargetBack);
            robot.rightDriveBack.setTargetPosition(newRightTargetBack);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed)); //Ensure that the speed value is positive through the use of the abs function
            robot.rightDrive.setPower(Math.abs(speed));
            robot.leftDriveBack.setPower(Math.abs(speed)+0.2); //Compensation to the motors due to them being very weird
            robot.rightDriveBack.setPower(Math.abs(speed)+0.3); //Compensation to the motors due to them being very weird

            /*
             Keep looping while we are still active, and there is time left, and both motors are running.
             * Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             * its target position, the motion will stop.  This is "safer" in the event that the robot will
             always end the motion as soon as possible.
             */
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftDriveBack.isBusy() && robot.rightDriveBack.isBusy())) {

                // Display it for the drivers.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            robot.setAllDriveMotorPower(0,0,0,0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //init Vuforia
    private void initVuforia(){
         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         Vuforia License Key obtained from -- https://developer.vuforia.com/license-manager --
         */
        parameters.vuforiaLicenseKey = "Afi7ZJb/////AAAAGSZPQI5OpkKRkWVA/oa7uKCJtmTcZUlDo1ZMlZfZ+aYKl+CTzQvPjhtoLE8n/cNTFSHvyUBnFpv9oPt/70QZlGQvgbnL/3TQ0x9UnTsNnz//HF8ctHrXTUfDNgAjYtxYFGvJMlNjfh8jBxFQKMQYJrLRFhlMcYEshS117KUjqy7Gwp3uCdF+eWdSsuPXinf0fj3DATNDYjdOB5ChcXh1r6DW3RkmIV7Q131gUFJBeVDKjxy0ZBmGBdWX+VhD/81xNpHZHbN8vVikjl7oOWD7HijxaOi8FFOKbNysVyNq0/UujoW6thNXXRo2fTb+1ZnpZ21Gfo2uAnDAQ5/xajmwNxEhCEW19O4DaVDlSRBq3vJi";

        /*
         * Back Camera allows for greater range
         * Front camera might be more convenient
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

    }

    private RelicRecoveryVuMark runVuforia(){

        //Retrieve relicTemplte and save it to vuMark variable for identification later
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN && (vuMark == RelicRecoveryVuMark.RIGHT || vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.LEFT)) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
            telemetry.update();
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        return vuMark;
    }//End of running method

    //Method allowing us to display position info in telemetry
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
