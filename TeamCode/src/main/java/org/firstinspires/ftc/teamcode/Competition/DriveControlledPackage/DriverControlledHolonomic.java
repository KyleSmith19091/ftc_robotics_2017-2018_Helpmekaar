/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Competition.DriveControlledPackage;

import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

/* Author: Helpmekaar Kollege Robotics Team 2018 */

@TeleOp(name="Driver Controlled Holonomic", group="TeleOP")
public class DriverControlledHolonomic extends LinearOpMode{

    /* Declare OpMode members. */
    Robot robot           = new Robot();
    OpenCVJewelID redVision  = new OpenCVJewelID();

    //Sound Variables
    public SoundPool mySound;
    public int starWarsID;

    /* Motor power variables */
    float  FrontLeft;
    float  FrontRight;
    float  BackRight;
    float  BackLeft;
    float  glyphPickupLift;

    /* Servo Variables */
    private double          armPosition     = robot.getArmHome();                   // Servo safe position
    private double          clawPosition    = robot.getClawHome();                  // Servo safe position
    private double          claw2POsition   = robot.getClawHome2();
    private final double    CLAW_SPEED      = 0.035 ;                            // sets rate to move servo
    private final double    ARM_SPEED       = 0.035 ;                            //sets rate to move servo

    /* Vuforia Opmode members */
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;

    @Override
    public void runOpMode() {

        /* Init Vuforia: Starts up robot controller camera and retrieves vuMarks from assets folder in robot controller package */
        //initVuforia();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        redVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        redVision.setShowBlue(false);

        // start the vision system
        redVision.enable();

        //Init soundpool to allow us to play sounds for debugging purposes
        //mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        //starWarsID = mySound.load(hardwareMap.appContext, R.raw.starwars, 1); // PSM



        // Send telemetry message to signify that the robot is waiting ---->
        telemetry.addData("", "**************");
        telemetry.addData("", "Hello Helpmekaar Kollege Robotics Team");
        telemetry.addData("", "GOOD LUCK");
        telemetry.addData("", "Press Play To Start -->");
        telemetry.addData("", "**************");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //Header of telemetry
            telemetry.addData("", "*****Robot Data*****");

            //----------------------------------

            runVuforia();

            //----------------------------------

            redVision.setShowBlue(gamepad1.start);

            //----------------------------------

            // Get values from gamepad 1
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // Holonomic formulas to allowing for vertical and horizaontal movement on gamepad1.left_stick


            FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            //----------------------------------

            //----------------------------------

            // Clip the right/left values so that the values never exceed +/- 0.2
            FrontRight = (float) Range.clip(FrontRight, -0.2, 0.2);
            FrontLeft = (float) Range.clip(FrontLeft, -0.2, 0.2);
            BackLeft = (float) Range.clip(BackLeft, -0.2, 0.2);
            BackRight = (float) Range.clip(BackRight, -0.2, 0.2);
            glyphPickupLift = (float) Range.clip(glyphPickupLift, -0.5, 0.5);

            // Prevent servo values from exceeding Max and Minimum range
            armPosition = (Range.clip(armPosition, robot.getArmMinRange(), robot.getArmMaxRange()));
            clawPosition =(Range.clip(clawPosition, robot.getClawMinRange(), robot.getClawMaxRange()));
            claw2POsition = (Range.clip(claw2POsition, robot.getClawMinRange2(), robot.getClawMaxRange2()));

            //----------------------------------

            // Adjust Motor max power for precise movement
            if (gamepad1.left_bumper) {
                /*
                FrontLeft = (float) ((-gamepad1LeftY - gamepad1LeftX - gamepad1RightX)/5.5);
                FrontRight = (float) ((gamepad1LeftY - gamepad1LeftX - gamepad1RightX)/5.5);
                BackRight = (float) ((gamepad1LeftY + gamepad1LeftX - gamepad1RightX)/5.5);
                BackLeft = (float) ((-gamepad1LeftY + gamepad1LeftX - gamepad1RightX)/5.5);
                */
            }
            else if(gamepad1.right_bumper) {
                /*
                FrontLeft = (float) ((-gamepad1LeftY - gamepad1LeftX - gamepad1RightX)/8);
                FrontRight = (float) ((gamepad1LeftY - gamepad1LeftX - gamepad1RightX)/8);
                BackRight = (float) ((gamepad1LeftY + gamepad1LeftX - gamepad1RightX)/8);
                BackLeft = (float) ((-gamepad1LeftY + gamepad1LeftX - gamepad1RightX)/8);
                */

            }else if(gamepad1.dpad_left) {

                //robot.clipMin(FrontLeft, FrontRight, BackRight, BackLeft);

            }else if(gamepad1.dpad_up){

                robot.glyphPickupLift.setPower(0.12);

            }else if(gamepad1.dpad_down){

                robot.glyphPickupLift.setPower(-0.12);

            }else if(gamepad1.dpad_right){

                robot.glyphPickupLift.setPower(0.0);

            }else if(gamepad1.left_trigger > 0){

                redVision.disable();

                initVuforia();

            }else if(gamepad1.right_trigger > 0){

                mySound.play(starWarsID,1,1,1,0,1);

            }else if(gamepad1.back){

                mySound.stop(starWarsID);

            }

            //----------------------------------

            //----------------------------------

            //Write Values to the motors
            robot.leftDrive.setPower(FrontLeft);
            robot.rightDrive.setPower(FrontRight);
            robot.leftDriveBack.setPower(BackLeft);
            robot.rightDriveBack.setPower(BackRight);

            //Move both servos to new position
            robot.arm.setPosition(armPosition);
            robot.claw.setPosition(clawPosition);
            robot.claw2.setPosition(claw2POsition);

            //----------------------------------

            //----------------------------------

            //Temporary Vars for raising and lowering jewel arm

            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.y) {armPosition += ARM_SPEED;}
            else if (gamepad1.a) {armPosition -= ARM_SPEED;}

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x) {clawPosition += CLAW_SPEED;}
            else if (gamepad1.b) {clawPosition += CLAW_SPEED;}

            //----------------------------------

            //----------------------------------

            // Send telemetry message to driver controller for debugging
            telemetry.addData("Arm",   "%.2f", armPosition);
            telemetry.addData("Claw",  "%.2f", clawPosition);
            telemetry.addData("Left",  "%.2f", FrontLeft);
            telemetry.addData("Right", "%.2f", FrontRight);
            telemetry.addData("Left back", "%.2f", BackLeft);
            telemetry.addData("Right back,","%.2f", BackRight);
            telemetry.addData("Rectangle X value: ", + redVision.getiJewelX());
            telemetry.addData("Rectangle Y value: ", + redVision.getiJewelY());
            telemetry.addData("", "*********************");
            telemetry.update();

            //----------------------------------

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }//End of while loop

    }//End of runOpmode method


    //init Vuforia
    public void initVuforia(){
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


    public void runVuforia(){

        //Retrieve relicTemplte and save it to vuMark variable for identification later
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

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
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

    }//End of running method

    //Method allowing us to display position info in telemetry
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}//End of class
