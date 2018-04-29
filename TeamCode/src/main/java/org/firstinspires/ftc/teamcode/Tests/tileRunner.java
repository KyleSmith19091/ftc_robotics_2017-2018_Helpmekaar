package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
     Written by Kyle Smith
 */
@TeleOp(name = "Tilerunner teleop", group = "TestRun")
@Disabled
public class tileRunner extends OpMode {

    /*NON OPMODE MEMBERS*/
    //private VuforiaRobot vuforiaRobot;
    private ElapsedTime runtime = new ElapsedTime();

    /*Declare Opmode members*/
    private DcMotor motorLeftTop;
    private DcMotor motorRightTop;
    private DcMotor motorLeftBottom;
    private DcMotor motorRightBottom;




    @Override
    public void init() {

        //Get references from hardware map class// Add stuff in that class to use them here
        motorLeftTop = hardwareMap.get(DcMotor.class, "left_drive");
        motorRightTop = hardwareMap.get(DcMotor.class, "right_drive");
        motorLeftBottom = hardwareMap.get(DcMotor.class, "left_drive_bot");
        motorRightBottom = hardwareMap.get(DcMotor.class, "right_drive_bot");

        //Set directions of motors
        motorLeftTop.setDirection(DcMotor.Direction.FORWARD);
        motorRightTop.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBottom.setDirection(DcMotor.Direction.REVERSE);
        motorRightBottom.setDirection(DcMotor.Direction.FORWARD);

        //Set drive modes for all the registered motors
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //init Vuforia for object recognition
        //vuforiaRobot.initVuforia(hardwareMap);

        telemetry.addData("Let's do this", null);
        runtime.reset();

    }

    @Override
    public void loop() {

        double leftTop = 0;
        double rightTop = 0;
        double leftBot = 0;
        double rightBot = 0;

        leftTop = -gamepad1.left_stick_x;
        leftBot = -gamepad1.left_stick_x;
        rightTop = -gamepad1.right_stick_x;
        rightBot = -gamepad1.right_stick_x;



        if(gamepad1.left_bumper == true){

            halfDrive(leftTop, rightTop, leftBot, rightBot);

        }else if(gamepad1.right_bumper == true){

            quarterDrive(leftTop, rightTop, leftBot, rightBot);

        }else{

            normalDrive(leftTop, rightTop, leftBot, rightBot);

        }

        if(gamepad1.x == true){

            //vuforiaRobot.scanPictogram();

        }

        //Display motor values and Vuforia info
        telemetry.addData("################", null);
        telemetry.addData("LEFT MOTOR POWER: ", leftTop);
        telemetry.addData("RIGHT MOTOR POWER: ", rightTop);
        //telemetry.addData("COLUMN FOR GLYPH PLACEMENT:" , vuforiaRobot.vuMark);
        telemetry.addData("CURRENT TIME: ", runtime.seconds());
        if(runtime.seconds() <= 30){telemetry.addData("ENDGAME!!!!!!", null);}
        telemetry.addData("#################", null);
        telemetry.update();

    }

    private void setMode(DcMotor.RunMode runMode){

        motorLeftTop.setMode(runMode);
        motorRightTop.setMode(runMode);
        motorLeftBottom.setMode(runMode);
        motorRightBottom.setMode(runMode);

    }

    private void normalDrive(double leftTop, double rightTop, double leftBot, double rightBot){

        motorLeftTop.setPower(leftTop);
        motorRightTop.setPower(rightTop);
        motorLeftBottom.setPower(leftBot);
        motorRightBottom.setPower(rightBot);

    }

    private void halfDrive(double left, double right, double leftBot, double rightBot){

        motorLeftTop.setPower(left/2);
        motorRightTop.setPower(right/2);
        motorLeftBottom.setPower(leftBot);
        motorRightBottom.setPower(rightBot);

    }

    private void quarterDrive(double left, double right, double leftBot, double rightBot){

        motorLeftTop.setPower(left/4);
        motorRightTop.setPower(right/4);
        motorLeftBottom.setPower(leftBot);
        motorRightBottom.setPower(rightBot);

    }


}//End of class
