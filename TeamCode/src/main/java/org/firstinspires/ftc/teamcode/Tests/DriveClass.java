package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by KyleSmith on 2017/10/15.
 */

    /*
Robot motion is described in three different ways
-Axial: Forward/Backward + = Forward
-Laterial side to side + = Right
-Yaw Rotating + = Counter Clockwise
 */

public class DriveClass {

    /* Declare Opmode members */
    private LinearOpMode myOpMode;

    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;

    private Gamepad gamepad1;
    private Gamepad gamepad2;


//------------------------------
    //Constructor
    public DriveClass(){

    }
//------------------------------

    //init all vars... Method allows versatility as it can be changed for each class
    public void initDrive(LinearOpMode opMode){

        //Save references to Hardware Map //FIXME: Check if there is a problem here
        myOpMode = opMode;

        //Get references from hardware map class// Add stuff in that class to use them here
        motorLeft = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        motorRight = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");

        //Set directions of motors
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //Set if motors should run with encoders, use method so you don't have to type it out each time
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all motor's power to 0 to prevent any problems when the program is run
        driveMotorStop();
    }

    //Set Runmode of motors--- It is an Override method in the init method we assume that the motors will run with encoders!
    public void setMode(DcMotor.RunMode mode){
        motorLeft.setMode(mode);
        motorRight.setMode(mode);
    }

    //-------------------------
    //Drive states

    //Normal Drive
    public void allDrive(double left, double right){

       left = -gamepad1.left_stick_x;
       right = -gamepad1.right_stick_x;

       motorLeft.setPower(left);
        motorRight.setPower(right);

    }

    //For a bit more precise movement
    public void halfDrive(double left, double right){

        left = -gamepad1.left_stick_x/2;
        right = -gamepad1.right_stick_x/2;

        motorLeft.setPower(left);
        motorRight.setPower(right);

    }

    //For precision maneuvering
    public void quarterDrive(double left, double right){

        left = -gamepad1.left_stick_x/4;
        right = -gamepad1.right_stick_x/4;

    }

    public void driveMotorStop(){

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public void turnLeft(double right){

        motorLeft.setPower(0);
        motorRight.setPower(right);

    }

    public void turnRight(double left){

        motorLeft.setPower(left);
        motorRight.setPower(0);

    }

    //-------------------------



}
