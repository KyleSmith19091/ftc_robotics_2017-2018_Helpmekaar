package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by KyleSmith on 12/12/17.
 */
@TeleOp(name = "Test")
@Disabled
public class Test1 extends OpMode {


    /*Declare Opmode members*/
    private DcMotor motorLeftTop;
    private DcMotor motorRightTop;
    private DcMotor motorLeftBottom;
    private DcMotor motorRightBottom;

    @Override
    public void init() {
        motorLeftTop = hardwareMap.get(DcMotor.class, "left_drive");
        motorRightTop = hardwareMap.get(DcMotor.class, "right_drive");
        motorLeftBottom = hardwareMap.get(DcMotor.class, "left_drive_bot");
        motorRightBottom = hardwareMap.get(DcMotor.class, "right_drive_bot");

        //Set directions of motors
        motorLeftTop.setDirection(DcMotor.Direction.FORWARD);
        motorRightTop.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBottom.setDirection(DcMotor.Direction.REVERSE);
        motorRightBottom.setDirection(DcMotor.Direction.FORWARD);

        motorLeftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {

        motorLeftTop.setPower(1);
        motorRightTop.setPower(1);
        motorLeftBottom.setPower(1);
        motorRightBottom.setPower(1);

        telemetry.addData("LEFT MOTOR POWER", motorLeftBottom);
        telemetry.addData("RIGHT MOTOR POWER", motorRightBottom);
        telemetry.addData("Left", motorLeftTop);
        telemetry.addData("Right", motorRightTop);

    }
}
