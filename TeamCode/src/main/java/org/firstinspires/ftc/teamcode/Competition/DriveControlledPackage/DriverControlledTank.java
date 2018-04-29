package org.firstinspires.ftc.teamcode.Competition.DriveControlledPackage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import static java.lang.Thread.sleep;

/**
 * Created by KyleSmith on 2/3/18.
 */
@TeleOp(name = "Driver controlled Tank", group = "TeleOP")
public class DriverControlledTank extends OpMode {

    /* Opmode members */
    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify that the robot is waiting ---->
        telemetry.addData("", "**************");
        telemetry.addData("", "Hello Helpmekaar Kollege Robotics Team");
        telemetry.addData("", "GOOD LUCK");
        telemetry.addData("", "Press Play To Start -->");
        telemetry.addData("", "**************");
        telemetry.update();

    }

    @Override
    public void loop() {

        double left;
        double right;
        double glyphArmPower = 0;

        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        right = (float) Range.clip(right, -0.5, 0.5);
        left = (float) Range.clip(left, -0.5, 0.5);

        if(gamepad1.left_bumper){

            right = (float) Range.clip(right, -0.25, 0.25);
            left = (float) Range.clip(left, -0.25, 0.25);

        }

            if (gamepad1.right_bumper) {

                robot.leftDrive.setPower(right);
                robot.rightDrive.setPower(right);

            }else {

                robot.leftDrive.setPower(left);
                robot.rightDrive.setPower(right);

            }

            if (gamepad2.x) {

                robot.leftClaw.setPosition(1);
                robot.rightClaw.setPosition(-1);

            }else if (gamepad2.b) {

                robot.leftClaw.setPosition(-1);
                robot.rightClaw.setPosition(1);

            }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();

        // Pause for 40 mS each cycle = update 25 times a second.
        try {
            sleep(40);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
