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

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Working Drive Mode", group="Linear Opmode")
@Disabled

public class WorkingNormal extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDriveTop = null;
    private DcMotor rightDriveTop = null;
    private DcMotor leftDriveBot = null;
    private DcMotor rightDriveBot = null;

    double leftPowerTop;
    double rightPowerTop;
    double leftPowerBot;
    double rightPowerBot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("GOOD LUCK!", "GO HELPIES");
        telemetry.addData(">>>>>>>>>>>>>>", null);
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDriveTop  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveTop = hardwareMap.get(DcMotor.class, "right_drive");
        leftDriveBot = hardwareMap.get(DcMotor.class, "left_drive_bot");
        rightDriveBot = hardwareMap.get(DcMotor.class, "right_drive_bot");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDriveTop.setDirection(DcMotor.Direction.FORWARD);
        rightDriveTop.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBot.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBot.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;


            leftPowerTop = Range.clip(drive + turn, -1.0, 1.0);
            rightPowerTop = Range.clip(drive - turn, -1.0, 1.0);
            leftPowerBot = Range.clip(drive + turn, -1.0, 1.0);
            rightPowerBot = Range.clip(drive - turn, -1.0, 1.0);


            if(gamepad1.a){

                diagnolLeft();

            }

            if(gamepad1.b){

                diagnolRight();

            }

            if(gamepad1.right_bumper){

                halfSpeed();

            }

            if(gamepad1.left_bumper){

                quarterSpeed();

            }

            if(gamepad1.left_trigger == 1){

                lateral();

            }
            leftDriveBot.setPower(leftPowerTop);
            rightDriveTop.setPower(rightPowerTop);
            leftDriveTop.setPower(leftPowerBot);
            rightDriveBot.setPower(rightPowerBot);

            // Show the elapsed game time and wheel power.
            telemetry.addData("*****************************", null);
            telemetry.addData("Status", "RUN TIME " + runtime.toString());
            telemetry.addData("MOTORS TOP", "left (%.2f), right (%.2f)", leftPowerTop, rightPowerTop);
            telemetry.addData("MOTORS BOTTOM", "left (%.2f), right (%.2f)", leftPowerBot, rightPowerBot);
            telemetry.addData("*****************************", null);

            if(runtime.seconds() <= 30){

                telemetry.addData("END GAME HAS COMMENCED", "!");

            }

            telemetry.update();
        }

        telemetry.addData("Run Complete", ">");
        telemetry.addData("How did it go", "?");
        telemetry.addData("<<<<<<<<<<<<<<<<", null);
    }

    private void diagnolLeft(){

        leftPowerTop = 1;
        rightPowerTop = 0;
        leftPowerBot = 0;
        rightPowerBot = 1;

    }


    private void diagnolRight(){

        leftPowerTop = 0;
        rightPowerTop = 1;
        leftPowerBot = 1;
        rightPowerBot = 0;

    }

    private void halfSpeed(){

        leftPowerTop = leftPowerTop / 2;
        rightPowerTop = rightPowerTop / 2;
        leftPowerBot = leftPowerBot / 2;
        rightPowerBot = rightPowerBot / 2;

    }

    private void quarterSpeed(){

        leftPowerTop = leftPowerTop / 4;
        rightPowerTop = rightPowerTop / 4;
        leftPowerBot = leftPowerBot / 4;
        rightPowerBot = rightPowerBot / 4;

    }

    private void lateral(){

        leftDriveTop.setDirection(DcMotor.Direction.REVERSE);
        rightDriveTop.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBot.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBot.setDirection(DcMotor.Direction.REVERSE);


    }

    private void normalOverride(){

        leftDriveTop.setDirection(DcMotor.Direction.FORWARD);
        rightDriveTop.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBot.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBot.setDirection(DcMotor.Direction.REVERSE);

    }


}
