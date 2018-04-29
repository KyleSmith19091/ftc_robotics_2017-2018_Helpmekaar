package org.firstinspires.ftc.teamcode.Competition.DriveControlledPackage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Hardware.Robot;

@TeleOp(name = "Driver Controlled Mecannum", group = "TeleOP")
public class DriverControlledMecannum extends LinearOpMode{

    /* Declare OpMode members. */
    Robot robot           = new Robot();   //Custom  hardware class

    /* Motor power variables */
    float  FrontLeft;
    float  FrontRight;
    float  BackRight;
    float  BackLeft;

    /* Servo Variables */
    private double          armPosition     = robot.getArmHome();                   // Servo safe position
    private double          clawPosition    = robot.getClawHome();                  // Servo safe position
    private double          claw2Position   = robot.getClawHome2();                 // Servo safe position
    private double          glyphPickupLiftServoPosition = robot.getGlyphpickupliftServoHome();    // Servo safe position
    private double          relicExtensionArmMouthPositon = robot.getRelicExtensionArmMouthHome(); // Servo safe position
    private double          relicExtensionArmRotaterPosition = robot.getRelicExtensionArmRotater(); // Servo safe position
    private final double    CLAW_SPEED      = 0.035 ;                            // sets rate to move servo
    private final double    ARM_SPEED       = 0.040 ;                            //sets rate to move servo
    private final double    RELIC_ARM_SPEED = 0.035 ;                            //sets rate to move servo

    @Override
    public void runOpMode() {

        /* Initialise the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Retract the jewel bump servo back into the robot
        robot.claw.setPosition(0.44);
        robot.claw2.setPosition(0.23);

        // Send telemetry message to signify that the robot is waiting ---->
        robot.composeTelemetryInit(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Header of telemetry
            telemetry.addData("", "*****Robot Data*****");

            //--------------------------------------------------------------------------------------

            // Get values from gamepad 1
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            //--------------------------------------------------------------------------------------

            float gamepad2LeftY = -gamepad2.left_stick_y;

            //--------------------------------------------------------------------------------------

            //Mecannum drive algorithms/formulas to allowing for vertical and horizaontal
            FrontLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            BackLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;

            //--------------------------------------------------------------------------------------

            // Clip the right/left values so that the values never exceed +/- clip value specified in robot hardware class
            FrontRight = (float) Range.clip(FrontRight, -robot.getClipValue(), robot.getClipValue());
            FrontLeft = (float) Range.clip(FrontLeft, -robot.getClipValue(), robot.getClipValue());
            BackLeft = (float) Range.clip(BackLeft, -robot.getClipValue(), robot.getClipValue());
            BackRight = (float) Range.clip(BackRight, -robot.getClipValue(), robot.getClipValue());
            gamepad2LeftY = (float) Range.clip(gamepad2LeftY, -robot.getClipValue(), robot.getClipValue());

            //--------------------------------------------------------------------------------------

            // Prevent servo values from exceeding Max and Minimum range
            armPosition = (Range.clip(armPosition, robot.getArmMinRange(), robot.getArmMaxRange()));
            clawPosition =(Range.clip(clawPosition, robot.getClawMinRange(), robot.getClawMaxRange()));
            claw2Position = (Range.clip(claw2Position, robot.getClawMinRange2(), robot.getClawMaxRange2()));
            glyphPickupLiftServoPosition = (Range.clip(glyphPickupLiftServoPosition, robot.getGlyphpickupliftservoMinRange(), robot.getGlyphpickupliftservoMaxRange()));

            //--------------------------------------------------------------------------------------

            //For precise movement of the robot
            if (gamepad1.left_bumper) {

                //Clip values to 30% power
                robot.clipDrive(0.3, 0.3, 0.3, 0.3);

            }else if(gamepad1.right_bumper) {

                //Clip values to 50% power
                robot.clipDrive(0.5, 0.5, 0.5, 0.5);

            }else if(gamepad1.left_trigger > 0){

                robot.setAllDriveMotorPower(0,0,0,0);

            }

            //--------------------------------------------------------------------------------------

            //Enable glyph feeding system -- pickup glyphs
            if(gamepad2.left_trigger > 0) {

                robot.glyphPickupLeft.setPower(1.0);
                robot.glyphPickupRight.setPower(-1.0);

            }else if(gamepad2.right_trigger > 0){    //Shoot glyphs out with feeding system motors

                robot.glyphPickupLeft.setPower(-1.0);
                robot.glyphPickupRight.setPower(1.0);

            }else if(gamepad2.dpad_up){    //Raise glyph pickup platform to highest position for glyph using motor encoders for position

                robot.glyphPickupLift.setTargetPosition(13000);
                robot.glyphPickupLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.glyphPickupLift.setPower(1.0);
                sleep(7000);
                robot.glyphPickupLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }else if(gamepad2.dpad_down){  //Lower glyph pickup platform to optimal position for glyph pickup using motor encoders for position

                robot.glyphPickupLift.setTargetPosition(-13000);
                robot.glyphPickupLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.glyphPickupLift.setPower(1.0);
                sleep(7000);
                robot.glyphPickupLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }else if(gamepad2.left_bumper){  //Stop the glyph feeding system motors

                robot.glyphPickupLeft.setPower(0.0);
                robot.glyphPickupRight.setPower(0.0);

            }else if(gamepad2.dpad_left){  //Lower platform on which glyphs will be moved with feeding system motors
                //Time is used due to the fact that a continuous rotating servo is used which does not have encoders
                robot.glyphPickupLiftServo.setPower(0.8);
                sleep(7300);
                robot.glyphPickupLiftServo.setPower(-0.05);

            }else if(gamepad2.dpad_right){ //Retract back into robot(platform which glyphs will be moved on
                //Time is used due to the fact that a continuous rotating servo is used which does not have encoders
                robot.glyphPickupLiftServo.setPower(-0.8);
                sleep(8000);
                robot.glyphPickupLiftServo.setPower(-0.05);

            } else if(gamepad2.start){

                glyphPickupLiftServoPosition = -0.05;

            }

            //--------------------------------------------------------------------------------------

            //Write Values to the motors
            robot.leftDrive.setPower(FrontLeft);
            robot.rightDrive.setPower(FrontRight);
            robot.leftDriveBack.setPower(BackLeft);
            robot.rightDriveBack.setPower(BackRight);
            robot.relicExtensionArmMotor.setPower(gamepad2LeftY);

            //--------------------------------------------------------------------------------------

            //Temporary Vars for raising and lowering jewel arm

            // Use gamepad Y & A raise and lower the platform which throws glyphs
            if (gamepad2.y) {armPosition += ARM_SPEED;}
            else if (gamepad2.a) {armPosition -= ARM_SPEED;}

            //For changing the servo which grabs the relic position on relic extension arm
            if(gamepad2.right_stick_y > 0){

                relicExtensionArmMouthPositon += RELIC_ARM_SPEED;

            }else if(gamepad2.right_stick_y < 0){

                relicExtensionArmMouthPositon -= RELIC_ARM_SPEED;

            }

            //For changing the servo which rotates the head of servo position on relic extension arm
            if(gamepad2.right_stick_x > 0){

                relicExtensionArmRotaterPosition += RELIC_ARM_SPEED;

            }else if(gamepad2.right_stick_x < 0){

                relicExtensionArmRotaterPosition -= RELIC_ARM_SPEED;

            }

            //Move both servos to new position
            robot.arm.setPosition(armPosition);
            robot.glyphPickupLiftServo.setPower(glyphPickupLiftServoPosition);
            robot.relicExtensionArmMouth.setPosition(relicExtensionArmMouthPositon);
            robot.relicExtensionArmRotater.setPosition(relicExtensionArmRotaterPosition);

            //--------------------------------------------------------------------------------------

            // Send telemetry message to driver controller for debugging
            // -- Displays all the power/ postions of the components associated with the Custom Hardware class(Robot)
            robot.composeTelemetryDriver(telemetry, armPosition, clawPosition, claw2Position, glyphPickupLiftServoPosition, FrontLeft, FrontRight, BackLeft, BackRight);
            telemetry.addData("", "*********************");

            //--------------------------------------------------------------------------------------

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }//End of while loop

    }//End of runOpmode method

}//End of class



