package org.firstinspires.ftc.teamcode.Competition.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by KyleSmith on 1/9/18.
 */

public class Robot{

    //Add motors here -->
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDriveBack = null;
    public DcMotor  rightDriveBack = null;
    public DcMotor  glyphPickupLift = null;
    public DcMotor  glyphPickupLeft = null;
    public DcMotor  glyphPickupRight = null;
    public DcMotor  relicExtensionArmMotor = null;

    //Motor Variables -->
    private double clipValue = 0.85;

    //----------------------------------------------------------------------------------------------

    //Add Servos here -->
    public Servo    arm         = null;
    public Servo    claw        = null;
    public Servo    claw2       = null;
    public Servo    relicExtensionArmMouth = null;
    public Servo    relicExtensionArmRotater  = null;
    public CRServo glyphPickupLiftServo = null;

    //Servo Home Position Values -->
    private final static double ARM_HOME = 0.23;
    private final static double CLAW_HOME = 0.50;
    private final static double CLAW_HOME2 = 0.80;
    private final static double RELIC_EXTENSION_ARM_MOUTH_HOME = 0.23;
    private final static double RELIC_EXTENSION_ARM_ROTATER = 0.23;
    private final static double GLYPH_PICKUP_LIFT_SERVO_HOME = -0.05;

    //Servo Minimum Position Values -->
    private final static double ARM_MIN_RANGE  = -100;
    private final static double GLYPH_PICKUP_LIFT_SERVO_MIN_RANGE = -1.0;
    private final static double RELIC_EXTENSION_ARM_MOUTH_MIN_RANGE = -1.0;
    private final static double RELIC_EXTENSION_ARM_ROTATER_MIN_RANGE = -1.0;

    //Servo Maximum Position Values -->
    private final static double CLAW_MAX_RANGE  = 1.1;
    private final static double CLAW_MIN_RANGE  = 0.20;
    private final static double CLAW_MAX_RANGE2 = 1.1;
    private final static double CLAW_MIN_RANGE2  = -1.1;
    private final static double GLYPH_PICKUP_LIFT_SERVO_MAX_RANGE = 1.0;
    private final static double ARM_MAX_RANGE  = 100;
    private final static double RELIC_EXTENSION_ARM_MOUTH_MAX_RANGE = 1.0;
    private final static double RELIC_EXTENSION_ARM_ROTATER_MAX_RANGE = 1.0;



    //----------------------------------------------------------------------------------------------

    /* Local OpMode members. */
    public HardwareMap hwMap  = null;

    /* Default Constructor */
    public Robot() {
    }

    //----------------------------------------------------------------------------------------------

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;

        //Initialise motors
        initMotors();

        //Set the direction of the motors
        setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        stopMotors();

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //Reset Encoders
        setRunmodeInit(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunmodeInit(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialise Servos
        initServos();

    }

    //----------------------------------------------------------------------------------------------

    // Define and Initialise ALL installed Motors
    public void initMotors(){

        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDriveBack = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
        glyphPickupLift = hwMap.get(DcMotor.class, "glyph_pickup_lift");
        glyphPickupLeft = hwMap.get(DcMotor.class, "glyph_pickup_left");
        glyphPickupRight = hwMap.get(DcMotor.class, "glyph_pickup_right");
        relicExtensionArmMotor = hwMap.get(DcMotor.class, "relic_extension_arm_motor");

    }

    //----------------------------------------------------------------------------------------------

    // Define and initialize ALL installed servos.
    public void initServos(){

        arm  = hwMap.get(Servo.class, "arm");
        claw = hwMap.get(Servo.class, "claw");
        claw2 = hwMap.get(Servo.class, "claw2");
        glyphPickupLiftServo = hwMap.get(CRServo.class, "glyph_pickup_lift_servo");
        relicExtensionArmMouth = hwMap.get(Servo.class, "relic_extension_arm_mouth");
        relicExtensionArmRotater = hwMap.get(Servo.class, "relic_extension_arm_rotater");

        arm.setPosition(ARM_HOME);
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW_HOME2);
        glyphPickupLiftServo.setPower(GLYPH_PICKUP_LIFT_SERVO_HOME);

    }

    //----------------------------------------------------------------------------------------------

    //Method for setting the motor runMode, used for convenience -- Method is declared twice due to problems caused when using in autonomous class
    public void setRunmodeInit(DcMotor.RunMode runMode){

        leftDrive.setMode(runMode);
        rightDrive.setMode(runMode);
        leftDriveBack.setMode(runMode);
        rightDriveBack.setMode(runMode);
        glyphPickupLift.setMode(runMode);

    }

    //Method for setting the motor runMode, used for convenience
    public void setRunmodeDrive(DcMotor.RunMode runMode){

        leftDrive.setMode(runMode);
        rightDrive.setMode(runMode);
        leftDriveBack.setMode(runMode);
        rightDriveBack.setMode(runMode);

    }

    //Method for setting the motor direction, used for convenience
    public void setDirection(DcMotor.Direction motorDirection){

        leftDrive.setDirection(motorDirection);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(motorDirection);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        glyphPickupLift.setDirection(motorDirection);
        glyphPickupLeft.setDirection(motorDirection);
        glyphPickupRight.setDirection(motorDirection);
        relicExtensionArmMotor.setDirection(motorDirection);

    }

    //Sets all motors to the same power
    public void stopMotors(){

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        glyphPickupLift.setPower(0);
        glyphPickupLeft.setPower(0);
        glyphPickupRight.setPower(0);
        relicExtensionArmMotor.setPower(0);

    }
    //----------------------------------------------------------------------------------------------

    public void clipInit(float frontLeft, float frontRight, float backLeft, float backRight){

        frontLeft = (float) Range.clip(frontLeft, -clipValue, clipValue);
        frontRight = (float) Range.clip(frontRight, -clipValue, clipValue);
        backLeft = (float) Range.clip(backLeft, -clipValue, clipValue);
        backRight = (float) Range.clip(backRight, -clipValue, clipValue);


    }

    public void clipDrive(double frontLeft, double frontRight, double backLeft, double backRight){

        frontLeft = (float) Range.clip(frontLeft, -clipValue, clipValue);
        frontRight = (float) Range.clip(frontRight, -clipValue, clipValue);
        backLeft = (float) Range.clip(backLeft, -clipValue, clipValue);
        backRight = (float) Range.clip(backRight, -clipValue, clipValue);

    }

    public void setTargetPosition(int newLeftTarget, int newRightTarget, int newLeftTargetBack, int newRightTargetBack){

          leftDrive.setTargetPosition(newLeftTarget);
          rightDriveBack.setTargetPosition(newRightTarget);
          leftDriveBack.setTargetPosition(newLeftTargetBack);
          rightDriveBack.setTargetPosition(newRightTargetBack);

    }

    public void setAllDriveMotorPower(double frontLeft, double frontRight, double backleft, double backRight){

        leftDrive.setPower(frontLeft);
        rightDrive.setPower(frontRight);
        leftDriveBack.setPower(backleft);
        rightDriveBack.setPower(backRight);


    }

    //----------------------------------------------------------------------------------------------

    public void composeTelemetryDriver(Telemetry telemetry, double armPosition, double clawPosition, double claw2Position, double glyphPickupLiftServoPosition, float FrontLeft, float FrontRight, float BackLeft, float BackRight){

        telemetry.addData("Motors", "-->");
        telemetry.addData("Left Motor Front : Right Motor Front", "%.2f : %.2f", FrontLeft, FrontRight);
        telemetry.addData("Left Motor Back : Right Motor Back", "%.2f : %.2f", BackLeft, BackRight);
        telemetry.addData("", "////////////////////////");
        telemetry.addData("Servos", "-->");
        telemetry.addData("Arm",   "%.2f", armPosition);
        telemetry.addData("Claw",  "%.2f", clawPosition);
        telemetry.addData("Claw 2", "%.2f", claw2Position);
        telemetry.addData("Lift", "%.2f",   glyphPickupLiftServoPosition);
        telemetry.addData("", "/////////////////////////");
        telemetry.update();

    }

    public void composeTelemetryInit(Telemetry telemetry){

        telemetry.addData("", "**************");
        telemetry.addData("", "Hello Helpmekaar Kollege Robotics Team");
        telemetry.addData("", "GOOD LUCK");
        telemetry.addData("", "Press Play To Start -->");
        telemetry.addData("", "**************");
        telemetry.update();

    }

    //----------------------------------------------------------------------------------------------

    //Getters and Setters for the Servos Home, Minimum, Max range
    public  double getGlyphpickupliftservoMinRange() {return GLYPH_PICKUP_LIFT_SERVO_MIN_RANGE;}

    public double getGlyphpickupliftservoMaxRange() {return GLYPH_PICKUP_LIFT_SERVO_MAX_RANGE;}

    public double getGlyphpickupliftServoHome(){return GLYPH_PICKUP_LIFT_SERVO_HOME;}

    public  double getArmHome() {
        return ARM_HOME;
    }

    public  double getClawHome() {
        return CLAW_HOME;
    }

    public  double getClawHome2() {
        return CLAW_HOME2;
    }

    public  double getArmMinRange() {
        return ARM_MIN_RANGE;
    }

    public  double getArmMaxRange() {
        return ARM_MAX_RANGE;
    }

    public  double getClawMinRange() {
        return CLAW_MIN_RANGE;
    }

    public  double getClawMaxRange() {
        return CLAW_MAX_RANGE;
    }

    public  double getClawMaxRange2() {return CLAW_MAX_RANGE2;}

    public  double getClawMinRange2() {return CLAW_MIN_RANGE2;}

    public double getClipValue() {return clipValue;}

    public double getRelicExtensionArmMouthHome(){return RELIC_EXTENSION_ARM_MOUTH_HOME;}

    public double getRelicExtensionArmMouthMaxRange(){return RELIC_EXTENSION_ARM_MOUTH_MAX_RANGE;}

    public double getRelicExtensionArmRotaterMinRange(){return RELIC_EXTENSION_ARM_MOUTH_MIN_RANGE;}

    public double getRelicExtensionArmRotater(){return RELIC_EXTENSION_ARM_MOUTH_HOME;}
    //----------------------------------------------------------------------------------------------





}
