package org.firstinspires.ftc.teamcode.Competition.OpenCVClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Created by KyleSmith on 2/3/18.
 */
@TeleOp(name = "OpenCVTest")
@Disabled
public class OpenCVTest extends OpMode{

    OpenCVClass blueVision;

    @Override
    public void init() {
        blueVision = new OpenCVClass();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowBlue(false);
        // start the vision system
        blueVision.enable();
    }

    @Override
    public void loop() {
        blueVision.setShowBlue(gamepad1.x);
    }

    public void stop() {
        // stop the vision system
        blueVision.disable();
    }

}
