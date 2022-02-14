package org.firstinspires.ftc.teamcode.Auto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;

@Autonomous(name="VISION-RED Test")
public class VisionTestRed extends LinearOpMode{

    MainBase base = new MainBase();

    public boolean GP1_DPADUP_HELD    = false;
    public boolean GP1_DPADSIDE_HELD  = false;
    public boolean GP1_Y_HELD         = false;

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true,true);

        if (gamepad1.dpad_up && !GP1_DPADUP_HELD) {
            GP1_DPADUP_HELD = true;
            detector = new ObjectDetector(this,true,false);
        }
        else if ((gamepad1.dpad_left || gamepad1.dpad_right) && !GP1_DPADSIDE_HELD) {
            GP1_DPADSIDE_HELD = true;
            detector = new ObjectDetector(this,false,false);
        }
        else if (gamepad1.y && !GP1_Y_HELD) {
            GP1_Y_HELD = true;
            detector = new ObjectDetector(this,true,true);
        }
        else {
            detector = new ObjectDetector(this,false,true);
        }

        if (!gamepad1.dpad_up) {
            GP1_DPADUP_HELD = false;
        }
        if (!gamepad1.dpad_left || !gamepad1.dpad_right) {
            GP1_DPADSIDE_HELD = false;
        }
        if (!gamepad1.y) {
            GP1_Y_HELD = false;
        }

        base.init(hardwareMap, this);

        //Once Gamepad 1 'A' is pressed, code exits while loop and provides safe stoppage of VISION Test.
        while (!gamepad1.a) {
            ObjectDetector.POSITIONS positionBefore = detector.getDecision();
            telemetry.addData("", positionBefore);
        }

        waitForStart();
    }
}