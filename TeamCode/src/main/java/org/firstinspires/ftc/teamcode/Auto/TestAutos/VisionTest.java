package org.firstinspires.ftc.teamcode.Auto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Autonomous(name="VISION Test")
public class VisionTest extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true,false);

        base.init(hardwareMap, this);

        //Once Gamepad 1 'A' is pressed, code exits while loop and provides safe stoppage of VISION Test.
        while (!gamepad1.a) {
            ObjectDetector.POSITIONS positionBefore = detector.getDecision();
            telemetry.addData("", positionBefore);
        }

        waitForStart();
    }
}