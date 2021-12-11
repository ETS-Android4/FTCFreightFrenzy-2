package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;

@Disabled
@Autonomous(name= "BLUE Carousel")
public class BlueCarousel extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        detector.getDecision();

        //---------------- CASE LEFT ----------------
        if (position == ObjectDetector.POSITIONS.LEFT) {
            //Robot movements for "Case Left" position of team marker.


        }

        //---------------- CASE MIDDLE ----------------
        else if (position == ObjectDetector.POSITIONS.MIDDLE) {
            //Robot movements for "Case Middle" position of team marker.

        }

        //---------------- CASE RIGHT ----------------
        else if (position == ObjectDetector.POSITIONS.RIGHT) {
            //Robot movements for "Case Right" position of team marker.

        }
    }
}
