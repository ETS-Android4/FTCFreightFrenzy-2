package org.firstinspires.ftc.teamcode.Auto.AltAutos;

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
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean keepGoing = true;
        while (keepGoing) {
            ObjectDetector.POSITIONS positionBefore = detector.getDecision();
            telemetry.addData("", positionBefore);
        }

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        detector.setTelemShow(false);

        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.encoderDrive(0.5, 10, 10, this);
                base.liftAuto(1, this);
                this.telemetry.update();
                while (base.lift.isBusy()) ;
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.encoderDrive(0.5, 11.2, 11.2, this);
                base.liftAuto(2, this);
                this.telemetry.update();
                while (base.lift.isBusy()) ;
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case RIGHT: //lvl. 3 and  open bucket
                base.encoderDrive(0.5, 13.5, 13.5, this);
                base.liftAuto(3, this);
                this.telemetry.update();
                while (base.lift.isBusy()) ;
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;

        }
    }
}
