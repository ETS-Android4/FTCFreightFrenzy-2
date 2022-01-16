package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Autonomous(name="BLUE-WH DELIVERY")
public class BlueWH extends LinearOpMode{

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

        telemetry.addData("Yes Lilly, you can signal thumbs-up now","");
        telemetry.update();

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;

        //Autonomous: Delivers Pre-loaded Block and Parks in Warehouse
        //Position: Facing forward

        base.encoderDrive(0.5, 12, 12, this); //clear wall
        base.gyroTurn(0.5,-57,this); //face hub
        base.bucket.setPosition(var.BUCKET_CLOSED);

        switch (position) {
            case LEFT: //lvl. 1
                base.encoderDrive(0.5,7,7,this);
                base.liftAuto(1,this);
                while(base.lift.isBusy());
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case MIDDLE: //lvl. 2
                base.encoderDrive(0.5,8,8,this);
                base.liftAuto(2,this);
                while(base.lift.isBusy());
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case RIGHT: //lvl. 3
                base.encoderDrive(0.5,9,9,this);
                base.liftAuto(3,this);
                while(base.lift.isBusy());
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            default: //Fallback auto if detection fails
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(1000);
                base.encoderDrive(.3,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
        }

        base.encoderDrive(0.5,-13,-13,this);
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN); // set and close bucket
        base.liftAuto(0,this); //Bring lift down
        sleep(1300);
        base.gyroTurn(.5,-115,this); // turn towards warehouse
        base.encoderDrive(0.8,30,30,this); //drive into warehouse
        base.gyroTurn(0.5,-150,this);
        base.encoderDrive(1.0,25,25,this);
    }
}