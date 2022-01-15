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

        telemetry.addData("Yes Lilly, you can signal thumbs-up now:","");
        telemetry.update();

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;

        telemetry.addData("Position is: ", position);
        telemetry.update();

        //Autonomous: Delivers Pre-loaded Block and Parks in Warehouse
        //Position: Facing forward

        base.encoderDrive(0.5, var.CLEAR_WALL, var.CLEAR_WALL, this); //clear wall
        base.gyroTurn(0.5,50,this); //face hub
        base.encoderDrive(0.7,15,15,this); //drive to hub
        base.gyroTurn(0.5,25,this);

        switch (position) {
            case LEFT: //lvl. 1
                base.liftAuto(1,this);
                while(base.lift.isBusy());
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case MIDDLE: //lvl. 2
                base.liftAuto(2,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.encoderDrive(.3,1.5,1.5,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                base.encoderDrive(.5,-1.5,-1.5,this);
                break;
            case RIGHT: //lvl. 3
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(1000);
                base.encoderDrive(.3,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                base.encoderDrive(.3,-2,-2,this);
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
        base.gyroTurn(0.5,5,this);
        base.encoderDrive(0.5,-13,-13,this);
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN); // set and close bucket
        base.liftAuto(0,this); //Bring lift down
        sleep(1500);
        base.gyroTurn(.5,-90,this); // turn towards warehouse
        base.encoderDrive(.5,40,40,this); //drive into warehouse
        }
}