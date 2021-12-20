package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Disabled
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

        telemetry.addData("Yes Lilly, you can signal thumbs-up now.","");
        telemetry.update();

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        telemetry.addData("Position is: ", position);
        telemetry.update();

        //Autonomous: Delivers Pre-loaded Block and Parks in Warehouse
        //Position: Facing forward

        base.encoderDrive(.5, var.CLEAR_WALL, var.CLEAR_WALL, this); //clear wall
        base.gyroTurn(.5,10,this); //face hub
        base.encoderDrive(.5,7,7,this); //drive to hub
        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1
                base.liftAuto(1,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
            case MIDDLE: //lvl. 2
                base.liftAuto(2,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.3,1.5,1.5,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.5,-1.5,-1.5,this);
                break;
            case RIGHT: //lvl. 3
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.3,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                base.encoderDrive(.3,-2,-2,this);
                break;
            default: //Fallback auto if detection fails
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_OPEN);
                sleep(1000);
                base.encoderDrive(.3,2,2,this);
                base.leftClaw.setPosition(var.LCLAW_OPEN);
                sleep(5000);
                break;
        }

        base.encoderDrive(.5,-8,-8,this);
        base.bucket.setPosition(var.BUCKET_CLOSED);
        base.leftClaw.setPosition(var.LCLAW_CLOSED); // set and close bucket
        base.liftAuto(1,this); //Bring lift down
        sleep(2000);
        base.gyroTurn(.5,-100,this); // turn towards warehouse
        base.encoderDrive(.5,60,61,this); //drive into warehouse
        }
}
