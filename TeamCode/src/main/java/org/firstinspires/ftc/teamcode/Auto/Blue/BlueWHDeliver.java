package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Autonomous: Delivers Pre-loaded Block and Parks in Warehouse
//Position: Facing forward

@Autonomous(name="BLUE-WH DELIVERY")
public class BlueWHDeliver extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("God Speed");
        telemetry.update();

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.LEFT;

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(1.0);

        //Positioning prior to scoring
        base.encoderDrive(0.7, 8, 8, this); //Clears back wall
        base.gyroTurn(0.5,45,this); //Faces shipping hub

        switch (position) {
            case LEFT: //SCORES IN FIRST (BOTTOM) TIER
                base.encoderDrive(0.7,11,11,this);
                base.liftAuto(1,false,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.encoderDrive(0.5,4,4,this);

                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //Drives backwards (away) from hub
                base.encoderDrive(0.5,-13,-13,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.5,-89,this); //Turns towards SHARED HUB
                base.encoderDrive(1.0,40,40,this); //Drives towards SHARED HUB

                //WH PARKING

                break;
            case MIDDLE: //SCORES IN SECOND (MIDDLE) TIER
                base.encoderDrive(0.7,12,12,this);
                base.liftAuto(2,false,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.encoderDrive(0.5,4,4,this);

                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(600);

                //Drives backwards (away) from hub
                base.encoderDrive(0.5,-13,-13,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(0.5,-89,this); //Turns towards SHARED HUB
                base.encoderDrive(1.0,40,40,this); //Drives towards SHARED HUB

                //WH PARKING
                break;
            case RIGHT: //SCORES IN THIRD (TOP) TIER
                base.encoderDrive(0.5,12,12,this);
                base.liftAuto(3,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                sleep(400);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);

                //Drives backwards (away) from hub
                base.encoderDrive(0.5,-13,-13,this);

                //CLOSES bucket and claw
                base.bucket.setPosition(var.BUCKET_OPEN);
                base.leftClaw.setPosition(var.LCLAW_OPEN);

                //Brings down lift while parking
                base.liftAuto(0,false,this);

                //Placement before WH PARKING
                base.gyroTurn(.5,-115,this); //Turns towards SHARED HUB
                base.encoderDrive(0.8,30,30,this); //Drives towards SHARED HUB

                //WH PARKING
                break;
        }
    }
}