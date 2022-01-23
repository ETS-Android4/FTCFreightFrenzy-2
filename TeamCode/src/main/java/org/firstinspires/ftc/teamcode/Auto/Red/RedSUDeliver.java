package org.firstinspires.ftc.teamcode.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Autonomous(name= "RED-SU DELIVERY")
public class RedSUDeliver extends LinearOpMode{

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


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //ObjectDetector.POSITIONS position = detector.getDecision();
        ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);
        //Blue autonomous: Delivers duck, delivers pre-loaded, and parks in WH
        //Position: Back facing Carousel (Back 10 degrees from wall.)

        base.encoderDrive(0.7,-19.4,-19.4,this); //Drives backwards to carousel
        base.rightDuck.setPower(var.DUCK_SPEED); //Spins carousel
        sleep(3500);
        base.rightDuck.setPower(0);
        base.gyroTurn(.5,-10,this); //rotate front towards SU
        base.encoderDrive(0.7,45,45,this);// drive into SU
        base.gyroTurn(0.5,-100,this);

        switch (position) { //hub level test result goes there <==
            case LEFT: //lvl. 1 and open bucket
                base.encoderDrive(0.5,10,10,this);
                base.liftAuto(1, this);
                this.telemetry.update();
                while(base.lift.isBusy());
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case MIDDLE: //lvl. 2 and open bucket
                base.encoderDrive(0.5,11.2,11.2,this);
                base.liftAuto(2, this);
                this.telemetry.update();
                while(base.lift.isBusy());
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            case RIGHT: //lvl. 3 and  open bucket
                base.encoderDrive(0.5,13.5,13.5,this);
                base.liftAuto(3, this);
                this.telemetry.update();
                while(base.lift.isBusy());
                base.encoderDrive(0.3,1.5,1.5,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
            default: //just in case
                base.encoderDrive(0.5,13,13,this);
                base.liftAuto(3, this);
                while(base.lift.isBusy());
                base.encoderDrive(0.3,1.2,1.2,this);
                base.bucket.setPosition(var.BUCKET_CLOSED);
                base.leftClaw.setPosition(var.LCLAW_CLOSED);
                sleep(1000);
                break;
        }

        base.encoderDrive(.5,-10,-10,this);
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN); // set and close bucket
        base.liftAuto(0,false, this); //Bring lift down
        base.gyroTurn(0.5,-30,this); //Turns diagonally towards WH
        base.encoderDrive(0.6,51,51,this); //Enters WH
        base.gyroTurn(0.5,-10,this); //Turns perpendicular to back wall
        base.encoderDrive(0.8,15,15,this); //Drives to topleft of WH
        telemetry.addData("Parked in RED WH:)","");
        base.lift.setPower(0);

        // Turn off RUN_TO_POSITION
        base.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.update();
    }
}