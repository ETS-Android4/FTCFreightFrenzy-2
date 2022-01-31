package org.firstinspires.ftc.teamcode.Auto.AltAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

@Disabled
@Autonomous(name="RED-WH DELIVERY ALT-PARK")
public class RedWHAltParking extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this,true,true);

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        base.gyro.resetZAxisIntegrator();

        ObjectDetector.POSITIONS position = detector.getDecision();
        //ObjectDetector.POSITIONS position = ObjectDetector.POSITIONS.RIGHT;
        detector.setTelemShow(false);

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Repositioning to face hub
        //base.gyroDrive(1.0,12,12,-1,0,0,this);
        base.encoderDrive(1.0, 12, 12, this); //Clears the wall
        base.gyroTurn(0.6,57,this); //Turns to face hub
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
        }

        //Drives backwards (away) from hub
        base.encoderDrive(1.0,-20,-20,this);

        //CLOSES bucket and claw
        base.bucket.setPosition(var.BUCKET_OPEN);
        base.leftClaw.setPosition(var.LCLAW_OPEN);

        //Brings down lift while parking
        base.liftAuto(0,false,this);

        //WH PARKING
        base.gyroTurn(0.5,10,this); //Turns towards elements in WH
        base.encoderDrive(1.0,30,30,this); //Drives into WH
    }
}
