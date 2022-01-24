package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;

//Blue autonomous: Delivers Duck and Parks in Storage Unit
//Position: Back facing Carousel (Back 10 degrees from wall.)

@Disabled
@Autonomous(name= "BLUE-SU PARKING")
public class BlueSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {


        base.init(hardwareMap,this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Resets bucket & claw to avoid lift collision
        base.bucket.setPosition(0.90);
        base.leftClaw.setPosition(1.0);

        //Scores duck at carousel
        base.encoderDrive(0.8,-19.4,-19.4,this); //Drives backwards to carousel
        base.leftDuck.setPower(0.53); //Spins duck-wheel for duck soring
        sleep(2000); //Sleeps to allow for adequate spin time
        base.leftDuck.setPower(0); //Stops duck-wheel

        //SU PARKING
        base.gyroTurn(.5,110,this); //rotate front towards SU
        base.encoderDrive(.5,21.5,21.5,this);// drive into SU
        base.gyroTurn(.5,99,this);
    }
}
