package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "BLUE SU PARK")
public class BlueSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, false);

        base.init(hardwareMap,this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Blue autonomous: Delivers Duck and Parks in Storage Unit
        //Position: Back facing Carousel (Back 10 degrees from wall.)
        //-13 in, 50r, 32 in, 140r, 8 in
        //base.gyroTurn(.5, -30, this);

        base.encoderDrive(0.5,-18.3,-18.3,this); // drive to Carousel
        base.leftDuck.setPower(-.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.gyroTurn(.5,100,this); //rotate front towards SU
        base.encoderDrive(.5,14,14,this);// drive into SU
        telemetry.addData("Parked in Blue SU :)","");
        telemetry.update();

    }
}
