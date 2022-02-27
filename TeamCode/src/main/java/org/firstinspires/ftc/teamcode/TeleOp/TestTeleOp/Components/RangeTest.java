package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOp.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.MainBase;

@TeleOp(name="Range Test")
public class RangeTest extends LinearOpMode {

    MainBase base = null;


    @Override
    public void runOpMode() {
        custom_init();
        waitForStart();
        while (opModeIsActive()) {
            custom_loop();
        }
    }

    public void custom_init() {
        base = new MainBase();
        base.init(hardwareMap, this);

        telemetry.addData("Initialization Complete!", "");
        telemetry.update();
    }

    public void custom_loop() {

        //MaxBotix RangeSensor
        telemetry.addData("Distance: ", base.MBRange.getDistance(DistanceUnit.INCH));

        //ModernRobotics RangeSensor
        /*telemetry.addData("BACK Range: ", base.backRange.getDistance(DistanceUnit.INCH));
        telemetry.addData("SIDE Range: ", base.sideRange.getDistance(DistanceUnit.INCH));*/
        telemetry.update();
    }
}