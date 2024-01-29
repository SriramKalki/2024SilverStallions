package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group="Testing")
public class TestLinearSlideEncoder extends LinearOpMode {
    private DcMotor slide;
    @Override
    public void runOpMode(){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            slide.setPower(gamepad2.right_stick_y);
            telemetry.addData("Slide Encoder: ", slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
