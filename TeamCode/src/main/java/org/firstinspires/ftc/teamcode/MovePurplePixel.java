package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class MovePurplePixel extends LinearOpMode {
    private Servo latch;
    public static double latchPos = 0.0;
    @Override
    public void runOpMode(){
        latch = hardwareMap.get(Servo.class, "latch");
        waitForStart();
        while(opModeIsActive()){
            latch.setPosition(latchPos);
        }
    }
}
