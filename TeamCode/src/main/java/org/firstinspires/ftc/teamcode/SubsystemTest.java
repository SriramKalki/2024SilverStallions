package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(group="Testing")
public class SubsystemTest extends LinearOpMode {
    private DcMotor leftPitchMotor = null;
    private DcMotor rightPitchMotor = null;
    private DcMotor extensionMotor = null;

    PIDController pid;

    public static double kP=0,kI=0,kD=0,kF=0;
    public static int pitchGoal = 0;

    @Override
    public void runOpMode(){
        pid = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        leftPitchMotor = hardwareMap.get(DcMotor.class, "leftPitchMotor");
        rightPitchMotor = hardwareMap.get(DcMotor.class, "rightPitchMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        leftPitchMotor.setDirection(DcMotor.Direction.REVERSE);

        leftPitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            pid.setPID(kP,kI,kD);
            double currPos = rightPitchMotor.getCurrentPosition();
            double angle = (Math.PI/560.0) * currPos - 15 * Math.PI/56.0;
            double output = pid.calculate(currPos,pitchGoal) + kF * Math.cos(angle);


            leftPitchMotor.setPower(output);
            rightPitchMotor.setPower(output);

            telemetry.addData("Output: ", output);
            telemetry.addData("Target: ", pitchGoal);
            telemetry.addData("Current: ", currPos);
            telemetry.addData("Cosine: ", Math.cos(angle));
            telemetry.addData("Angle: ", angle);
            telemetry.update();
        }
    }
}
