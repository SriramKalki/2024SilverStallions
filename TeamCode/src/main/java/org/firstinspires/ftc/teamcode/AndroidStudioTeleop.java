package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Config
public class AndroidStudioTeleop extends LinearOpMode {

    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor leftPitchMotor = null;
    private DcMotor rightPitchMotor = null;
    private DcMotor extensionMotor = null;

    private Servo leftWristServo = null;
    private Servo rightWristServo = null;
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;

    PIDController pitchPid = new PIDController(0,0,0);
    public static double kP=0.01,kI=0,kD=0;
    public static double kF=0;

    boolean prevLeft = false;
    boolean leftClose = false;
    boolean prevRight = false;
    boolean rightClose = false;

    double pitchGoal = 0;

    boolean prevDpadUp = false;
    boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();


        while (opModeIsActive()) {
            pitchPid.setPID(kP,kI,kD);
            moveDrivetrain();
            moveExtension();
            moveWrist();
            moveClaw();






            telemetry.addData("Linear Slide Position: ", extensionMotor.getCurrentPosition());
            telemetry.addData("Left Pitch Position: ", leftPitchMotor.getCurrentPosition());
            telemetry.addData("Right Pitch Position: ", rightPitchMotor.getCurrentPosition());
            telemetry.addData("Pitch Goal: ", pitchGoal);
            telemetry.update();
        }

    }

    public void moveDrivetrain(){
        double drive = -gamepad1.left_stick_x;
        double turn = -gamepad1.left_stick_y;
        double strafe = - gamepad1.right_stick_x;

        double speedMod = 0.7;
        if(gamepad1.left_trigger >= 0.7){//sloow down
            speedMod = 0.2;
        }else if (gamepad1.right_trigger >= 0.7){//vroooom
            speedMod = 1.5;
        }

        double frontLeftPower = Range.clip((turn - drive) * speedMod - strafe, -1.0, 1.0);
        double frontRightPower = Range.clip((turn + drive) * speedMod + strafe, -1.0, 1.0);
        double backLeftPower = Range.clip((turn - drive) * speedMod + strafe, -1.0, 1.0);
        double backRightPower = Range.clip((turn + drive) * speedMod - strafe, -1.0, 1.0);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void moveExtension(){

        double extension = -gamepad2.left_stick_y;
        if(!prevDpadDown && gamepad1.dpad_down){
            pitchGoal += 30;
        }else if(!prevDpadUp && gamepad1.dpad_up){
            pitchGoal -= 30;
        }

        double output = pitchPid.calculate(rightPitchMotor.getCurrentPosition(),pitchGoal);
        leftPitchMotor.setPower(output);
        rightPitchMotor.setPower(output);

        extensionMotor.setPower(extension);
    }

    public void moveWrist(){
        //wrist control
        if(gamepad2.dpad_right){
            rightWristServo.setPosition(0.3);
            leftWristServo.setPosition(0.7);
        }else if(gamepad2.dpad_left){
            rightWristServo.setPosition(0.85);
            leftWristServo.setPosition(0.15);
        }
    }
    public void moveClaw(){
        if(!prevLeft && gamepad1.left_bumper){
            leftClose = !leftClose;
        }else if(!prevRight && gamepad1.right_bumper){
            rightClose = !rightClose;
        }
        leftClawServo.setPosition(leftClose ? 1 : 0);
        rightClawServo.setPosition(rightClose ? 1 : 0.4);
        prevLeft = gamepad1.left_bumper;
        prevRight = gamepad1.right_bumper;
    }

    public void initialize(){
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

        leftPitchMotor = hardwareMap.get(DcMotor.class, "leftPitchMotor");
        rightPitchMotor = hardwareMap.get(DcMotor.class, "rightPitchMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");

        leftPitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPitchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftPitchMotor.setDirection(DcMotor.Direction.REVERSE);

        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}
