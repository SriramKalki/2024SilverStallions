package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Blue Near Auton")
public class BlueNear extends LinearOpMode {
    private NewBluePropProcessor.Location location = MIDDLE;
    private NewBluePropProcessor BluePropProcessor;
    private VisionPortal visionPortal;

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

    SampleMecanumDrive drive;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        BluePropProcessor = new NewBluePropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), BluePropProcessor);

        initHardware();
        rightClawServo.setPosition(1);
        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .strafeLeft(3)
                .back(24)
                .turn(Math.toRadians(-90))
                .back(2)
                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightWristServo.setPosition(1-0.73);
                    leftWristServo.setPosition(0.73);
                    extensionMotor.setPower(0.5);
                    extensionMotor.setTargetPosition(500);
                    extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightClawServo.setPosition(0.3);
                })
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .strafeLeft(3)
                .back(24)
                .turn(Math.toRadians(180))
                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightWristServo.setPosition(1-0.73);
                    leftWristServo.setPosition(0.73);
                    extensionMotor.setPower(0.5);
                    extensionMotor.setTargetPosition(500);
                    extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightClawServo.setPosition(0.3);
                })
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .strafeLeft(3)
                .back(24)
                .turn(Math.toRadians(90))
                .back(2)
                //drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightWristServo.setPosition(1-0.73);
                    leftWristServo.setPosition(0.73);
                    extensionMotor.setPower(0.5);
                    extensionMotor.setTargetPosition(500);
                    extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightClawServo.setPosition(0.3);
                })
                .build();

        while(!isStarted()){
            location = BluePropProcessor.getLocation();
        }
        waitForStart();
        switch(location){
            case LEFT:
                drive.followTrajectorySequence(leftPurple);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePurple);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightPurple);
                break;
        }


    }



    public void initHardware(){

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



    }



}
