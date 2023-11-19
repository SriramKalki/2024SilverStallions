package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.MIDDLE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Config
@Autonomous(name="Red Near Auton")
public class RedNearAuton extends LinearOpMode {
    private NewRedPropProcessor.Location location = MIDDLE;
    private NewRedPropProcessor redPropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotor climb;
    private DcMotor plane;
    private Servo latch, boxWrist, intakeWrist;
    private CRServo intake, belt; // belt is orange pass through thing
    SampleMecanumDrive drive;

    public static double wristVal = 0.1;
    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new NewRedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        initHardware();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(30)
                .turn(Math.toRadians(-90))

                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(3)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .addTemporalMarker(() -> belt.setPower(0))
                .waitSeconds(2)
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //.turn(Math.toRadians(-1))
                .back(27)
                //.turn(Math.toRadians(-90))
                .turn(Math.toRadians(180))
                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .addTemporalMarker(() -> belt.setPower(0))
                .waitSeconds(2)
                .back(10)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(180))
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .forward(10)
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //.turn(Math.toRadians(-1))
                .strafeLeft(4)
                .back(30)
                .turn(Math.toRadians(90))

                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .waitSeconds(2)
                .back(5)
                .strafeLeft(10)
                .forward(30)
                .waitSeconds(1)
                .turn(Math.toRadians(180))
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .build();

        while(!isStarted()){
            location = redPropProcessor.getLocation();
            telemetry.update();
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
        slide = hardwareMap.get(DcMotor.class, "slide");
        climb = hardwareMap.get(DcMotor.class, "climb");
        plane = hardwareMap.get(DcMotor.class, "plane");
        latch = hardwareMap.get(Servo.class, "latch");
        boxWrist = hardwareMap.get(Servo.class, "boxWrist");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        belt = hardwareMap.get(CRServo.class, "belt");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latch.setPosition(1);
    }



}
