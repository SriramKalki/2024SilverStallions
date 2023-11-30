package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Blue Near Auton")
public class BlueNearAuton extends LinearOpMode {
    private NewBluePropProcessor.Location location = MIDDLE;
    private NewBluePropProcessor bluePropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotor climb;
    private DcMotor plane;
    private Servo latch, boxWrist, intakeWrist, pixel;
    private CRServo intake, belt; // belt is orange pass through thing
    SampleMecanumDrive drive;
    public static double wristVal = 0.65;
    public static int targetVal = 3000;
    public static double slidePower = 0.5;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);

        initHardware();

        // These positions are almost entirely wrong and need to be reversed. They are the Red positions:
        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(25)
                .turn(Math.toRadians(-90))
                .back(8)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(46)
                .turn(Math.toRadians(180))
                //.addTemporalMarker(() -> slide.setPower(slidePower))
                .addTemporalMarker(() -> slide.setTargetPosition(targetVal))
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(1)
                .addTemporalMarker(() -> latch.setPosition(0))
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(32.5)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(90))
                .back(33)
                .strafeRight(5)
                //.addTemporalMarker(() -> slide.setPower(slidePower))
                .addTemporalMarker(() -> slide.setTargetPosition(targetVal))
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(1)
                .addTemporalMarker(() -> latch.setPosition(0))
                .waitSeconds(1)
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(30)
                .turn(Math.toRadians(-90))
                .forward(14)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(22)
                .turn(Math.toRadians(180))
                .strafeRight(10)
                //.addTemporalMarker(() -> slide.setPower(slidePower))
                .addTemporalMarker(() -> slide.setTargetPosition(targetVal))
                .waitSeconds(1)
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(1)
                .addTemporalMarker(() -> latch.setPosition(0))
                .waitSeconds(1)
                .build();


        while(!isStarted()){
            location = bluePropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        drive.followTrajectorySequence(rightPurple);
        switch(location){

            /*case LEFT:
                drive.followTrajectorySequence(leftPurple);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePurple);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightPurple);
                break;*/
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
        pixel = hardwareMap.get(Servo.class, "pixel");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latch.setPosition(1);
        boxWrist.setPosition(0);
    }



}
