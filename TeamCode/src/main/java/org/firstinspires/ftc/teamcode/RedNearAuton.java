package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.MIDDLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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
    private Servo latch, boxWrist, intakeWrist, pixel;
    private CRServo intake, belt; // belt is orange pass through thing
    SampleMecanumDrive drive;

    public static double wristVal = 0.65;
    public static int targetVal = 5;
    public static double slidePower = 0.5;
    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        redPropProcessor = new NewRedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        initHardware();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                //on outake side
                .back(30)
                .turn(Math.toRadians(90))
                .forward(13)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(22)
                .turn(Math.toRadians(180))
                .strafeLeft(5)
                //drop off yellow
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(targetVal);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                })
                .waitSeconds(2)
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(2)
                .addTemporalMarker(() -> latch.setPosition(0))
                .waitSeconds(1)
                .build();


        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))

                //on the outtake side
                .back(32.5)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(-90))
                .back(33)
                //drop off yellow
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(targetVal);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                })
                .waitSeconds(2)
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(3)
                .addTemporalMarker(() -> latch.setPosition(0))
                .waitSeconds(1)
                .build();



        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))

                //on the outtake side
                .back(25)
                .turn(Math.toRadians(90))
                .back(8)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(46)
                .turn(Math.toRadians(180))
                .strafeRight(5)
                //drop off yellow
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(targetVal);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                })
                .waitSeconds(2)
                .addTemporalMarker(() -> boxWrist.setPosition(wristVal))
                .waitSeconds(2)
                .addTemporalMarker(() -> latch.setPosition(0))
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
        pixel = hardwareMap.get(Servo.class, "pixel");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latch.setPosition(1);
        boxWrist.setPosition(0);
    }



}
