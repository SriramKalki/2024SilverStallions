package org.firstinspires.ftc.teamcode.disabled;

import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.MIDDLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Disabled
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
    public static int targetVal = 1000;
    public static double slidePower = 0.5;
    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        redPropProcessor = new NewRedPropProcessor(telemetry);
        AprilTagProcessor aprilTagProcessor  = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor, aprilTagProcessor);

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
                .strafeRight(6)
                .back(37)
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))

                //on the outtake side
                .back(25)
                .turn(Math.toRadians(90))
                .back(8) //change
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> pixel.setPosition(0))
                .waitSeconds(1)
                .forward(12)
                .turn(Math.toRadians(180))
                .back(30)
                .strafeRight(12)
                .back(1)
                .build();



        while(!isStarted()){
            location = redPropProcessor.getLocation();
            telemetry.addLine("location: " + location);
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
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for(AprilTagDetection detection:detections) {
            int id = detection.id;
            AprilTagPoseFtc tagPose = detection.ftcPose;
            telemetry.addLine("Detected tag ID: " + id);
            //range - distance to tag
            telemetry.addLine("Distance to tag: " + tagPose.range);
            //bearing - angle to tag
            telemetry.addLine("Bearing to tag: " + tagPose.bearing);
            //yaw - angle of tag
            telemetry.addLine("Angle of tag: " + tagPose.yaw);

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
        latch.setPosition(0);
        boxWrist.setPosition(0);
    }



}
