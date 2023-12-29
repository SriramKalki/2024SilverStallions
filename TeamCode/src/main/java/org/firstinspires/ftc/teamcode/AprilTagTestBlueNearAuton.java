package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor.Location.MIDDLE;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="April Tag Test Blue Near Auton")
public class AprilTagTestBlueNearAuton extends LinearOpMode {
    private NewBluePropProcessor.Location location = MIDDLE;
    private NewBluePropProcessor bluePropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotor climb;
    private DcMotor plane;
    private Servo latch, boxWrist, intakeWrist, pixel;
    private CRServo intake, belt;
    private WebcamName webcam1;// belt is orange pass through thing
    SampleMecanumDrive drive;
    public static double wristVal = 0.7; //0.65
    public static int targetVal = 1000;
    public static double slidePower = 0.5;
    public static int idNum = 0;


    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        AprilTagProcessor aprilTagProcessor  = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor, aprilTagProcessor);

        initHardware();

        TrajectorySequence idOne = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(25)
                .build();
        TrajectorySequence idTwo = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(25)
                .build();
        TrajectorySequence idThree = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(25)
                .build();

        while(!isStarted()){
            location = bluePropProcessor.getLocation();
            telemetry.addLine("it is about to start");
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for(AprilTagDetection detection:detections) {
                int id = detection.id;
                idNum = id;
                AprilTagPoseFtc tagPose = detection.ftcPose;
                telemetry.addLine("Detected tag ID: " + id);
                //range - distance to tag
                telemetry.addLine("Distance to tag: " + tagPose.range);
                //bearing - angle to tag
                telemetry.addLine("Bearing to tag: " + tagPose.bearing);
                //yaw - angle of tag
                telemetry.addLine("Angle of tag: " + tagPose.yaw);

            }
            telemetry.update();
        }
        switch(idNum){
            case 1:
                drive.followTrajectorySequence(idOne);
                break;
            case 2:
                drive.followTrajectorySequence(idTwo);
                break;
            case 3:
                drive.followTrajectorySequence(idThree);
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
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");


        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latch.setPosition(0);
        boxWrist.setPosition(0);
    }



}
