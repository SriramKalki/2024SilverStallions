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

@Autonomous(name="!!Meet 3 Blue Near Auton")
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
    public static int purplePixel = 5;
    public static int gameId = 0;
    public static double strafeDistance = 0;
    public static double distanceToBackdrop = 0;


    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        AprilTagProcessor aprilTagProcessor  = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor, aprilTagProcessor);

        initHardware();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(25)
                .turn(Math.toRadians(-90))
                .back(8)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .strafeRight(1)
                .strafeLeft(1)
                .forward(16)
                .turn(Math.toRadians(180))
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(purplePixel);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> boxWrist.setPosition(0.73))
                .waitSeconds(1)
                .back(28)
                .strafeLeft(5)
                .addTemporalMarker(() -> latch.setPosition(0.75))
                .waitSeconds(2)
                .forward(5)
                .addTemporalMarker(() -> boxWrist.setPosition(0.37))
                //.strafeLeft(20)
                //strafeRight(20)
                //.back(10)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(33.5)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .strafeRight(1)
                .strafeLeft(1)
                .forward(15)
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(purplePixel);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> boxWrist.setPosition(0.73))
                .waitSeconds(1)
                .back(35)
                .strafeLeft(8)
                .addTemporalMarker(() -> latch.setPosition(0.75))
                .waitSeconds(2)
                .forward(5)
                .addTemporalMarker(() -> boxWrist.setPosition(0.37))
                //.strafeLeft(25)
                //strafeRight(20)
                //.back(10)
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .back(30)
                .turn(Math.toRadians(-90))
                .forward(13)
                .addTemporalMarker(() -> pixel.setPosition(1))
                .waitSeconds(1)
                .strafeRight(1)
                .strafeLeft(1)
                .forward(10)
                .turn(Math.toRadians(180))
                .addTemporalMarker(() -> {
                    slide.setTargetPosition(purplePixel);
                    slide.setPower(0.5);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> boxWrist.setPosition(0.73))
                .waitSeconds(1)
                .back(9)
                .strafeRight(13)
                .addTemporalMarker(() -> latch.setPosition(0.75))
                .waitSeconds(2)
                .forward(5)
                .addTemporalMarker(() -> boxWrist.setPosition(0.37))
                //.strafeLeft(27)
                //strafeRight(13)
                //.back(10)
                .build();




        while(!isStarted()){
            location = bluePropProcessor.getLocation();
            telemetry.update();
        }

        waitForStart();
        switch(location){
            case LEFT:
                gameId = 4;
                drive.followTrajectorySequence(leftPurple);
                break;
            case MIDDLE:
                gameId = 5;
                drive.followTrajectorySequence(middlePurple);
                break;
            case RIGHT:
                gameId = 6;
                drive.followTrajectorySequence(rightPurple);
                break;
        }


        /*while(opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for(AprilTagDetection detection : detections) {
                int id = detection.id;

                if(id == gameId) {  // This AprilTag id is the one we're looking for!
                    AprilTagPoseFtc tagPose = detection.ftcPose; // gets the values for the location and orientation of april tag
                    strafeDistance = tagPose.range * Math.sin(Math.toRadians(tagPose.bearing)); //range = hypotenuse (distance from tag), bearing = angle of robot to tag
                    //ADD method to strafe this distance!!!
                    distanceToBackdrop = tagPose.range;//distance from tag
                    //ADD method to move to close to backdrop to outtake!
                    telemetry.addLine("Id:" + id);
                    telemetry.addLine("Strafe Distance:" + strafeDistance);
                    telemetry.update();
                    break;
                }

            }
            telemetry.update();
        }*/





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
