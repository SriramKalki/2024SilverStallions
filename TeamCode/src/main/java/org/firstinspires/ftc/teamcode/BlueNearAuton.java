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
    private Servo latch, boxWrist, intakeWrist;
    private CRServo intake, belt; // belt is orange pass through thing
    SampleMecanumDrive drive;
    public static double wristVal = 0.1;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);

        initHardware();

        // These positions are almost entirely wrong and need to be reversed. They are the Red positions:
        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                /*on the intake side
                .back(30)
                .turn(Math.toRadians(90))
                //DROP PURPLE
                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(3)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .addTemporalMarker(() -> belt.setPower(0))
                .waitSeconds(2)
                //DROP PURPLE
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(2)
                .strafeRight(10)
                .build();*/

                //on the outtake side
                .back(30)
                .turn(-90)
                //.addTemporalMarker(() -> <SERVO>.setPower(1))
                .waitSeconds(1)
                //.addTemporalMarker(() -> <SERVO>.setPower(0))
                .strafeLeft(15)
                .turn(180)
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .addTemporalMarker(() -> slide.setPower(0))
                .addTemporalMarker(() -> latch.setPosition(0))
                .strafeRight(10)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                /*on the intake side
                .back(27)
                .turn(Math.toRadians(180))
                //DROP PURPLE
                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .addTemporalMarker(() -> belt.setPower(0))
                .waitSeconds(2)
                //DROP PURPLE
                .forward(10)
                .turn(Math.toRadians(-90))
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(2)
                .strafeRight(10)
                .build();*/

                //on the outtake side
                .back(30)
                //.addTemporalMarker(() -> <SERVO>.setPower(1))
                .waitSeconds(1)
                //.addTemporalMarker(() -> <SERVO>.setPower(0))
                .forward(5)
                .turn(90)
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .addTemporalMarker(() -> slide.setPower(0))
                .addTemporalMarker(() -> latch.setPosition(0))
                .strafeRight(10)
                .build();


        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                /*on the intake side
                .back(30)
                .turn(Math.toRadians(-90))
                //DROP PURPLE
                .addTemporalMarker(() -> intakeWrist.setPosition(wristVal))
                .addTemporalMarker(() -> belt.setPower(-1))
                .waitSeconds(2)
                .addTemporalMarker(() -> intakeWrist.setPosition(0))
                .addTemporalMarker(() -> belt.setPower(0))
                .waitSeconds(2)
                //DROP PURPLE
                .strafeLeft(10)
                .turn(180)
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(2)
                .strafeRight(10)
                .build();*/

                //on the outtake side
                .back(30)
                .turn(90)
                //.addTemporalMarker(() -> <SERVO>.setPower(1))
                .waitSeconds(1)
                //.addTemporalMarker(() -> <SERVO>.setPower(0))
                .strafeRight(10)
                .back(30)
                .addTemporalMarker(() -> slide.setPower(0.75))
                .addTemporalMarker(() -> latch.setPosition(0.71))
                .waitSeconds(1)
                .addTemporalMarker(() -> slide.setPower(0))
                .addTemporalMarker(() -> latch.setPosition(0))
                .strafeRight(10)
                .build();


        while(!isStarted()){
            location = bluePropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        //drive.followTrajectorySequence(leftPurple);
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
