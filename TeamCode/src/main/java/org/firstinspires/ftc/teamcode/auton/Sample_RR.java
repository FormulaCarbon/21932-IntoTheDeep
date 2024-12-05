package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import java.util.HashMap;

@TeleOp(name = "Sample Cycle", group = "Sensor")
public class Sample_RR extends LinearOpMode {
    SparkFunOTOS myOtos;
    public HashMap<String, String> deviceConf = new HashMap<String, String>();

    public static int tickChange = 100, pos = 150;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Map HashMap
        deviceConf.put("leftPivot",       "leftPivot");
        deviceConf.put("rightPivot",      "rightPivot");
        deviceConf.put("leftExtension",   "leftExtension");
        deviceConf.put("rightExtension",  "rightExtension");
        deviceConf.put("wrist",           "pivot");
        deviceConf.put("reset",           "reset");

        Claw claw = new Claw(hardwareMap);

        Pivot pivot = new Pivot(hardwareMap, deviceConf);

        Extension extension = new Extension(hardwareMap, deviceConf);

        Wrist wrist = new Wrist(hardwareMap, deviceConf);

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        Pose2d startPos = new Pose2d(42, 64, Math.PI/2);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, startPos);

        TrajectoryActionBuilder bucket0 = drive.actionBuilder(startPos)
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(56,56, 5*Math.PI/4), 0);

        TrajectoryActionBuilder wait1 = drive.actionBuilder(startPos)
                .waitSeconds(1);


        TrajectoryActionBuilder block1 = bucket0.fresh()
                .setTangent(5*Math.PI/4)
                .splineToLinearHeading(new Pose2d(48, 40, 3*Math.PI/2), 3*Math.PI/2);

        TrajectoryActionBuilder bucket1 = block1.fresh()
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(56,56, 5*Math.PI/4), Math.PI/4);


        TrajectoryActionBuilder block2 = bucket1.fresh()
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(58, 40, 3*Math.PI/2), 3*Math.PI/2);

        TrajectoryActionBuilder bucket2 = block2.fresh()
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(56,56, 5*Math.PI/4), Math.PI/2);

        TrajectoryActionBuilder block3 = bucket2.fresh()
                .setTangent(3*Math.PI/2)
                .splineToLinearHeading(new Pose2d(60, 40, 5*Math.PI/3), 5*Math.PI/3);

        TrajectoryActionBuilder bucket3 = block3.fresh()
                .setTangent(2*Math.PI/3)
                .splineToLinearHeading(new Pose2d(56,56, 5*Math.PI/4), Math.PI/2);

        TrajectoryActionBuilder park = bucket3.fresh()
                .strafeTo(new Vector2d(50, 50));

        Thread update = new Thread(()->updateAll(pivot, extension, wrist));
        claw.directSet(0.56);

        // Wait for the start button to be pressed
        waitForStart();

        wrist.setPos("High Basket");
        wrist.update();

        while (!pivot.checkReset() && opModeIsActive())
        {
            if (pivot.checkReset())
            {
                pivot.applyPower(0);
            }
            pivot.setDirectPos(pivot.getPos() - tickChange);
            telemetry.addData("pos", pivot.getPos());
            telemetry.addData("target", pivot.getPos() - 10);
            telemetry.addData("error", pivot.getError());
            telemetry.update();
            pivot.update();
            if (pivot.checkReset())
            {
                pivot.applyPower(0);
            }
        }
        pivot.checkReset();
        update.start();

        setBucket(bucket0.build(), pivot, extension, wrist, claw);
        sleep(500);

        getBlock(block1.build(), pivot, extension, wrist, claw);
        setBucket(bucket1.build(), pivot, extension, wrist, claw);
        sleep(500);

        getBlock(block2.build(), pivot, extension, wrist, claw);
        setBucket(bucket2.build(), pivot, extension, wrist, claw);
        sleep(500);

        getBlock(block3.build(), pivot, extension, wrist, claw);
        setBucket(bucket3.build(), pivot, extension, wrist, claw);
        sleep(500);

        extension.setPos("Idle");
        sleep(500);
        pivot.setPos("Intake");
        pivot.setKP("Idle");
        Actions.runBlocking(park.build());

    }
    public void sleep(int t) {
        try {
            Thread.sleep(t); // Wait for 1 millisecond
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted status
            // Optionally, log or handle the interruption
        }
    }
    public void updateAll(Pivot pivot, Extension extension, Wrist wrist) {
        while (opModeIsActive())
        {

            pivot.setKP(extension.getTarget());
            pivot.update();
            extension.update();
            wrist.update();
            telemetry.addData("pos", pivot.getPos());
            telemetry.addData("target", pivot.getPos() - 10);
            telemetry.addData("error", pivot.getError());
            telemetry.update();
        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public void setBucket(Action trajectory , Pivot pivot, Extension extension, Wrist wrist, Claw claw) {
        pivot.setPos("High Basket");
        pivot.setKP("High Basket");
        sleep(500);
        extension.setPos("High Basket");
        wrist.setPos("Sample Intake");

        Actions.runBlocking(trajectory);
        wrist.setPos("High Basket");
        claw.directSet(0.26);
        sleep(750);
        wrist.setPos("Sample Intake");

    }

    public void getBlock(Action trajectory , Pivot pivot, Extension extension, Wrist wrist, Claw claw) {
        extension.setPos("Idle");
        sleep(500);
        pivot.setPos("Idle");
        pivot.setKP("Idle");
        Actions.runBlocking(trajectory);

        pivot.setPos("Intake");
        pivot.setKP("Intake");
        wrist.setPos("Intake");
        sleep(500);
        extension.setPos("Intake");
        claw.directSet(0.56);
        sleep(500);
        wrist.setPos("Sample Intake");
        extension.setPos("Idle");
        pivot.setPos("Idle");
        pivot.setKP("Idle");
    }

}
