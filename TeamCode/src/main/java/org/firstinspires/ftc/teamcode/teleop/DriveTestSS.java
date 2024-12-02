package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

import java.util.HashMap;
import java.util.Map;

@TeleOp
@Config
public class DriveTestSS extends LinearOpMode {

    public HashMap<String, String> deviceConf = new HashMap<String, String>();

    public static int tickChange = 50;

    public static double clawOpen = 0.21, clawClose = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot Config Mapping
        deviceConf.put("frontLeft",       "frontLeftMotor");
        deviceConf.put("backLeft",        "backLeftMotor");
        deviceConf.put("frontRight",      "frontRightMotor");
        deviceConf.put("backRight",       "backRightMotor");
        deviceConf.put("leftPivot",       "leftPivot");
        deviceConf.put("rightPivot",      "rightPivot");
        deviceConf.put("leftExtension",   "leftExtension");
        deviceConf.put("rightExtension",  "rightExtension");
        deviceConf.put("wrist",           "pivot");
        deviceConf.put("reset",           "reset");

        Claw claw = new Claw(hardwareMap);
        Drive drive = new Drive(hardwareMap, deviceConf);

        Pivot pivot = new Pivot(hardwareMap, deviceConf);

        Extension extension = new Extension(hardwareMap, deviceConf);

        Wrist wrist = new Wrist(hardwareMap, deviceConf);

        boolean pivotReady, wristReady, extensionReady, swapReady, cycleReady, clawReady;
        boolean wristManual = false, extensionManual = false, pivotManual = false;


        String sequence = "sample";

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pivotReady = wristReady = extensionReady = swapReady = cycleReady = clawReady = true;
            drive.getXYZ(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if (gamepad2.right_bumper && swapReady) {
                sequence = "specimen";
                incr = 0;
                swapReady = false;
            }
            if (gamepad2.left_bumper && swapReady) {
                sequence = "sample";
                incr = 0;
                swapReady = false;
            }
            if (gamepad1.x && wristReady ) {
                wristManual = true;
                wrist.setPos("Intake");
                wristReady = false;
            }
            else if (gamepad1.y && wristReady) {
                wristManual = true;
                wrist.setPos("High Basket");
                wristReady = false;
            }

            if (gamepad2.x && extensionReady) {
                extensionManual = true;
                extension.setPos("Low Specimen");
                extensionReady  = false;
            }
            else if ((gamepad2.y || gamepad1.dpad_up) && extensionReady) {
                extensionManual = true;
                extension.setPos("High Specimen");
                extensionReady  = false;
            }
            else if (gamepad2.a && extensionReady) {
                extensionManual = true;
                extension.setPos("Low Basket");
                extensionReady  = false;
            }
            else if (gamepad2.b && extensionReady) {
                extensionManual = true;
                extension.setPos("High Basket");
                extensionReady  = false;
            }
            else if (gamepad1.dpad_down && extensionReady) {
                extensionManual = true;
                extension.setPos("Idle");
                extensionReady  = false;
            }

            if (gamepad2.dpad_down && pivotReady) {
                pivotManual = true;
                pivot.checkReset();
                pivotReady  = false;
            }
            if (gamepad1.dpad_right && pivotReady) {
                pivotManual = true;
                pivot.setDirectPos(pivot.getPos()-tickChange);
                pivotReady  = false;
            }

            if (gamepad1.dpad_left && pivotReady) {
                pivotManual = true;
                pivot.setDirectPos(pivot.getPos()+tickChange);
                pivotReady  = false;
            }
            if (gamepad2.right_bumper || gamepad2.left_bumper || gamepad1.right_bumper || gamepad1.left_bumper)
            {
                wristManual = false;
                extensionManual = pivotManual =  false;

            }






            target = increment(gamepad1.right_bumper, gamepad1.left_bumper, sequence);

            if (!extensionManual)
            {
                extension.setPos(target);
            }
            /*if (!extension.isBusy())
            {

            }

            if (!extensionManual && !pivot.isBusy())
            {
                extension.setPos(target);
            }*/
            if (!wristManual)
            {
                wrist.setPos(target);
            }
            if (!pivotManual)
            {
                pivot.setPos(target);
            }


            pivot.setKP(extension.getTarget());
            //pivot.checkReset();

            drive.update();
            pivot.update();
            extension.update();
            wrist.update();
            claw.update(gamepad1.a);

            telemetry.addData("incr", incr);
            telemetry.addData("lastincr", lastIncr);
            telemetry.addData("target", target);
            telemetry.addData("wrist", wristManual);
            telemetry.addData("e", extensionManual);
            telemetry.addData("error", pivot.getError());
            telemetry.addData("r", pivot.getTicks());
            telemetry.addData("kp", pivot.getKP());

            telemetry.update();


        }
    }
    int incr = 0, lastIncr = 0; boolean isPr = false; String target;

    public String increment(boolean upFlag, boolean downFlag, String sequence) {
        if (downFlag && !isPr && incr > 0) {
            incr -= 1;
            isPr = true;
        } else if (upFlag && !isPr ) {
            incr += 1;
            isPr = true;
        } else if (!downFlag && !upFlag) {
            isPr = false;
        }
        if (incr > 3)
        {
            incr = 0;
        }

        target = getPositions(sequence, incr);
        //pivot.setPos(target);
        //extension.setPos(target);
        //wrist.setPos(target);
        // claw
        return target;
    }
    public String getPositions(String sequence, int state) {
        if (sequence == "sample") {

            switch (state) {
                case 0:
                    return "Idle";

                case 1:
                    return "Sample Intake";

                case 2:
                    return "Idle";

                case 3:
                    return "High Basket";
            }
        }

        if (sequence == "specimen") {

            switch (state) {
                case 0:
                    return "Idle";

                case 1:
                    return "Specimen Intake";

                case 2:
                    return "Idle";

                case 3:
                    return "High Specimen";
            }
        }

        return "Idle";
    }
}