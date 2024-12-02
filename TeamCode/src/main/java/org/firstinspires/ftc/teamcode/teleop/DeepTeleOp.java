/*package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.shhh.subsystems.Arm;
import org.firstinspires.ftc.teamcode.testing.subsystems.Drive;
import org.firstinspires.ftc.teamcode.testing.subsystems.MainArm;

@TeleOp(name = "DeepTeleOp")
public class DeepTeleOp extends OpMode {

    public Drive drive; public MainArm arm; //public Arm deposit;

    @Override
    public void init() {
        drive = new Drive(hardwareMap, "frontLeftMotor", "backLeftMotor", "frontRightMotor", "backRightMotor");
        arm = new MainArm(hardwareMap, "rightPivot","leftPivot","leftExtension", "rightExtension", "reset", 10, false);
        //deposit = new Arm(hardwareMap);

        arm.setMainState(MainArm.State.intake);
    }

    @Override
    public void loop() {
        drive.setFlags(gamepad1.left_bumper, gamepad1.right_bumper);
        drive.setXYZ(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

        //deposit.process(gamepad2.dpad_up, gamepad2.dpad_down);

        arm.setLmi(gamepad2.right_stick_x);
        if (leftTriggerBoolean() && !rightTriggerBoolean()) {
            arm.setLtg(0);
        } else if (rightTriggerBoolean() && !leftTriggerBoolean()) {
            arm.setLtg(3000);
        }

        increment(gamepad2.b, gamepad2.x);
        telemetry.addData("Arm Statse", arm.getState());
        drive.update();
        //deposit.update(gamepad2.left_bumper, gamepad2.right_bumper);
        arm.update( gamepad1.y);

        telemetry.addData("Drive Power %", drive.getMultiplier() * 100);
        telemetry.addData("Arm State", arm.getlState());
        telemetry.addData("Increment", incr);
        telemetry.addData("angle", arm.getAngle());
        telemetry.addData("angle", arm.getTAngle());
        telemetry.addData("power", arm.getPower());
        telemetry.update();
    }

    int incr = 0, lastIncr = 0; boolean isPr = false;

    public void increment(boolean upFlag, boolean downFlag) {
        if (downFlag && !isPr && incr > 0) {
            incr -= 1;
            isPr = true;
        } else if (upFlag && !isPr && incr < 3) {
            incr += 1;
            isPr = true;
        } else if (!downFlag && !upFlag) {
            isPr = false;
        }

        if (incr == 1) {
            arm.setMainState(MainArm.State.idle);
        } else if (incr == 2) {
            arm.setMainState(MainArm.State.basket);
        } else if (incr == 3) {
            arm.setMainState(MainArm.State.backpickup);
        } else {
            arm.setMainState(MainArm.State.intake);
        }
    }

    public boolean leftTriggerBoolean() {
        return gamepad2.left_trigger > 0;
    }

    public boolean rightTriggerBoolean() {
        return gamepad2.right_trigger > 0;
    }
}*/