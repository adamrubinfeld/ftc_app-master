package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

public class TeleOp extends OpMode {

    Robot robot = new Robot();

    @Override
    public void init() {
        robot.Initialize(hardwareMap);
        gamepad1.setJoystickDeadzone(0.2f);
        gamepad2.setJoystickDeadzone(0.2f);
    }

    @Override
    public void loop() {
        robot.FieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_trigger-gamepad1.left_trigger);

        telemetry.addData("angle", robot.GetAngle());
        telemetry.update();
    }
}
