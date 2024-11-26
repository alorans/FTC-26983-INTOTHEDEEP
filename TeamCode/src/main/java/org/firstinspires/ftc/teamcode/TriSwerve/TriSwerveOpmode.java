package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp(group = "TriSwerve")
public class TriSwerveOpmode extends CommandOpMode {
    @Register
    private TriRobot robot;

    @Override
    public void preInit() {
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d());

        schedule(true, () -> {
            Vector2d leftStick = gamepad().p1.getLeftStick();
            Vector2d rightStick = gamepad().p1.getRightStick();
            robot.drive.setDrivePower(
                    new Pose2d(
                            -leftStick.y,
                            -leftStick.x,
                            Angle.rad(-rightStick.x)
                    )
            );

            telem.drawRobot(robot.drive.getLocalizer().getPoseEstimate(), "blue");
        });
    }
}
