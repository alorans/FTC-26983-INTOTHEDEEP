package org.firstinspires.ftc.teamcode.TriSwerve;

import androidx.annotation.NonNull;

import com.amarcolini.joos.command.BasicCommand;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.SelectCommand;
import com.amarcolini.joos.command.TimeCommand;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.control.PIDController;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.gamepad.Toggleable;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleRobot;

import java.sql.Array;
import java.util.Set;

@TeleOp(group = "TriSwerve")
public class TriSwerveOpmode extends CommandOpMode {
    @Register
    private TriRobot robot;

    // Settings
    private boolean fieldCentric = false;
    private boolean headingLock = false;
    public PIDCoefficients headingCoeffs = new PIDCoefficients(1.0, 0.0, 0.1);
    private final PIDController headingController = new PIDController(headingCoeffs);

    private double armPos = 0.0;

    @Override
    public void preInit() {
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d());

        // Heading controller PID
        headingController.reset();
        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setOutputBounds(-0.5, 0.5);

        // Map A and B buttons to settings
        map(gamepad().p1.a::isJustActivated, () -> fieldCentric = !fieldCentric);
        map(gamepad().p1.b::isJustActivated, () -> headingLock = !headingLock);
        // Drivetrain command
        Command.of(() -> {
            Vector2d leftStick = gamepad().p1.getLeftStick();
            Vector2d rightStick = gamepad().p1.getRightStick();
            Angle currentHeading = robot.headingSensor.getAngle();
            Pose2d drivePower =
                    new Pose2d(
                            new Vector2d(
                                    -leftStick.y,
                                    -leftStick.x
                            ),
                            Angle.rad(-rightStick.x)
                    );
            // Handle field centric and heading lock
            if (fieldCentric) {
                drivePower = new Pose2d(
                        drivePower.vec().rotated(currentHeading.unaryMinus()),
                        drivePower.heading
                );
            }
            if (headingLock) {
                drivePower = new Pose2d(
                        drivePower.vec(),
                        Angle.rad(headingController.update(currentHeading.radians()))
                );
            }
            robot.drive.setDrivePower(drivePower);

            // Use telemetry to draw robot on FTC Dashboard
            telem.drawRobot(robot.drive.getLocalizer().getPoseEstimate(), "blue");

        // Set requirements, run forever, and schedule
        }).requires(robot.drive).repeatForever().schedule();

        // Arm assembly command
        /*Command.of(() -> {
            // Handle pivot
            Vector2d leftStick = gamepad().p2.getLeftStick();
            robot.arm.slideTarget += 1;

            // Handle slide
            Vector2d rightStick = gamepad().p2.getRightStick();
            robot.arm.runSlide(rightStick.y);

        // Set requirements, run forever, and schedule
        }).requires(robot.arm).repeatForever().schedule();*/

        map(gamepad().p2.dpad_up::isActive, () -> armPos += 2);
        map(gamepad().p2.dpad_down::isJustActivated, () -> armPos -= 2);
        Command.of(() -> robot.arm.setPivotTarget(armPos)).requires(robot.arm).repeatForever().schedule();
    }
}
