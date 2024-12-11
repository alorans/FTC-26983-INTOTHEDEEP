package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.command.CommandOpMode;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.control.PIDController;
import com.amarcolini.joos.gamepad.Toggleable;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleRobot;

@TeleOp(group = "TriSwerve")
public class TriSwerveOpmode extends CommandOpMode {
    @Register
    private TriRobot robot;

    // Settings
    private boolean fieldCentric = true;
    private boolean headingLock = false;
    public PIDCoefficients headingCoeffs = new PIDCoefficients(1.0, 0.0, 0.1);
    private final PIDController headingController = new PIDController(headingCoeffs);

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

        // Drive
        schedule(true, () -> {
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

            telem.drawRobot(robot.drive.getLocalizer().getPoseEstimate(), "blue");
            telem.addData("FF: ", 10*robot.pivot.getCurrentPos());
        });
    }
}
