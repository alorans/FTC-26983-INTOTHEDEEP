package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.PIDSwerveModule;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.hardware.CRServo;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.localization.AngleSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AxonAngleSensor;
import org.jetbrains.annotations.NotNull;

@JoosConfig
public class TriSwerveModule extends PIDSwerveModule {
    public static final PIDCoefficients coeffs = new PIDCoefficients(0.7, 0.0, 0.02);
    public static double servoPositionTolerance = 0.05;
    public final AxonAngleSensor moduleOrientationSensor;
    public final Motor motor;
    public final CRServo servo;

    public TriSwerveModule(
            AxonAngleSensor moduleOrientationSensor,
            Motor motor,
            CRServo servo
    ) {
        super(coeffs);
        this.moduleOrientationSensor = moduleOrientationSensor;
        this.motor = motor;
        this.servo = servo;
    }

    public TriSwerveModule(
            HardwareMap hMap,
            String motorId,
            String servoId,
            String axonAngleSensorId,
            Angle axonAngleOffset
    ) {
        super(coeffs);
        this.moduleOrientationSensor = new AxonAngleSensor(
                hMap.get(AnalogInput.class, axonAngleSensorId),
                axonAngleOffset, false
        );
        this.motor = new Motor(hMap, motorId, Motor.Type.GOBILDA_MATRIX);
        this.servo = new CRServo(hMap, servoId);
    }

    @Override
    protected void setCorrectedDrivePower(double v) {
        motor.setPower(v);
    }

    @Override
    protected void setCorrectedWheelVelocity(double v, double v1) {
        motor.setDistanceVelocity(v, v1);
    }

    @Override
    public void setModulePower(double v) {
        servo.setPower(v);
    }

    @NotNull
    @Override
    public Angle getModuleOrientation() {
        return moduleOrientationSensor.getAngle();
    }

    @Override
    public double getWheelPosition() {
        return motor.getDistance();
    }
}