package org.firstinspires.ftc.teamcode.TriSwerve;

import androidx.annotation.NonNull;
import com.amarcolini.joos.dashboard.JoosConfig;

@Deprecated
public class PIDFCoefficients {
    public double d;
    public double f;
    public double i;
    public double p;

    public PIDFCoefficients() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    public PIDFCoefficients(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    @NonNull
    public String toString() {
        return "{ p=" + p + ", i=" + i + ", d=" + d + ", f=" + f + " }";
    }
}