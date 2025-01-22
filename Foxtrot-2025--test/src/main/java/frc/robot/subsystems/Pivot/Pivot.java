package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Pivot extends SubsystemBase {
    private final PivotIO pivot;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Pose3d pivotPose = new Pose3d();


    private double targetAngle = 0.0; // Internal target angle in radians
    private final LinearFilter smoothingFilter = LinearFilter.singlePoleIIR(0.1, 0.02); // Smoother transitions

    public static Pivot create() {
        PivotIO pivotIO = Robot.isReal() ? new RealPivot(62) : new NoPivot();
        return new Pivot(pivotIO);
    }

    public Pivot(PivotIO pivot) {
        this.pivot = pivot;
    }

    @Override
    public void periodic() {
        pivot.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        updatePivotPose();
        Logger.recordOutput("Pivot/Angle", pivotPose);

        double smoothedAngle = smoothingFilter.calculate(getAngle());

        SmartDashboard.putNumber("Aci", Math.toDegrees(smoothedAngle));
    }

    private void updatePivotPose() {
        double angle = getAngle();
        double x = angle * 0.1;
        double z = -angle * 0.005;
        double yRotation = -angle;


        pivotPose = pivotPose.interpolate(
            new Pose3d(x, 0, z, new Rotation3d(0, yRotation, 0)),
            0.1
        );
    }

    public void setDesiredAngle(double angle) {
        targetAngle = Math.toRadians(angle);
        pivot.setDesiredAngle(targetAngle);
    }

    public void stop() {
        pivot.stop();
        targetAngle = 0.0;
    }

    public double getAngle() {
        return pivot.getAngle();
    }
}
