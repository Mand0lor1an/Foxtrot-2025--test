package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Roller.Rollers;
import frc.robot.subsystems.Roller.Rollers.Rollerstate;
import frc.robot.subsystems.Roller.Extender.Extender;
import frc.robot.subsystems.Roller.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class SourceIntake extends SubsystemBase {
    private final BeamBreak beamBreak;
    private final Pivot pivot;
    private final Rollers rollers;
    private final Shooter shooter;


    private boolean actionActive = false;
    private double stopTime = 0.0;

    public SourceIntake(Pivot pivot, Rollers rollers, Shooter shooter, BeamBreak beamBreak) {
        this.pivot = pivot;
        this.rollers = rollers;
        this.shooter = shooter;
        this.beamBreak = beamBreak;
    }

    @Override
    public void periodic() {
        if (actionActive) {
            if (!beamBreak.b_upper) {
                stopTime = Timer.getFPGATimestamp() + 1.0;
            }


            if (Timer.getFPGATimestamp() >= stopTime) {
                stopAllActions();
            }
        }
    }

    public void startAction() {
        actionActive = true;

        pivot.setDesiredAngle(90);
        rollers.state = Rollerstate.EJECTING;
        shooter.state = Shooter.ShooterState.SOURCE_I;
    }

    public void stopAllActions() {
        actionActive = false;

        pivot.stop();
        rollers.state = Rollerstate.IDLE;
        shooter.state = Shooter.ShooterState.IDLE;
    }
}