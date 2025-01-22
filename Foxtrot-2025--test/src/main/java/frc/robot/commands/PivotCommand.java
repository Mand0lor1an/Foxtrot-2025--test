package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Pivot.Pivot;

public class PivotCommand extends Command {
    private final Pivot pivot;
    private final edu.wpi.first.wpilibj2.command.button.CommandXboxController controller;

    private static final double DEADBAND = 0.1; // Joystick deadband for precision control
    private static final double MAX_ANGLE = 90.0; // Maximum angle in degrees
    private static final double MIN_ANGLE = -90.0; // Minimum angle in degrees

    public PivotCommand(Pivot pivot, edu.wpi.first.wpilibj2.command.button.CommandXboxController controller) {
        this.pivot = pivot;
        this.controller = controller;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.stop();
    }

    @Override
    public void execute() {
        double joystickInput = -controller.getLeftY();


        if (Math.abs(joystickInput) < DEADBAND) {
            joystickInput = 0.0;
        }

        double desiredAngle = joystickInput * (MAX_ANGLE - MIN_ANGLE) / 2.0;

        desiredAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, desiredAngle));

        pivot.setDesiredAngle(desiredAngle);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
