package frc.robot.commands;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

import static frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters.*;


public class AlignLeft extends Command {
    private double yaw;
    private PIDController rotationalController;

    public AlignLeft() {
        addRequirements(SwerveSubsystem.getInstance(), PhotonvisionSubsystem.getInstance());
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        yaw = PhotonvisionSubsystem.getInstance().getYaw();
        rotationalController = new PIDController(ROTATIONAL_PID.getP(), ROTATIONAL_PID.getI(), ROTATIONAL_PID.getD());
        rotationalController.setTolerance(1.5);
        rotationalController.setSetpoint(0);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        yaw = PhotonvisionSubsystem.getInstance().getYaw();
        if (PhotonvisionSubsystem.getInstance().hasTag()) {
            SwerveSubsystem.getInstance().setDriveSpeeds(0, 0, rotationalController.calculate(yaw, 0), false);
        } else {
            SwerveSubsystem.getInstance().stop();
        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stop();
    }
}
