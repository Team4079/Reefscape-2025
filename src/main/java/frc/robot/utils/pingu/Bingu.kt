package frc.robot.utils.pingu

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Kommand.cmd
import frc.robot.utils.emu.Button
import java.util.concurrent.atomic.AtomicReference
import java.util.function.Consumer

/**
 * Type alias for a map of buttons to joystick buttons.
 */
typealias ButtonMap = Map<Button, JoystickButton>

/**
 * Class that manages bindings between buttons and commands.
 *
 * @property buttonMap A map of buttons to joystick buttons.
 */
class Bingu(
    private val buttonMap: ButtonMap,
) {
    /**
     * Binds a button to a command.
     *
     * @param button The button to bind.
     * @param command The command to execute when the button is pressed.
     */
    fun bind(
        button: Button,
        command: Command,
    ) = apply {
        buttonMap[button]?.onTrue(command) ?: run {
            println("Missing binding for button $button in list $buttonMap")
        }
    }

    /**
     * Binds a button to a command.
     *
     * @param button The button to bind.
     * @param runnable The runnable to execute when the button is pressed.
     */
    fun bind(
        button: Button,
        runnable: Runnable,
    ) = apply {
        buttonMap[button]?.onTrue(runnable.cmd) ?: run {
            println("Missing binding for button $button in list $buttonMap")
        }
    }

    /**
     * Binds a button to a command.
     *
     * @param button The button to bind.
     * @param runnable The runnable to execute when the button is pressed.
     */
    fun bind(
        button: Button,
        consumer: Consumer<Any>,
        parameter: AtomicReference<Any>,
    ) = apply {
        buttonMap[button]?.onTrue(consumer.acceptCmd(parameter)) ?: run {
            println("Missing binding for button $button in list $buttonMap")
        }
    }
}

/**
 * Extension property to convert a `Runnable` to an `InstantCommand`.
 */
val Runnable.cmd: InstantCommand
    get() = cmd { run() }

/**
 * Extension function to convert a `Consumer` with a parameter to an `InstantCommand`.
 *
 * @param T The type of the parameter.
 * @param parameter The `AtomicReference` holding the parameter to be consumed.
 * @return An `InstantCommand` that executes the consumer with the parameter.
 */
fun <T> Consumer<T>.acceptCmd(parameter: AtomicReference<T>): InstantCommand = cmd { accept(parameter.get()) }
