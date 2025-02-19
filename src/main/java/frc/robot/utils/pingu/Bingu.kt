package frc.robot.utils.pingu

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.utils.emu.Button

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
}
