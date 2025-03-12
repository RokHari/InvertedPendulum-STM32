# STM32 Inverted pendulum

Repository containing the STM32 code for balancing a rotary inverted pendulum Edukit.

[Please view this Web site for complete information on Edukit](https://sites.google.com/view/ucla-st-motor-control/home)

## Build

Build as a standard STM32 project using STM32CubeIDE. Upload the built binary to STM32 using STM32CubeProgrammer.

## Description

This is the STM32 part of a project for balancing an inverted pendulum using MARTe2 real-time framework. TODO: link to a "permanent" InvertedPendulum-MARTe2 repository.

The InvertedPendulum-STM32 exposes the high-level motor API via serial connection. The main loop waits for a command, handles the command, and sends a reply. See the code for details, but in short, the command consists of 10 bytes including command ID, device ID, and command parameters. The interpretation of command parameters bytes depends on the command ID. Once the command is processed, a 7 byte reply is return, containing command ID, device ID, error code, and return value. Again, return value is interpretation depends on the command.

There are also three extra commands, which are not part of the high-level motor API:
  - End real-time control (command ID 252) - When using real-time move (command ID 254), the high-level API might no longer work as expected. This command reset the software state of the motor, which makes the high-level API usable again. No response is returned.
  - Status redout (command ID 253) - returns the motor position, encoder/pendulum position, and motor state. Size of the response is 13 bytes.
  - Real-time move (command ID 254) - changes the speed of the motor using the provided acceleration and period, without stopping the motor. No response is returned.