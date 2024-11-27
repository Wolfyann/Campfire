# MSP430 Campfire Simulation

This project simulates a small campfire effect using LEDs controlled by a Texas Instruments MSP430G2553 microcontroller. The simulation uses PWM (Pulse Width Modulation) to vary the brightness of the LEDs, creating a realistic flickering effect.

## Features
- **Simulated Campfire Effect**: LEDs flicker to mimic the appearance of a small flame.
- **PWM Control**: Uses TimerA for precise control of LED brightness.
- **Day/Night Detection**: (Optional) Can integrate an LDR (light-dependent resistor) for ambient light detection.

## Components
- **Microcontroller**: MSP430G2553
- **LEDs**:
  - Yellow LED (P2.1)
  - Red LED (P2.5)
  - Optional Spark Effect (P1.2)
- **Button**: P1.3 for manual mode toggle
- **Optional LDR**: P1.7 for ambient light sensing

## Usage
1. **Setup Hardware**: Connect LEDs to the specified pins on the MSP430G2553.
2. **Compile and Flash**: Use Code Composer Studio (CCS) to build and upload the code.
3. **Run**: Power the MSP430G2553 and observe the flickering LEDs.

## Development
- **Compiler**: CCS V12.x or higher
- **Language**: C
- **Platform**: MSP430G2553

## Code Overview
- **Timers**: Configured for PWM signal generation.
- **Interrupts**: Handle LED intensity modulation and button presses.
- **ADC**: Reads from the LDR (optional) to adjust the effect based on ambient light.

## Contributing
Feel free to fork this repository and suggest improvements.

## License
This project is licensed under the MIT License.
