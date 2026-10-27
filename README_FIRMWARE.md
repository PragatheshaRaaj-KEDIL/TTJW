# TTJW Firmware — Modular STM32CubeIDE Project

Target MCU: **STM32L476RG**  
Main peripherals: **ADS1220 + PT100**, **SH1106 OLED over I2C**, **TIM1 CH1 heater PWM**, and a 2×2 matrix keypad.

## What changed

The original application logic was consolidated into `main.c`. The refactor separates application behavior by responsibility while retaining the CubeMX board initialization and existing HAL support files.

### Source modules

| File | Responsibility |
|---|---|
| `Core/Src/main.c` | Application startup/main loop, peripheral initialization, clock configuration, GPIO initialization, and fatal error handler |
| `Core/Src/ttjw_state.c` / `Core/Inc/ttjw_state.h` | Shared application state and operator-adjustable values |
| `Core/Src/ads1220.c` / `Core/Inc/ads1220.h` | ADS1220 reset, register setup, SPI transfer, and conversion read |
| `Core/Src/pt100.c` / `Core/Inc/pt100.h` | Existing resistance equation and resistance-to-temperature conversion |
| `Core/Src/heater_control.c` / `Core/Inc/heater_control.h` | TIM1 PWM output and PID state/control calculations |
| `Core/Src/keypad.c` / `Core/Inc/keypad.h` | Matrix scan, debounce, S1/S3 hold-repeat, and screen navigation |
| `Core/Src/oled_sh1106.c` / `Core/Inc/oled_sh1106.h` | SH1106 I2C driver, framebuffer, glyph rendering, SV/PV screen, ALM and OUT indicators |
| `Core/Inc/ttjw_config.h` | Centralized application ranges, keypad timing, ADC timeout, and PWM constants |

`main.h` remains the CubeMX board header for HAL types, pin names, and board-level declarations. It is not used as a container for application logic.

## Current application settings retained

- Initial setpoint (`SV`): **37.0 °C**
- Setpoint adjustment range: **−199.9 to 500.0 °C**
- S1 increments SV by **1.0 °C per press**; S3 decrements it by **1.0 °C per press**
- AL-1 starts at 0.0; 0.0 disables the alarm. Positive AL-1 activates the steady ALM indicator when valid PV is at or above the threshold.
- OUT indicator follows the TIM1 PWM phase: it is filled during the commanded ON phase and outlined during the OFF phase. At 0% it is OFF; at 100% it is ON.
- Existing PWM demand remains quantized in 10% steps and uses the existing 2-second timer window.
- Existing PID settings start at P = 30.0, I = 0.0, D = 0.0.
- Existing keypad screen navigation and ERROR offset remain in place.

## Important limitations and verification notes

1. **Negative PV is not yet supported by the current PT100 conversion path.** `ResistanceToTemperature()` retains the existing implementation, which clamps negative results to 0 °C and uses the positive-temperature Callendar–Van Dusen equation. Therefore, allowing a negative SV does not mean the measured PV can accurately represent temperatures below 0 °C. This requires a separately reviewed negative-temperature conversion implementation and sensor validation.
2. The OUT icon represents the MCU's commanded timer phase. It is not a measurement of heater current, voltage, or delivered heat.
3. The ALM display is a software indication, not an independent over-temperature safety mechanism.
4. The original resistance equation, ADC configuration, timer setup, PID behavior, and keypad behavior were retained rather than silently redesigned.
5. The available environment did not include `arm-none-eabi-gcc` or STM32CubeIDE. The newly split application source files passed host-GCC C syntax checks with the project's STM32 headers; this is **not** a full STM32 target build or hardware test. The unrelated `sysmem.c` syntax check under host GCC reports standard-library type errors because the host compiler is not the project's ARM toolchain.
6. Before heater connection, build with the STM32CubeIDE ARM toolchain, inspect warnings, and test with the heater disconnected. Verify sensor readings, alarm threshold behavior, PWM duty and phase, and fault shutoff. This is temperature-control equipment intended for an IVF-related application; software refactoring alone does not establish clinical or safety suitability.

## Open and build

1. Extract the ZIP.
2. In STM32CubeIDE, use **File → Import → Existing Projects into Workspace** and select the extracted `TTJW` folder.
3. Confirm the selected target is STM32L476RG and the project includes the new `.c` files under `Core/Src`.
4. Run **Project → Clean**, then **Build Project**. This allows CubeIDE to regenerate its build file lists for the new modules.
5. Review build errors and warnings before programming the board.

## Design convention

- Keep shared state limited to values that multiple modules genuinely use.
- Keep PID history, OLED framebuffer, and keypad scan/debounce state private to their owning modules.
- Add new public functions to the corresponding module header.
- Put application-level numeric settings in `ttjw_config.h`.
- Do not move CubeMX-generated pin definitions or HAL peripheral initialization into application modules without a clear reason.
