# TTJW Firmware Refactor Notes

## Intent
Reduce coupling and make the firmware easier to navigate without changing the established hardware configuration or control flow unnecessarily.

## Module boundaries
- **Board/application entry:** `main.c`
- **Shared state:** `ttjw_state.c/.h`
- **Settings/constants:** `ttjw_config.h`
- **Sensor interface:** `ads1220.c/.h`
- **Sensor mathematics:** `pt100.c/.h`
- **Control output:** `heater_control.c/.h`
- **User input:** `keypad.c/.h`
- **Display:** `oled_sh1106.c/.h`

## Why not make every variable global?
State that genuinely crosses module boundaries is exposed in `ttjw_state.h`. Internal PID history, keypad scan/debounce state, and the OLED framebuffer remain `static` inside their modules. This reduces accidental cross-module writes and makes the owner of each piece of state explicit.

## First validation checklist
- [ ] Clean/build in STM32CubeIDE using the ARM GCC toolchain.
- [ ] Confirm no undefined references after CubeIDE discovers the new `.c` files.
- [ ] Verify OLED title, SV, PV, ALM, and OUT layout.
- [ ] Verify S1/S3 change SV by 1.0 °C and clamp at −199.9/500.0 °C.
- [ ] Verify AL-1 changes in 0.1 °C steps and 0.0 disables it.
- [ ] Verify OUT indicator follows TIM1 CNT/CCR phase at 0%, 10%, 50%, and 100%.
- [ ] Verify heater output turns off on ADC timeout and at/above SV.
- [ ] Verify keypad navigation and hold-repeat.
- [ ] Verify measured PT100 conversion against a calibrated reference over the intended temperature range.
- [ ] Test with heater power disconnected before any powered heater trial.

## Known issue retained from the source
The current `ResistanceToTemperature()` implementation clamps negative temperatures to 0 °C and uses the positive-temperature equation only. It was retained rather than silently changing sensor math during a structural refactor. Accurate sub-zero PV support needs a deliberate CVD negative-region implementation and bench validation.
