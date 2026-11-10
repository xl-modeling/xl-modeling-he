# LDO 1.1

SystemC-AMS model of the capless LDO used for the seven-item chiplet model acceptance test.

## Build and reproduce the acceptance test

Use C++17, SystemC 2.3.4 and SystemC-AMS 2.3.4. The nlohmann/json 3.12.0 header and its embedded MIT license are bundled in `include/nlohmann`.

From this model directory on Linux or WSL, run

```sh
export SYSTEMC_HOME=/path/to/systemc-2.3.4
export SYSTEMC_AMS_HOME=/path/to/systemc-ams-2.3.4
cmake -S test -B test/build -DCMAKE_BUILD_TYPE=Release
cmake --build test/build --parallel 2
python3 test/run.py
```

The seven checks use VDD=1.2 V, VREF=1 V, AVDD=2.5 V, IBIAS=10 uA and 27 degrees C. They measure the 1 mA output, 100 mA output, load regulation, no-load quiescent current and PSRR at 1 kHz, 100 kHz and 1 MHz. Bounds are exactly 95%-105% of the stored typicals. PSRR uses SystemC-AMS AC analysis after settling the operating point. Logs, waveforms and results are written to `test/build/results`.

## Integration

Keep `json/LDO.json` and `json/LDO_grid.txt` together. The data table is required at runtime and is resolved relative to the JSON file. Compile `source/LDO.cpp` with both model headers in `include/`. `test/model_tb.cpp` demonstrates binding every port.

Voltage ports use volts relative to AVSS. IBIAS and load_current use amperes. load_conductance uses siemens, external_capacitance uses farads and temperature uses degrees C. The default timestep is 0.5 ns.

## Calibration and scope

The DC surface contains 506044 native transistor-level operating points across supply, reference, current and temperature. The nominal domain is VDD 1.2-1.4 V, VREF 0.9-1.2 V, load 0-100 mA and temperature -40 to 85 degrees C, at nominal AVDD and bias current. Additional supply data down to 1.12 V supports dropout characterization.

Supply AC calibration uses 2745 complex samples over 1 kHz-1 MHz at VREF=1 V and 100 mA. Startup, load-step, line-transient and noise fits apply to their documented nominal fixtures. Independent verification covered 137 DC conditions and 427 AC frequency points. Detailed measured deviations are in `doc/delivery_verification.json`.

The user confirmed adoption of the native-circuit dropout measurement of 144.9948 mV, approximately 145 mV. The original specification gives 200 mV. This difference is recorded in `doc/delivery_verification.json` using the 100 mA, 1% output-reduction definition.

Waveform fits cover simultaneous 1 us startup ramps with a 1 kohm load, 1-to-100 mA load changes with 100 ns ramps, and 1.2-to-1.4 V supply ramps over 1 us at 27 degrees C. Noise calibration covers 10 Hz-100 kHz at the nominal fixture. Additional slew rates, external capacitors, nonnominal AVDD or bias, process corners and mismatch require further native comparisons.
