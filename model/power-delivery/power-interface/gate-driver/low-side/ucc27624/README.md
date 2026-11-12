# ucc27624

SystemC/SystemC-AMS power-delivery model. Model documentation and the existing self-test report are in `doc/`.

## Build and run

Use C++17, SystemC 2.3.4 and SystemC-AMS 2.3.4. The repository's nlohmann/json 3.12.0 header is bundled under `include/nlohmann` with its embedded MIT license.

From this model directory on Linux or WSL, run

```sh
export SYSTEMC_HOME=/path/to/systemc-2.3.4
export SYSTEMC_AMS_HOME=/path/to/systemc-ams-2.3.4
cmake -S test -B test/build -DCMAKE_BUILD_TYPE=Release
cmake --build test/build --parallel 2
python3 test/run.py
```

The runner uses the existing expert-package baseline scenarios and writes waveforms, logs and measurements under `test/build/results`. `PASS` means the supplied numeric checks passed. `OBSERVED` means the scenario completed with finite outputs and provides recorded measurements without numeric acceptance limits.

## Files

- `include/` and `source/` contain the model implementation.
- `json/` contains model parameters.
- `test/` contains the build entry and baseline scenarios.
- `doc/source_manifest.json` records the supplied source files and their SHA-256 values.
