# Power-delivery SystemC models

The datasheet-model collection contains 31 model implementations and 32 baseline scenarios.

| Model | Directory |
| --- | --- |
| auirs2117s | [power-interface/gate-driver/high-side/auirs2117s](power-interface/gate-driver/high-side/auirs2117s) |
| lm1117_3v3 | [ldo/external-cap/general/lm1117_3v3](ldo/external-cap/general/lm1117_3v3) |
| lm2665_sw | [charge-pump/fixed-ratio-boost/lm2665_sw](charge-pump/fixed-ratio-boost/lm2665_sw) |
| lm3350_sw | [charge-pump/fixed-ratio-step-down/lm3350_sw](charge-pump/fixed-ratio-step-down/lm3350_sw) |
| lm337 | [ldo/external-cap/negative/lm337](ldo/external-cap/negative/lm337) |
| lm5116mh | [dc-dc/buck-controller/single-phase/lm5116mh](dc-dc/buck-controller/single-phase/lm5116mh) |
| lm5118mh | [dc-dc/buck-boost-controller/lm5118mh](dc-dc/buck-boost-controller/lm5118mh) |
| lt8627sp | [dc-dc/buck-converter/low-noise/lt8627sp](dc-dc/buck-converter/low-noise/lt8627sp) |
| max20310 | [dc-dc/simo-converter/max20310](dc-dc/simo-converter/max20310) |
| max865_sw | [charge-pump/bipolar/max865_sw](charge-pump/bipolar/max865_sw) |
| mcp1623 | [dc-dc/boost-converter/low-power/mcp1623](dc-dc/boost-converter/low-power/mcp1623) |
| mcp1711_3v3 | [ldo/capless/general/mcp1711_3v3](ldo/capless/general/mcp1711_3v3) |
| mp2183 | [dc-dc/buck-converter/single-phase/mp2183](dc-dc/buck-converter/single-phase/mp2183) |
| tang2019_dcldo_0v8 | [ldo/capless/digital/tang2019_dcldo_0v8](ldo/capless/digital/tang2019_dcldo_0v8) |
| tlv71333_3v3 | [ldo/capless/low-power/tlv71333_3v3](ldo/capless/low-power/tlv71333_3v3) |
| tps22975 | [power-interface/load-switch/tps22975](power-interface/load-switch/tps22975) |
| tps43060 | [dc-dc/boost-controller/tps43060](dc-dc/boost-controller/tps43060) |
| tps51200 | [ldo/external-cap/source-sink/tps51200](ldo/external-cap/source-sink/tps51200) |
| tps53647 | [dc-dc/buck-controller/multiphase/tps53647](dc-dc/buck-controller/multiphase/tps53647) |
| tps546a24a | [dc-dc/buck-converter/multiphase/tps546a24a](dc-dc/buck-converter/multiphase/tps546a24a) |
| tps60150_sw | [charge-pump/regulated-output/tps60150_sw](charge-pump/regulated-output/tps60150_sw) |
| tps60400_sw | [charge-pump/inverting/tps60400_sw](charge-pump/inverting/tps60400_sw) |
| tps61235p | [dc-dc/boost-converter/general/tps61235p](dc-dc/boost-converter/general/tps61235p) |
| tps62746 | [dc-dc/buck-converter/low-power/tps62746](dc-dc/buck-converter/low-power/tps62746) |
| tps63021 | [dc-dc/buck-boost-converter/tps63021](dc-dc/buck-boost-converter/tps63021) |
| tps73133_3v3 | [ldo/capless/low-noise-high-psrr/tps73133_3v3](ldo/capless/low-noise-high-psrr/tps73133_3v3) |
| tps7a02_1v8 | [ldo/external-cap/low-power/tps7a02_1v8](ldo/external-cap/low-power/tps7a02_1v8) |
| tps7a20_2v8 | [ldo/external-cap/low-noise-high-psrr/tps7a20_2v8](ldo/external-cap/low-noise-high-psrr/tps7a20_2v8) |
| tps7b4250_q1 | [ldo/external-cap/tracking/tps7b4250_q1](ldo/external-cap/tracking/tps7b4250_q1) |
| ucc27282 | [power-interface/gate-driver/half-bridge/ucc27282](power-interface/gate-driver/half-bridge/ucc27282) |
| ucc27624 | [power-interface/gate-driver/low-side/ucc27624](power-interface/gate-driver/low-side/ucc27624) |

Each model provides its own C++17 build entry, parameter files, documentation and baseline runner.
