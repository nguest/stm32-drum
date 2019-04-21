# STM32-drum
STM32Ff103 drum machine / 12bits

Output 1.3" 128x64 OLED via Arduino Nano as SPI slave.
Capacitative touch for drum triggers (WIP)

TODO: 
- [x] Basic operation from pre-programmed pattern
- [x] SPI transfer to Nano and live pattern display on screen
- [x] Play/ Pause/ Record buttons
- [ ] Digital tempo change
- [ ] Shorten samples / save length pattern
- [ ] Reduce gain / save gain pattern
- [ ] Display gain / length on screen
- [ ] Swing?
- [x] Capacitative touch buttons >= 8
- [ ] Multiple buffers for multiple PWM output?
- [ ] Save / recall patterns
- [ ] DAC output
- [ ] MIDI input (at least for tempo?)

![STM32-drum breadboard](/stm32-drum_bb.png)
