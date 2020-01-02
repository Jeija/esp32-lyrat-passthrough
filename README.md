# ESP32-LyraT Audio Passthrough
### Short Description
This [esp-idf](https://github.com/espressif/esp-idf)-based application for the [ESP32-LyraT](https://www.espressif.com/en/products/hardware/esp32-lyrat) reads stereo samples from the board's analog "AUX IN" input jack and outputs them through the "PHONE JACK" analog output again. It does so without all the overhead that [esp-adf](https://github.com/espressif/esp-adf) brings at the cost of not being as portable as [esp-adf's pipeline_passthrough](https://github.com/espressif/esp-adf/tree/master/examples/audio_processing/pipeline_passthru).

By default, this application uses a word length of 16 bits (bits per channel) and a sample rate of 44100kHz. It first configures the ES8388 audio codec chip on the ESP32-LyraT board through I²C and then transfers digital audio samples bidirectionally over I²S.

This project can be used as a starting point for implementing **digital filters** such as FIR / IIR / biquad filters and other digital signal processing (DSP) applications on the ESP32 plattform. The ESP32's Tensilica Xtensa processor architecture is well-know for its DSP capabilities. For instance, FIR filters can be efficiently implemented on the ESP32 thanks to support for special 16-bit multiply-accumulate instructions with optional parallel loading (**MAC16** option for Tensilica Xtensa LX6).. See [esp-dsp](https://github.com/espressif/esp-dsp) and the Xtensa Instruction Set Architecture (ISA) Reference Manual for more information.

### Build and Flash
Make sure to have a recent [esp-idf](https://github.com/espressif/esp-idf) version with support for cmake-based projects installed. Then compile and flash this project as usual using the following steps:

* `idf.py all` to compile everything
* Put your ESP32-LyraT into download mode: Press and hold `Boot`, press and release `RST`, release `Boot`
* `idf.py -p /dev/ttyUSBX flash`to flash everything
* Reset your ESP32-LyraT by pressing `RST`

### License
This project is licensed under the MIT license. See `LICENSE` for details.
