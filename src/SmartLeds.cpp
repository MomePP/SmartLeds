#include "SmartLeds.h"

IsrCore SmartLed::_interruptCore = CoreCurrent;

SmartLed*& IRAM_ATTR SmartLed::ledForChannel(int channel) {
    static SmartLed* table[led_timing::CHANNEL_COUNT] = {};
    assert(channel < led_timing::CHANNEL_COUNT);
    return table[channel];
}
