#include <Arduino.h>

#include "cansat/flight_app.hpp"
#include "cansat/ground_app.hpp"

namespace {

#if defined(MODE_TRANSMITTER)
cansat::FlightApp app;
#elif defined(MODE_RECEIVER)
cansat::GroundApp app;
#else
#error "Define MODE_TRANSMITTER or MODE_RECEIVER in platformio.ini"
#endif

}  // namespace

void setup() { app.setup(); }

void loop() { app.loop(); }
