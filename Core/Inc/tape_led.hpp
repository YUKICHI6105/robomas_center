#include <main.h>

namespace tapeled{
	struct State{
		uint8_t red;
		uint8_t green;
		uint8_t blue;
	};
	extern State led_state;

	bool updateLedState(State state);
	State getLedState();
	bool setLedState(State state);

	// Colour requested by the host over USB (command 0x05). The USB interrupt
	// only records the request here; update_tape_led() in the 1 kHz tick owns
	// the hardware and applies it once the emergency stop is released, so a
	// host command can never paint over the stop indication. The value is held
	// packed as 0x00RRGGBB so publishing it is a single 32-bit store and cannot
	// tear against the lower-priority tick that reads it.
	void setRequestedState(State state);
	State getRequestedState();
}
