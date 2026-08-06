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
}
