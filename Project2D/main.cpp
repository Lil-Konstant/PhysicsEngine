#include "PhysicsApp.h"

/// <summary>
/// Main simply instantiates a new PhysicsApp and calls run on it,
/// which continually loops gameplay until the game is closed.
/// </summary>
int main() {
	
	// allocation
	auto app = new PhysicsApp();

	// initialise and loop
	app->run("AIE", 1280, 720, false);

	// deallocation
	delete app;

	return 0;
}