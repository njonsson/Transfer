#include "RunGame.h"


void StartGame()
{
	SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_AUDIO);
	SDL_Window* window = SDL_CreateWindow("title", 1920, 1080, 0);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // black background
	SDL_RenderClear(renderer);
	
	GameState state; //instantiate game state
	state.playing = true;
	while (state.playing) {
		PlayGame(state); // updates game state by 1 frame
		// package this as a helper function later.


		// render the frame
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // black background
		SDL_RenderClear(renderer);
		for (const auto& body : state.bodies) {
			instantiateGravitationalBody(renderer, body);
		}
		SDL_Delay(TIME_STEP_MS); // 62.5 FPS cap
		SDL_RenderPresent(renderer);
	}
	SDL_Delay(5000);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}


void PlayGame(GameState& state)
{
	SDL_Event event;
	while (SDL_PollEvent(&event)) {
		if (event.type == SDL_EVENT_QUIT) {
			state.playing = false;
		}
		HandleBodyPropertyKeyEvent(event, state.selectedRadius, state.selectedMass);
		CheckBodyInstantiationValidity(state);
		if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && state.bodySelectionValidity) {
			int xPos = static_cast<int>(event.button.x);
			int yPos = static_cast<int>(event.button.y);
			GravitationalBody newBody;
			newBody.x = xPos;
			newBody.y = yPos;
			newBody.radius = state.selectedRadius;
			newBody.mass = state.selectedMass;
			newBody.netVelocity = { 0, 0};
			newBody.netForce = { 0.0, 0.0 };
			state.bodies.push_back(newBody);
			state.selectedRadius = 0;
			state.selectedMass = 0;
		}
		//if (event.type == SDL_EVENT_KEY_DOWN && state.bodies.size() == 2) {
		//	std::cout << "P key pressed. Swapping velocities of two bodies." << std::endl;
		//	if (event.key.scancode == SDL_SCANCODE_P) {
		//		// 1 to 2 vector 
		//		double xvec = state.bodies[1].x - state.bodies[0].x;
		//		double yvec = state.bodies[1].y - state.bodies[0].y;
		//		state.bodies[0].netVelocity = { xvec,yvec };
		//		state.bodies[1].netVelocity = { - xvec, -yvec };
		//	}
		//}
		
	}
	//std::cout << "Reached physics update step with " << state.bodies.size() << " bodies." << std::endl;
	if (state.bodies.size() >= 2) {
		// Debug: print out the net force on the first 
		applyGravityToSystem(state.bodies);
		//std::cout << "Force applied to first body" << state.bodies[0].netForce.f_x << "," << state.bodies[0].netForce.f_y << std::endl;
		// velocities of first body
		std::cout << "Velocity of first body" << state.bodies[0].netVelocity.v_x << "," << state.bodies[0].netVelocity.v_y << std::endl;
		//// --- physics update should live here ---
	for (auto& body : state.bodies) {
	    body.x += body.netVelocity.v_x * TIME_STEP;
	    body.y += body.netVelocity.v_y * TIME_STEP;
	}
	}
    

    // Later: applyGravityToSystem(state.bodies);
}

void CheckBodyInstantiationValidity(GameState& state)
{
	if ((state.selectedRadius > 0) && (state.selectedMass > 0))
	{
		state.bodySelectionValidity = true;
		//std::cout << "Body can be instantiated. Click to place body." << std::endl;
	}
	else
	{
		state.bodySelectionValidity = false;
		//std::cout << "Body cannot be instantiated. Please select radius and mass." << std::endl;
	}
}