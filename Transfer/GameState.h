#pragma once

#include "Bodies.h"
#include <vector>

struct GameState {
	std::vector<GravitationalBody> bodies;
	double selectedRadius = 0.0;
	double selectedMass = 0.0;
	bool bodySelectionValidity = false;
	bool playing = false;
};
