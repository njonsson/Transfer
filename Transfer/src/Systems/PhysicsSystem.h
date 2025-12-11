// File: Transfer/src/Systems/PhysicsSystem.h

#pragma once

// Custom Imports
#include "Core/GameState.h"
#include "Core/UIState.h"
#include "Utilities/EngineConstants.h"
#include "Utilities/GameSystemConstants.h"
#include "Entities/GravitationalBody.h"
#include "Entities/PhysicsStructures.h"

// Standard Library Imports
#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>

#include <iostream>

class PhysicsSystem
{
    public:
        // Constructor and Destructor
        PhysicsSystem();
        ~PhysicsSystem();

    public:
        // Methods to update physics. Essentially integrates one physics frame worth of information
        void CleanUp();
        void UpdateSystemFrame(GameState& state, UIState& UIState);

    private:
        void handleCollision(GameState& state);
        void handleDynamicCollision(GameState& state);
        void createPlanetaryCluster(GameState& state, InputState& inputState);

};