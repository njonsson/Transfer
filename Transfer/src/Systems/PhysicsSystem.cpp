// File: Transfer/src/Systems/PhysicsSystem.cpp

#include "PhysicsSystem.h"

PhysicsSystem::PhysicsSystem()
{
    // Initialize physics system variables if needed
}

PhysicsSystem::~PhysicsSystem()
{
    // Cleanup if necessary
}
void PhysicsSystem::CleanUp()
{
    // Any necessary cleanup code for the physics system
}

void PhysicsSystem::UpdateSystemFrame(GameState& state, UIState& UIState)
{
    InputState& inputState = UIState.getMutableInputState();
    if (inputState.dirty)
    {
        if (inputState.isCreatingCluster)
            {
                // std::cout<<"Creating cluster"<<std::endl;
                createPlanetaryCluster(state, inputState);
            }
    }

    // std::cout<<"Number of particles"<<state.getParticles().size()<<std::endl;
}


void PhysicsSystem::handleCollision(GameState& state)
{
    // Handle collision logic here
}
void PhysicsSystem::handleDynamicCollision(GameState& state)
{
    // Handle dynamic collision logic here
}

// void PhysicsSystem::createPlanetaryCluster(GameState& state, InputState& inputState)
// {
//     // Create planetary clusters logic here
//     Particle particle;
//     particle.mass = inputState.selectedMass;
//     particle.radius = inputState.selectedRadius;
//     particle.position = inputState.mouseCurrPosition;
//     particle.prevPosition = particle.position;

//     state.getParticlesMutable().push_back(particle);
//     inputState.dirty = false;
// }

void PhysicsSystem::createPlanetaryCluster(GameState& state, InputState& inputState)
{
    // 1. Define Cluster Parameters (You'll need to pass these in, 
    //    or derive them from the mass of the destroyed body)
    const int numParticles = 25; 
    const double clusterRadius = inputState.selectedRadius * 1.5; // Spread them out
    const double totalMass = inputState.selectedMass; 
    const double singleParticleMass = totalMass / numParticles; 

    // Center of the cluster (e.g., the mouse click position or collision point)
    Vector2D centerPosition = inputState.mouseCurrPosition; 

    // Seed the random number generator (do this once outside the loop for performance)
    std::random_device rd;
    std::mt19937 gen(rd());

    // --- 2. The Spawning Loop ---
    for (int i = 0; i < numParticles; ++i)
    {
        // 2a. Random Polar Coordinates: Generate random distance (r) and angle (theta)
        // Note: Squaring the radius distribution (r * r) gives a more uniform
        // distribution of points across the circle's area, avoiding clumping in the center.
        double r = clusterRadius * std::sqrt(std::uniform_real_distribution<>(0.0, 1.0)(gen));
        double theta = std::uniform_real_distribution<>(0.0, 2.0 * M_PI)(gen);

        // 2b. Convert to Cartesian Coordinates (x, y)
        double dx = r * std::cos(theta);
        double dy = r * std::sin(theta);

        // 2c. Create and Initialize the Particle
        Particle particle;
        particle.mass = singleParticleMass;
        // The particle radius should be tiny, representing dust/molecules
        particle.radius = inputState.selectedRadius / std::sqrt(numParticles); 
        
        // Position is center + offset
        particle.position.x_val = centerPosition.x_val + dx;
        particle.position.y_val = centerPosition.y_val + dy;
        
        // PrevPosition is initialized to current position for the first step of Verlet
        particle.prevPosition = particle.position; 
        
        // Optionally, add a small, random initial velocity to make them "jiggle"
        // particle.velocity = Vector2::randomSmallVector();

        // 2d. Add to the game state
        state.getParticlesMutable().push_back(particle);
    }

    inputState.dirty = false;
}