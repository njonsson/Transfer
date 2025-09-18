#pragma once
// Custom imports
#include "Constants.h"
#include "Bodies.h"

//standard library imports
#include <cmath>
#include <iostream>
#include <vector>
// In the gravitational body, we have access to mass and radius of the bodies. We can pretend particles are pointlike, but need to add a physical radius 
// for collision detection

double calculateGravitationalForce(int mass1, int mass2, int radius1, int radius2, int distance);
void applyGravitationalForceEffects(GravitationalBody& body1, GravitationalBody& body2);
int distanceBetweenBodies(const GravitationalBody& body1, const GravitationalBody& body2);
//void applyGravityToSystem(std::vector<GravitationalBody>& bodies);