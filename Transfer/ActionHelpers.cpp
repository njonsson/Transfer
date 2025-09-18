#include "ActionHelpers.h"

void instantiateGravitationalBody(SDL_Renderer* renderer, const GravitationalBody& body)
{
	Color color = GetColorForMass(body.mass);
	renderCircle(renderer, body.x, body.y, body.radius, color);
}