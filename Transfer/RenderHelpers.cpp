#include "RenderHelpers.h"
#include "Colors.h"
void renderCircle(SDL_Renderer* renderer, int x, int y, int radius, Color color)
{
	SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
	for (int w = 0; w < radius * 2; w++)
	{
		for (int h = 0; h < radius * 2; h++)
		{
			int dx = radius - w; // horizontal offset
			int dy = radius - h; // vertical offset
			if ((dx * dx + dy * dy) <= (radius * radius))
			{
				SDL_RenderPoint(renderer, x + dx, y + dy);
			}
		}
	}
}

