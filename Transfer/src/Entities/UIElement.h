// File: Transfer/src/Entities/UIElement.h

#pragma once

// SDL3 Imports
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

// Custom Imports
#include "Core/UIState.h"
#include "Utilities/Colors.h"

// Standard Library Imports
#include <string>




class UIElement{
    public:
        UIElement();
        virtual ~UIElement();

        virtual void renderElement(SDL_Renderer* renderer, UIState& UIState, TTF_Font* UIFont)
        {
            // Default implementation (can be empty)
            return;
        }; // will be overloaded by derived classes
    private:
        int posX = 0;
        int posY = 0;
        int width = 0;
        int height = 0;
};

// Derived UI Element Classes

// class SimulationSpeedSlider : public UIElement
// {
// };
// class VelocityVectorToggle : public UIElement
// {
// };

// class GravityToggle : public UIElement
// {
// };

class FPSCounter : public UIElement
{
    FPSCounter() = default; // Need to instantiate stuff here like the fps text and font?
    ~FPSCounter() = default;
    virtual void renderElement(SDL_Renderer* renderer, UIState& UIState, TTF_Font* UIFont) override
    {
        if (!UIState.getShowFPSCounter())
        {
            return;
        }
        else
        {
            float fps = UIState.getFPS();
            std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
            SDL_Surface* textSurface = TTF_RenderText_Blended(UIFont, fpsText.c_str(), fpsText.length(), ColorLibrary::White);
            if (!textSurface) {
                SDL_Log("Text surface creation failed: %s", SDL_GetError());
                return;
            }

            SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
            if (!textTexture) {
                SDL_Log("Text texture creation failed: %s", SDL_GetError());
                return;
            }
            SDL_FRect dstRect = {10.0f, 10.0f, static_cast<float>(textSurface->w), static_cast<float>(textSurface->h)};
            SDL_RenderTexture(renderer, textTexture, nullptr, &dstRect);
            SDL_DestroySurface(textSurface);
            SDL_DestroyTexture(textTexture);
        }
    };
    private:
        // TTF_Font* font; // Font for rendering text
        // SDL_Texture* fpsTexture; // Texture for the FPS text
};

// class MassSlider : public UIElement
// {
// };

// class PauseMenu : public UIElement
// {
// };

// class SelectionCheckbox : public UIElement
// {
// };