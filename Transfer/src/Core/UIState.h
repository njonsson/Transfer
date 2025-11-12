// File: Transfer/src/Core/UIState.h

#pragma once

// No SDL here. Will just include data to manage the UI state.

// Custom Imports
// #include "Entities/UIElement.h"

// Standard Library Imports
#include <vector>


class UIElement;

class UIState
{
    public:
        UIState();
        ~UIState();

    public:
        float getFPS() const { return fps; }
        void setFPS(float framesPerSecond) { fps = framesPerSecond; }
        // Add UI state management methods and members here
        std::vector<UIElement*> getUIElements() const { return elements; }

        bool getShowFPSCounter() const { return showFPSCounter; }
        void setShowFPSCounter(bool show) { showFPSCounter = show; }
    private:
        // FPS counter state
        float fps = 0.0f;
        bool showFPSCounter = false;

        std::vector<UIElement*> elements;
};