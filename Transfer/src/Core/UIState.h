// File: Transfer/src/Core/UIState.h

#pragma once

// No SDL here. Will just include data to manage the UI state.

// Custom Imports
#include "Core/InputState.h"

// Standard Library Imports
#include <vector>

// Forward declaration of UI Element class. Full include leads to circular dependency, causing failures in compilation.
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
        std::vector<UIElement*> getUIElements() const { return uielements; }
        // Initializing helper.
        void addUIElement(UIElement* uielement) { uielements.push_back(uielement); }
        // Cleanup helper.
        void clearUIElements() { uielements.clear(); }

        bool getShowFPSCounter() const { return showFPSCounter; }
        void setShowFPSCounter(bool show) { showFPSCounter = show; }

        InputState& getMutableInputState() { return inputState; }
        const InputState& getInputState() const { return inputState; }
    private:
        // FPS counter state
        float fps = 0.0f;
        bool showFPSCounter = false;
        // Input state for UI interactions
        InputState inputState;
        // Collection of UI elements
        std::vector<UIElement*> uielements;
};