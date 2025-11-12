// File: Transfer/src/Systems/UISystem.cpp

#include "UISystem.h"

// Constructor
UISystem::UISystem()
{
    // Initialize any UI system state here
}
// Destructor
UISystem::~UISystem()
{
    // Clean up any allocated resources here
}

void UISystem::ProcessUIFrame(GameState& state, UIState& UIState)
{
    
}

void UISystem::RenderUIElements(SDL_Renderer* renderer, UIState& UIState, TTF_Font* UIFont)
{
    std::vector<UIElement*> ui_elements = UIState.getUIElements();
    for (auto& element : ui_elements) {
        element->renderElement(renderer, UIState, UIFont); // Pass UIFont if needed
    }
}   