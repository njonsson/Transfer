// File: Transfer/src/Core/InputState.h

#pragma once
#include "Entities/PhysicsStructures.h"
// class InputState
// {
//     public:
//         // Constructor and Destructor
//         InputState();
//         ~InputState();
//     public:
//         bool isMouseDragging = false;
//         float mouseDragStartX = 0.0f;
//         float mouseDragStartY = 0.0f;
//         float mouseDragEndX = 0.0f;
//         float mouseDragEndY = 0.0f;


//     private:

//         // Body instantiation control vars
//         // float selectedRadius = 0.0;
//         // double selectedMass = 0.0;
//         // bool bodySelectionValidity = false;
// };

struct InputState
{   
    bool dirty = false;
    bool isMouseDragging = false;
    bool isCreatingCluster = false;
    Vector2D mouseCurrPosition = {0.0, 0.0};
    Vector2D mouseDragStartPosition = {0.0, 0.0};
    Vector2D mouseDragEndPosition = {0.0, 0.0};
    double selectedMass = 0.0;
    double selectedRadius = 0.0;
};