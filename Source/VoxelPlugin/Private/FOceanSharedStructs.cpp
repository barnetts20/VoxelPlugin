#include "FOceanSharedStructs.h"

// ---------------------------------------------------------------------------
// FCubeTransform::FaceTransforms
//
// Defines the axis mapping, winding, and face-crossing transition table for
// each of the six cube faces used by the cube-sphere quadtree topology.
// ---------------------------------------------------------------------------

const FCubeTransform FCubeTransform::FaceTransforms[6] =
{
    { // X+
        FIntVector3(2, 1, 0),   // AxisMap:  face X->world Z, face Y->world Y, normal->world X
        FIntVector3(-1, 1, 1),  // AxisDir
        { EdgeOrientation::RIGHT, EdgeOrientation::RIGHT, EdgeOrientation::LEFT, EdgeOrientation::RIGHT },
        {
            { (uint8)EFaceDirection::Z_POS, {2,3,0,1} }, // LEFT
            { (uint8)EFaceDirection::Z_NEG, {0,1,2,3} }, // RIGHT
            { (uint8)EFaceDirection::Y_NEG, {3,1,2,0} }, // DOWN
            { (uint8)EFaceDirection::Y_POS, {2,0,3,1} }  // UP
        },
        true  // bFlipWinding
    },

    { // X-
        FIntVector3(2, 1, 0),
        FIntVector3(1,-1,-1),
        { EdgeOrientation::LEFT, EdgeOrientation::LEFT, EdgeOrientation::LEFT, EdgeOrientation::RIGHT },
        {
            { (uint8)EFaceDirection::Z_NEG, {1,0,3,2} }, // LEFT
            { (uint8)EFaceDirection::Z_POS, {3,2,1,0} }, // RIGHT
            { (uint8)EFaceDirection::Y_POS, {3,1,2,0} }, // DOWN
            { (uint8)EFaceDirection::Y_NEG, {2,0,3,1} }  // UP
        },
        false
    },

    { // Y+
        FIntVector3(0, 2, 1),
        FIntVector3(-1,-1, 1),
        { EdgeOrientation::UP, EdgeOrientation::DOWN, EdgeOrientation::DOWN, EdgeOrientation::DOWN },
        {
            { (uint8)EFaceDirection::X_POS, {1,3,0,2} }, // LEFT
            { (uint8)EFaceDirection::X_NEG, {3,1,2,0} }, // RIGHT
            { (uint8)EFaceDirection::Z_POS, {3,2,1,0} }, // DOWN
            { (uint8)EFaceDirection::Z_NEG, {2,3,0,1} }  // UP
        },
        false
    },

    { // Y-
        FIntVector3(0, 2, 1),
        FIntVector3(1, 1,-1),
        { EdgeOrientation::UP, EdgeOrientation::DOWN, EdgeOrientation::UP, EdgeOrientation::UP },
        {
            { (uint8)EFaceDirection::X_NEG, {1,3,0,2} }, // LEFT
            { (uint8)EFaceDirection::X_POS, {3,1,2,0} }, // RIGHT
            { (uint8)EFaceDirection::Z_NEG, {0,1,2,3} }, // DOWN
            { (uint8)EFaceDirection::Z_POS, {1,0,3,2} }  // UP
        },
        true
    },

    { // Z+
        FIntVector3(0, 1, 2),
        FIntVector3(1, 1, 1),
        { EdgeOrientation::LEFT, EdgeOrientation::RIGHT, EdgeOrientation::UP, EdgeOrientation::DOWN },
        {
            { (uint8)EFaceDirection::X_NEG, {3,2,1,0} }, // LEFT
            { (uint8)EFaceDirection::X_POS, {2,3,0,1} }, // RIGHT
            { (uint8)EFaceDirection::Y_NEG, {1,0,3,2} }, // DOWN
            { (uint8)EFaceDirection::Y_POS, {3,2,1,0} }  // UP
        },
        true
    },

    { // Z-
        FIntVector3(0, 1, 2),
        FIntVector3(1, 1,-1),
        { EdgeOrientation::RIGHT, EdgeOrientation::LEFT, EdgeOrientation::DOWN, EdgeOrientation::UP },
        {
            { (uint8)EFaceDirection::X_NEG, {1,0,3,2} }, // LEFT
            { (uint8)EFaceDirection::X_POS, {0,1,2,3} }, // RIGHT
            { (uint8)EFaceDirection::Y_NEG, {0,1,2,3} }, // DOWN
            { (uint8)EFaceDirection::Y_POS, {2,3,0,1} }  // UP
        },
        false
    }
};