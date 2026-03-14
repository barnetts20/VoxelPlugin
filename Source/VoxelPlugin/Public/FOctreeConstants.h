// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
struct VOXELPLUGIN_API OctreeConstants {
    static inline const FVector Directions[6] =
    {
        FVector(1, 0, 0),
        FVector(-1, 0, 0),
        FVector(0, 1, 0),
        FVector(0, -1, 0),
        FVector(0, 0, 1),
        FVector(0, 0, -1)
    };

    inline static const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1),  FVector(1, 1, -1),
        FVector(-1, -1, 1),  FVector(1, -1, 1),
        FVector(-1, 1, 1),   FVector(1, 1, 1)
    };

    inline static const int EdgePairs[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    inline static const int32 ChildCornerSources[8][8] = {
    {  0,  8, 12, 24, 16, 22, 20, 26 },
    {  8,  1, 24, 13, 22, 17, 26, 21 },
    { 12, 24,  2,  9, 20, 26, 18, 23 },
    { 24, 13,  9,  3, 26, 21, 23, 19 },
    { 16, 22, 20, 26,  4, 10, 14, 25 },
    { 22, 17, 26, 21, 10,  5, 25, 15 },
    { 20, 26, 18, 23, 14, 25,  6, 11 },
    { 26, 21, 23, 19, 25, 15, 11,  7 },
    };

    inline static const int8 GridCoords[27][3] = {
        {-1,-1,-1},{+1,-1,-1},{-1,+1,-1},{+1,+1,-1},
        {-1,-1,+1},{+1,-1,+1},{-1,+1,+1},{+1,+1,+1},
        { 0,-1,-1},{ 0,+1,-1},{ 0,-1,+1},{ 0,+1,+1},
        {-1, 0,-1},{+1, 0,-1},{-1, 0,+1},{+1, 0,+1},
        {-1,-1, 0},{+1,-1, 0},{-1,+1, 0},{+1,+1, 0},
        {-1, 0, 0},{+1, 0, 0},{ 0,-1, 0},{ 0,+1, 0},
        { 0, 0,-1},{ 0, 0,+1},{ 0, 0, 0}
    };

    // CoordToGrid[x+1][y+1][z+1]
    inline static const int8 CoordToGrid[3][3][3] = {
        { {  0, 16,  4 }, { 12, 20, 14 }, {  2, 18,  6 } },
        { {  8, 22, 10 }, { 24, 26, 25 }, {  9, 23, 11 } },
        { {  1, 17,  5 }, { 13, 21, 15 }, {  3, 19,  7 } },
    };

    inline static const int32 FaceCorners[6][4] = {
        { 0, 2, 4, 6 },
        { 1, 3, 5, 7 },
        { 0, 1, 4, 5 },
        { 2, 3, 6, 7 },
        { 0, 1, 2, 3 },
        { 4, 5, 6, 7 },
    };
};