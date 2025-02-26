#pragma once

#include "CoreMinimal.h"

// Define different SDF types
UENUM(BlueprintType)
enum class ESDFType : uint8
{
    Sphere  UMETA(DisplayName = "Sphere"),
    Box     UMETA(DisplayName = "Box"),
    Torus   UMETA(DisplayName = "Torus"),
    Custom  UMETA(DisplayName = "Custom")
};

// Struct for parameters sent to the compute shader
struct FSDFParameters
{
public:
    FVector Center;  // SDF center position
    float Radius;    // Used for Sphere SDF
    FVector BoxSize; // Used for Box SDF
    float Smoothness; // Used for blending in some SDFs
    ESDFType SDFType; // Type of SDF

    FSDFParameters()
        : Center(FVector::ZeroVector), Radius(100.0f), BoxSize(FVector(100.0f)), Smoothness(0.0f), SDFType(ESDFType::Sphere) {}

    FSDFParameters(FVector InCenter, float InRadius, ESDFType InSDFType)
        : Center(InCenter), Radius(InRadius), BoxSize(FVector(100.0f)), Smoothness(0.0f), SDFType(InSDFType) {}

    FSDFParameters(FVector InCenter, FVector InBoxSize, float InSmoothness)
        : Center(InCenter), Radius(0.0f), BoxSize(InBoxSize), Smoothness(InSmoothness), SDFType(ESDFType::Box) {}
};
