// PlanetGravityZone.h — Spherical gravity zone subclass for planets.
// Pulls objects toward the actor's origin. Uses two sphere colliders:
//   - Inner sphere (actor scale): the radius at which gravity reaches full
//     strength (BaseVector magnitude). Visual-only, no overlap events.
//   - Outer sphere (actor scale * InfluenceScale): the zone boundary that
//     generates overlap events. Objects inside this sphere are affected.
//
// Gravity direction is always toward the actor origin. Magnitude is derived
// from BaseVector.Size() and ramps from zero at the outer edge to full
// strength at the inner sphere, controlled by the Falloff exponent.
//
// Inherits Priority, ExcludeTags, and BaseVector from AGravityZone.
// When spawned by APlanetActor, transforms are locked to follow the planet.

#pragma once

#include "CoreMinimal.h"
#include "GravityZone.h"
#include "Components/SphereComponent.h"
#include "PlanetGravityZone.generated.h"

UCLASS()
class VOXELPLUGIN_API APlanetGravityZone : public AGravityZone
{
    GENERATED_BODY()

public:
    APlanetGravityZone();

    /** Multiplier for the outer (influence) sphere radius.
     *  Outer radius = actor scale * InfluenceScale.
     *  Objects inside the outer sphere are affected by gravity. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet Gravity", meta = (ClampMin = "1.0"))
    double InfluenceScale = 3.0;

    /** Falloff exponent controlling how gravity ramps between the outer and
     *  inner spheres. 1.0 = linear, 2.0 = quadratic, etc.
     *  At the outer edge gravity is zero; at the inner sphere it reaches
     *  full strength (BaseVector magnitude). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet Gravity", meta = (ClampMin = "0.01"))
    double Falloff = 2.0;

    /** True when spawned and driven by APlanetActor. Location and scale
     *  become read-only (follows planet center). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet Gravity")
    bool bIsPlanetOwned = false;

    // --- Inherited from AGravityZone (exposed in BP, listed here for reference) ---
    // Priority      — int32, zone ordering
    // ExcludeTags   — TArray<FName>, actors with these tags are excluded
    // BaseVector    — FVector, magnitude = max gravity acceleration (direction unused)

    virtual FVector GetGravityVector_Implementation(const FVector& InWorldPosition) const override;

    /** Called by APlanetActor after spawn + attach. Configures radii and
     *  marks the zone as planet-owned. */
    void InitializeFromPlanet(double InPlanetRadius, double InAtmosphereScale);

    /** Updates the outer sphere radius from actor scale * InfluenceScale.
     *  Call after changing InfluenceScale or actor scale at runtime. */
    void UpdateInfluenceSphereRadius();

    // --- Editor Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

private:
    /** Inner sphere — visual display only, shows the full-gravity radius.
     *  Radius matches actor scale (set via the root component). */
    UPROPERTY(VisibleAnywhere, Category = "Planet Gravity")
    TObjectPtr<USphereComponent> InnerSphere;

    /** Outer sphere — generates overlap events for the gravity influence zone.
     *  Radius = actor scale * InfluenceScale. */
    UPROPERTY(VisibleAnywhere, Category = "Planet Gravity")
    TObjectPtr<USphereComponent> InfluenceSphere;

    /** Cached planet-driven location for the transform guard. */
    FVector PlanetDrivenLocation = FVector::ZeroVector;

    /** Overlap callbacks — forward to base class OnZoneBeginOverlap / OnZoneEndOverlap. */
    UFUNCTION()
    void OnInfluenceBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
        UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
        bool bFromSweep, const FHitResult& SweepResult);

    UFUNCTION()
    void OnInfluenceEndOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
        UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Snaps location back to planet-driven values. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);
};