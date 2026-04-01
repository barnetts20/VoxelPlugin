// PlanetGravityZone.h — Spherical gravity zone subclass for planets.
// C++ equivalent of BP_PlanetGravitySphere, adapted to use actor scale
// for the inner radius (same pattern as PlanetAtmosphereActor).
//
// Two sphere collision components:
//   - Planet (inner): display only, unit radius scaled by actor transform.
//     Effective radius = GetActorScale3D().GetMax(). Marks the boundary
//     at which gravity reaches full strength.
//   - Sphere (outer): overlap events, sized explicitly to
//     GetActorScale3D().GetMax() * InfluenceRatio. Uses absolute scale
//     so it is not affected by the actor transform.
//
// GetGravityVector:
//   InnerRadius = GetActorScale3D().GetMax()
//   Direction   = Normalize(ActorLocation - WorldPosition)
//   Magnitude   = BaseVector.Size()
//   If distance <= InnerRadius: full strength (Direction * Magnitude)
//   If distance >  InnerRadius: Direction * Magnitude * Pow(Distance/InnerRadius, Falloff)
//
// Inherits Priority, ExcludeTags, and BaseVector from AGravityZone.

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
     *  Outer radius = ActorScale * InfluenceRatio.
     *  Objects inside this sphere are affected by gravity. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity Zone", meta = (ClampMin = "1.0"))
    float InfluenceRatio = 2.0f;

    /** Falloff exponent applied beyond the inner sphere.
     *  Gravity = BaseVector.Size() * Pow(InnerRadius / Distance, Falloff).
     *  Positive values give decay: 2 = inverse square, 1 = linear.
     *  0 = constant strength everywhere in the influence zone. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity Zone")
    float Falloff = 2.0f;

    /** True when spawned and driven by APlanetActor. Location and scale
     *  become read-only (follows planet transform). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Gravity Zone")
    bool bIsPlanetOwned = false;

    // --- Inherited from AGravityZone ---
    // Priority      — int32, zone ordering
    // ExcludeTags   — TArray<FName>, actors with these tags are excluded
    // BaseVector    — FVector, magnitude = max gravity acceleration (direction unused)

    virtual FVector GetGravityVector_Implementation(const FVector& InWorldPosition) const override;

    /** Called by APlanetActor after spawn + attach. Marks planet-owned and
     *  updates sphere radii (actor scale must already be set by the planet). */
    void InitializeFromPlanet();

    /** Resizes the outer sphere from current actor scale * InfluenceRatio.
     *  Call after changing scale or InfluenceRatio at runtime. */
    void UpdateInfluenceSphereRadius();

    // --- Lifecycle ---

    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;

    // --- Editor Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

private:
    /** Root scene component — provides the transform gizmo in the editor.
     *  Matches the BP's DefaultSceneRoot. */
    UPROPERTY(VisibleAnywhere, Category = "Gravity Zone")
    TObjectPtr<USceneComponent> DefaultSceneRoot;

    /** Inner sphere — display only, unit radius (1.0). Scaled by actor
     *  transform so its effective world radius = actor scale. */
    UPROPERTY(VisibleAnywhere, Category = "Gravity Zone")
    TObjectPtr<USphereComponent> Planet;

    /** Outer sphere — overlap events, absolute scale. Radius set explicitly
     *  to ActorScale * InfluenceRatio via UpdateInfluenceSphereRadius. */
    UPROPERTY(VisibleAnywhere, Category = "Gravity Zone")
    TObjectPtr<USphereComponent> Sphere;

    /** Cached planet-driven location for the transform guard. */
    FVector PlanetDrivenLocation = FVector::ZeroVector;

    /** Bound to OnActorBeginOverlap / OnActorEndOverlap delegates —
     *  exactly what BP "Event ActorBeginOverlap" nodes bind to. */
    UFUNCTION()
    void OnActorOverlapBegin(AActor* OverlappedActor, AActor* OtherActor);

    UFUNCTION()
    void OnActorOverlapEnd(AActor* OverlappedActor, AActor* OtherActor);

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Snaps location and scale back to planet-driven values. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);
};