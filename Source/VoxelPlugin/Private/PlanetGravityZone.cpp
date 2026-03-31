// PlanetGravityZone.cpp — Implementation of spherical planet gravity zone.

#include "PlanetGravityZone.h"

APlanetGravityZone::APlanetGravityZone()
{
    // --- Inner sphere: display only, shows full-gravity radius ---
    InnerSphere = CreateDefaultSubobject<USphereComponent>(TEXT("InnerSphere"));
    SetRootComponent(InnerSphere);

    InnerSphere->SetSphereRadius(1.0f); // Scales with actor scale
    InnerSphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    InnerSphere->SetGenerateOverlapEvents(false);
    InnerSphere->SetHiddenInGame(true);
    InnerSphere->bVisualizeComponent = true; // Visible in editor for tuning

    // --- Outer sphere: overlap detection for gravity influence zone ---
    InfluenceSphere = CreateDefaultSubobject<USphereComponent>(TEXT("InfluenceSphere"));
    InfluenceSphere->SetupAttachment(InnerSphere);

    InfluenceSphere->SetSphereRadius(1.0f); // Will be sized to actor scale * InfluenceScale
    InfluenceSphere->SetCollisionProfileName(TEXT("OverlapAllDynamic"));
    InfluenceSphere->SetGenerateOverlapEvents(true);
    InfluenceSphere->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
    InfluenceSphere->SetHiddenInGame(true);
    InfluenceSphere->bVisualizeComponent = true;

    // Influence sphere should not inherit actor scale — we size it manually.
    InfluenceSphere->SetAbsolute(false, false, true);

    // Bind overlap events to forward into the base class GravPlugin notifications.
    InfluenceSphere->OnComponentBeginOverlap.AddDynamic(this, &APlanetGravityZone::OnInfluenceBeginOverlap);
    InfluenceSphere->OnComponentEndOverlap.AddDynamic(this, &APlanetGravityZone::OnInfluenceEndOverlap);

    // Default BaseVector: 980 cm/s² magnitude (direction unused for spherical gravity).
    BaseVector = FVector(0.0, 0.0, -980.0);
}

// ---------------------------------------------------------------------------
// Gravity calculation
// ---------------------------------------------------------------------------

FVector APlanetGravityZone::GetGravityVector_Implementation(const FVector& InWorldPosition) const
{
    FVector ToCenter = GetActorLocation() - InWorldPosition;
    double Distance = ToCenter.Size();

    if (Distance < KINDA_SMALL_NUMBER)
    {
        return FVector::ZeroVector;
    }

    FVector Direction = ToCenter / Distance;
    double MaxAcceleration = BaseVector.Size();

    // Inner radius = actor scale (the full-gravity boundary).
    double InnerRadius = GetActorScale3D().GetMax();
    // Outer radius = inner * InfluenceScale (the influence boundary).
    double OuterRadius = InnerRadius * InfluenceScale;

    if (Distance <= InnerRadius)
    {
        // Inside inner sphere: full gravity.
        return Direction * MaxAcceleration;
    }

    if (Distance >= OuterRadius)
    {
        // Outside influence zone: no gravity.
        return FVector::ZeroVector;
    }

    // Between inner and outer: ramp from 1.0 at inner edge to 0.0 at outer edge.
    // t = 0 at inner radius, t = 1 at outer radius.
    double t = (Distance - InnerRadius) / (OuterRadius - InnerRadius);
    // Invert so strength is 1 at inner, 0 at outer, then apply falloff exponent.
    double Strength = FMath::Pow(1.0 - t, Falloff);

    return Direction * MaxAcceleration * Strength;
}

// ---------------------------------------------------------------------------
// Planet initialization
// ---------------------------------------------------------------------------

void APlanetGravityZone::InitializeFromPlanet(double InPlanetRadius, double InAtmosphereScale)
{
    bIsPlanetOwned = true;

    // Cache location for the transform guard.
    PlanetDrivenLocation = GetActorLocation();

    // Update the influence sphere to match current scale * InfluenceScale.
    UpdateInfluenceSphereRadius();

    // Bind the transform guard so editor drags snap back.
    if (USceneComponent* Root = GetRootComponent())
    {
        Root->TransformUpdated.RemoveAll(this);
        Root->TransformUpdated.AddUObject(this, &APlanetGravityZone::OnTransformUpdated);
    }
}

void APlanetGravityZone::UpdateInfluenceSphereRadius()
{
    if (InfluenceSphere)
    {
        double InnerRadius = GetActorScale3D().GetMax();
        double OuterRadius = InnerRadius * InfluenceScale;

        // InfluenceSphere uses absolute scale, so we set the radius directly
        // in world units.
        InfluenceSphere->SetSphereRadius(OuterRadius);
    }
}

// ---------------------------------------------------------------------------
// Overlap forwarding
// ---------------------------------------------------------------------------

void APlanetGravityZone::OnInfluenceBeginOverlap(
    UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
    UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
    bool bFromSweep, const FHitResult& SweepResult)
{
    if (OtherActor && OtherActor != this)
    {
        OnZoneBeginOverlap(OtherActor);
    }
}

void APlanetGravityZone::OnInfluenceEndOverlap(
    UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
    UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{
    if (OtherActor && OtherActor != this)
    {
        OnZoneEndOverlap(OtherActor);
    }
}

// ---------------------------------------------------------------------------
// Transform locking (planet-owned)
// ---------------------------------------------------------------------------

void APlanetGravityZone::OnTransformUpdated(
    USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    // Snap location back — gravity zone must stay centered on the planet.
    if (!GetActorLocation().Equals(PlanetDrivenLocation, 0.01))
    {
        SetActorLocation(PlanetDrivenLocation);
    }
}

// ---------------------------------------------------------------------------
// Editor
// ---------------------------------------------------------------------------

#if WITH_EDITOR

void APlanetGravityZone::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    FName PropName = PropertyChangedEvent.GetPropertyName();

    // If InfluenceScale changed, resize the outer sphere.
    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetGravityZone, InfluenceScale))
    {
        UpdateInfluenceSphereRadius();
    }
}

bool APlanetGravityZone::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty)) return false;

    if (bIsPlanetOwned && InProperty)
    {
        FName PropName = InProperty->GetFName();
        // Lock transform-related properties when planet-owned.
        if (PropName == TEXT("RelativeLocation") || PropName == TEXT("RelativeScale3D"))
        {
            return false;
        }
    }

    return true;
}

void APlanetGravityZone::PostEditMove(bool bFinished)
{
    Super::PostEditMove(bFinished);

    if (bIsPlanetOwned)
    {
        SetActorLocation(PlanetDrivenLocation);
    }

    // Actor scale may have changed via gizmo — update influence sphere.
    UpdateInfluenceSphereRadius();
}

void APlanetGravityZone::EditorApplyTranslation(
    const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void APlanetGravityZone::EditorApplyScale(
    const FVector& DeltaScale, const FVector* PivotLocation,
    bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);

    // Scale changed — update influence sphere.
    UpdateInfluenceSphereRadius();
}

#endif