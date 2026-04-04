// PlanetGravityZone.cpp — C++ equivalent of BP_PlanetGravitySphere,
// using actor scale for the inner radius (same pattern as atmosphere actor).

#include "PlanetGravityZone.h"

APlanetGravityZone::APlanetGravityZone()
{
    // --- Root scene component (matches BP DefaultSceneRoot) ---
    DefaultSceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneRoot"));
    RootComponent = DefaultSceneRoot;

    // --- Inner sphere ("Planet"): display only ---
    // Unit radius; actor scale determines effective world radius.
    Planet = CreateDefaultSubobject<USphereComponent>(TEXT("Planet"));
    Planet->SetupAttachment(DefaultSceneRoot);
    Planet->SetSphereRadius(1.0f);
    Planet->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Planet->SetGenerateOverlapEvents(false);
    Planet->SetHiddenInGame(true);

    // --- Outer sphere ("Sphere"): overlap detection ---
    Sphere = CreateDefaultSubobject<USphereComponent>(TEXT("Sphere"));
    Sphere->SetupAttachment(DefaultSceneRoot);
    Sphere->SetSphereRadius(1.0f); // Sized properly in UpdateInfluenceSphereRadius
    Sphere->SetCollisionProfileName(TEXT("OverlapAllDynamic"));
    Sphere->SetGenerateOverlapEvents(true);
    Sphere->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
    Sphere->SetHiddenInGame(true);

    // Default BaseVector: 980 cm/s² magnitude (direction unused for spherical gravity).
    BaseVector = FVector(0.0, 0.0, -980.0);

    // Default inner radius: 1010 km = 101,000,000 cm (slightly above default
    // planet surface of 1000 km so full gravity is reached above ground).
    SetActorScale3D(FVector(101000000.0));
}

// ---------------------------------------------------------------------------
// Lifecycle — mirrors Construction Script + BeginPlay from the blueprint
// ---------------------------------------------------------------------------

void APlanetGravityZone::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    UpdateInfluenceSphereRadius();
}

void APlanetGravityZone::BeginPlay()
{
    Super::BeginPlay();

    // Bind actor overlap delegates — this is exactly what the BP
    // "Event ActorBeginOverlap" / "Event ActorEndOverlap" nodes bind to.
    OnActorBeginOverlap.AddDynamic(this, &APlanetGravityZone::OnActorOverlapBegin);
    OnActorEndOverlap.AddDynamic(this, &APlanetGravityZone::OnActorOverlapEnd);

    // Force overlap recalculation by resizing the sphere, matching what the
    // BP does by calling SetSphereRadius in both Construction Script and BeginPlay.
    if (Sphere)
    {
        Sphere->SetSphereRadius(0.0f, false);
    }
    UpdateInfluenceSphereRadius();

    // Explicit overlap update in case SetSphereRadius didn't trigger it
    if (Sphere)
    {
        Sphere->UpdateOverlaps();
    }

    // Register any actors already inside the zone — overlap events don't
    // fire for objects that were already overlapping when the component registered.
    TArray<AActor*> AlreadyOverlapping;
    GetOverlappingActors(AlreadyOverlapping);
    for (AActor* Actor : AlreadyOverlapping)
    {
        if (Actor && Actor != this)
        {
            OnZoneBeginOverlap(Actor);
        }
    }
}

// ---------------------------------------------------------------------------
// Sphere sizing
// ---------------------------------------------------------------------------

void APlanetGravityZone::UpdateInfluenceSphereRadius()
{
    if (Sphere)
    {
        // Sphere inherits actor scale. World radius = local radius * actor scale.
        // We want world radius = actor scale * InfluenceRatio,
        // so local radius = InfluenceRatio.
        Sphere->SetSphereRadius(InfluenceRatio, true);
    }
}

// ---------------------------------------------------------------------------
// Gravity calculation — matches BP_PlanetGravitySphere::GetGravityVector
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
    double Magnitude = BaseVector.Size();

    // Inner radius from actor scale.
    double InnerRadius = GetActorScale3D().GetMax();

    FVector Result;
    if (Distance > InnerRadius)
    {
        // BP does: Power(PlanetRadius / Distance, Falloff)
        // With positive Falloff (e.g. 2), this decays with distance since ratio < 1.
        double Ratio = InnerRadius / Distance;
        double FalloffMultiplier = FMath::Pow(Ratio, (double)Falloff);
        Result = Direction * Magnitude * FalloffMultiplier;
    }
    else
    {
        Result = Direction * Magnitude;
    }

    return Result;
}

// ---------------------------------------------------------------------------
// Planet initialization
// ---------------------------------------------------------------------------

void APlanetGravityZone::InitializeFromPlanet()
{
    bIsPlanetOwned = true;

    UpdateInfluenceSphereRadius();

    // Bind the transform guard so editor drags snap back to parent.
    if (USceneComponent* Root = GetRootComponent())
    {
        Root->TransformUpdated.RemoveAll(this);
        Root->TransformUpdated.AddUObject(this, &APlanetGravityZone::OnTransformUpdated);
    }
}

// ---------------------------------------------------------------------------
// Overlap forwarding — bound to OnActorBeginOverlap / OnActorEndOverlap
// delegates, which is exactly what BP "Event ActorBeginOverlap" binds to.
// ---------------------------------------------------------------------------

void APlanetGravityZone::OnActorOverlapBegin(AActor* OverlappedActor, AActor* OtherActor)
{
    if (OtherActor && OtherActor != this)
    {
        OnZoneBeginOverlap(OtherActor);
    }
}

void APlanetGravityZone::OnActorOverlapEnd(AActor* OverlappedActor, AActor* OtherActor)
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

    // Check relative location — if it's non-zero, someone dragged us away from
    // the parent. Snap back. This doesn't fire when the parent (planet) moves
    // because the relative offset stays at zero.
    if (USceneComponent* Root = GetRootComponent())
    {
        if (!Root->GetRelativeLocation().IsNearlyZero(0.01))
        {
            Root->SetRelativeLocation_Direct(FVector::ZeroVector);
            Root->UpdateComponentToWorld();
        }
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

    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetGravityZone, InfluenceRatio))
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
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
        }
    }

    // Scale may have changed via gizmo — update influence sphere.
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

    UpdateInfluenceSphereRadius();
}

#endif