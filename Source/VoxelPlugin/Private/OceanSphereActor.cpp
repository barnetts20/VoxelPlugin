#include "OceanSphereActor.h"
#include "PlanetActor.h"
#include "VoxelPlugin.h"
#include "VoxelOceanSurface.h"
#include "VoxelTerrainStats.h"
#include "RealtimeMeshComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MaterialDomain.h"

DECLARE_CYCLE_STAT(TEXT("Ocean: actor tick"), STAT_VoxelOceanActorTick, STATGROUP_VoxelTerrain);

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    // Default scale = ocean radius matching APlanetActor defaults:
    // PlanetRadius (100M) + SeaLevel (0.5) * NoiseAmplitude (15M) = 107,500,000 cm.
    SetActorScale3D(FVector(107500000.0));

    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    // A planet's ocean is initialized by the planet (InitializeFromPlanet).
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
    if (!GetWorld() || GetWorld()->IsPreviewWorld()) return;
    if (!bInitialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        Initialize();
}

void AOceanSphereActor::BeginPlay()
{
    Super::BeginPlay();
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
    Initialize();
}

void AOceanSphereActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (Surface.IsValid())
        Surface->Reset();
    bInitialized = false;
    Super::EndPlay(EndPlayReason);
}

void AOceanSphereActor::Destroyed()
{
    if (Surface.IsValid())
        Surface->Reset();
    bInitialized = false;
    Super::Destroyed();
}

#if WITH_EDITOR
void AOceanSphereActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (!bInitialized) return;

    const FName PropName = PropertyChangedEvent.GetPropertyName();

    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, OceanMaterial))
    {
        SetOceanMaterial(OceanMaterial);
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, LodDistanceFactor))
    {
        if (PropertyChangedEvent.ChangeType != EPropertyChangeType::Interactive && Surface.IsValid())
            Surface->SetLodDistanceFactor(LodDistanceFactor);
        return;
    }

    // The mesh rebuilds; slider drags rebuild once, on release.
    if (PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive) return;
    static const TSet<FName> MeshProps = {
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, FinestSpacing),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, TileQuads),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, SpareBufferSets),
    };
    if (MeshProps.Contains(PropName))
    {
        InitializeInternal(Compositor);
        return;
    }

    // A standalone ocean's density follows the terrain's settings.
    const bool bDensityChanged = !bIsPlanetOwned && PropertyChangedEvent.MemberProperty
        && (PropertyChangedEvent.MemberProperty->GetFName() == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, SurfaceSettings)
            || PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, TerrainPlanetRadius)
            || PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, NoiseAmplitudeRatio));
    if (bDensityChanged)
        Initialize();
}

bool AOceanSphereActor::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty))
        return false;

    if (bIsPlanetOwned && InProperty)
    {
        const FName PropName = InProperty->GetFName();
        static const FName TransformNames[] = {
            TEXT("RelativeLocation"), TEXT("RelativeRotation"), TEXT("RelativeScale3D"),
        };
        for (const FName& Name : TransformNames)
        {
            if (PropName == Name) return false;
        }
    }

    return true;
}

void AOceanSphereActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::PostEditMove(bool bFinished)
{
    if (bIsPlanetOwned)
    {
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
            Root->SetRelativeRotation(FRotator::ZeroRotator);
        }
    }
    Super::PostEditMove(bFinished);
}
#endif

bool AOceanSphereActor::ShouldTickIfViewportsOnly() const
{
    return bInitialized;
}

UMaterialInterface* AOceanSphereActor::DefaultOceanMaterial()
{
    UMaterialInterface* PluginMaterial = Cast<UMaterialInterface>(StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
        TEXT("/VoxelPlugin/MT_Ocean.MT_Ocean"), nullptr, LOAD_NoWarn | LOAD_Quiet));
    return PluginMaterial ? PluginMaterial : UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

void AOceanSphereActor::SetOceanMaterial(UMaterialInterface* InMaterial)
{
    OceanMaterial = InMaterial ? InMaterial : DefaultOceanMaterial();
    if (!bInitialized) return;
    SurfaceMaterialInstance = UMaterialInstanceDynamic::Create(OceanMaterial, this);
    if (Surface.IsValid())
        Surface->SetMaterial(SurfaceMaterialInstance);
}

void AOceanSphereActor::Initialize()
{
    // Standalone: a density of its own from the terrain settings (no edits).
    const TSharedPtr<FDensitySampleCompositor> Own = VoxelDensity::MakeCompositor(SurfaceSettings, TerrainPlanetRadius,
        TerrainPlanetRadius * NoiseAmplitudeRatio, nullptr);
    InitializeInternal(Own);
}

void AOceanSphereActor::OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    if (USceneComponent* Root = GetRootComponent())
    {
        if (!Root->GetRelativeLocation().IsNearlyZero(0.01)
            || !Root->GetRelativeRotation().IsNearlyZero(0.01)
            || !Root->GetComponentScale().Equals(PlanetDrivenScale, 0.01))
        {
            Root->SetRelativeLocation_Direct(FVector::ZeroVector);
            Root->SetRelativeRotation_Direct(FRotator::ZeroRotator);
            Root->SetRelativeScale3D_Direct(PlanetDrivenScale);
            Root->UpdateComponentToWorld();
        }
    }
}

void AOceanSphereActor::InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
    USceneComponent* InAttachParent, FVector InScale)
{
    bIsPlanetOwned = true;

    // Apply scale BEFORE binding the transform guard, so the old guard doesn't revert a scale
    // change the planet intends.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.RemoveAll(this);

    if (!InScale.IsZero())
        SetActorScale3D(InScale);

    PlanetDrivenScale = GetActorScale3D();

    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.AddUObject(this, &AOceanSphereActor::OnTransformUpdated);

    // The pages follow the planet's position and rotation directly.
    if (InAttachParent && MeshAttachmentRoot)
    {
        MeshAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }
    InitializeInternal(InCompositor);
}

void AOceanSphereActor::CleanSceneRoot()
{
    if (!MeshAttachmentRoot) return;
    const TArray<TObjectPtr<USceneComponent>> Attached = MeshAttachmentRoot->GetAttachChildren();
    for (const TObjectPtr<USceneComponent>& Child : Attached)
    {
        if (URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Child))
            MeshComponent->DestroyComponent();
    }
}

void AOceanSphereActor::InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor)
{
    bInitialized = false;
    if (Surface.IsValid())
        Surface->Reset();
    CleanSceneRoot();
    Compositor = InCompositor;

    OceanRadius = GetActorScale3D().GetMax();
    // 8, 16 or 32 quads per tile edge.
    TileQuads = 1 << FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(TileQuads, 1))), 3, 5);
    LodDistanceFactor = FMath::Max(LodDistanceFactor, VoxelOceanTiles::MinLodDistanceFactor);
    MaxLevel = VoxelOceanTiles::LevelForSpacing(OceanRadius, TileQuads, FinestSpacing);
    ActualSpacing = VoxelOceanTiles::NominalArc(MaxLevel) * OceanRadius / TileQuads;

    if (!OceanMaterial)
        OceanMaterial = DefaultOceanMaterial();
    SurfaceMaterialInstance = UMaterialInstanceDynamic::Create(OceanMaterial, this);

    FVoxelOceanSurfaceSettings Settings;
    Settings.Radius = OceanRadius;
    Settings.TileQuads = TileQuads;
    Settings.MaxLevel = MaxLevel;
    Settings.LodDistanceFactor = LodDistanceFactor;
    Settings.SpareBufferSets = SpareBufferSets;
    Settings.Material = SurfaceMaterialInstance.Get();
    if (!Surface.IsValid())
        Surface = MakeShared<FVoxelOceanSurface>();
    Surface->Initialize(this, MeshAttachmentRoot, Settings);

    LastInitScale = GetActorScale3D();
    bInitialized = true;
}

void AOceanSphereActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanActorTick);
    Super::Tick(DeltaTime);
    if (!bInitialized || !Surface.IsValid()) return;

    if (UWorld* World = GetWorld())
    {
        const TArray<FVector>& Views = World->ViewLocationsRenderedLastFrame;
        if (Views.Num() > 0)
        {
            // The planet frame: position and rotation only (the surface is built at world scale).
            const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            CameraPosition = NoScaleTransform.InverseTransformPosition(Views[0]);
        }
    }
    Surface->Tick(CameraPosition);
}
