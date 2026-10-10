#include "VoxelOceanPages.h"
#include "VoxelPlugin.h"
#include "VoxelGPUCommon.h"
#include "VoxelBrickTerrain.h"
#include "VoxelBrickMesherGPU.h"
#include "VoxelDensityGPU.h"
#include "VoxelTerrainStats.h"
#include "HAL/IConsoleManager.h"
#include "RealtimeMeshSimple.h"
#include "RealtimeMeshComponent.h"
#include "Compute/RealtimeMeshComputeProvider.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "Misc/ScopeExit.h"

DECLARE_CYCLE_STAT(TEXT("Ocean: page flush"), STAT_VoxelOceanFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: page commit"), STAT_VoxelOceanCommit, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: create buffer set"), STAT_VoxelOceanCreate, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: provider (render thread)"), STAT_VoxelOceanProvider, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Ocean: pages"), STAT_VoxelOceanPages, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Ocean: pooled buffer sets"), STAT_VoxelOceanPooled, STATGROUP_VoxelTerrain);
DECLARE_GPU_STAT(VoxelOceanPageWrite);

using namespace RealtimeMesh;

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanWriteParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, Jobs)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, VertexWords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, OutPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutArgs)
    SHADER_PARAMETER(uint32, NumJobs)
    SHADER_PARAMETER(uint32, TileQuads)
    SHADER_PARAMETER(uint32, DrawSlots)
    SHADER_PARAMETER(uint32, VertexCapacity)
    SHADER_PARAMETER(uint32, TriangleCapacity)
    SHADER_PARAMETER(uint32, Color)
    SHADER_PARAMETER(uint32, StreamMask)
    SHADER_PARAMETER(uint32, FlipWinding)
    // The density (VoxelDensitySample.ush) and the edits (VoxelEditSample.ush), as the
    // terrain's FVoxelDensityCS takes them.
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOctave>, Octaves)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, EditCorners)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, EditChildren)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditFlags)
    SHADER_PARAMETER(uint32, EditNodeCount)
    SHADER_PARAMETER(int32, EditRootExtent)
    SHADER_PARAMETER(FVector2f, HalfFinest)
    SHADER_PARAMETER(uint32, ContinentOctaves)
    SHADER_PARAMETER(uint32, MountainOctaves)
    SHADER_PARAMETER(uint32, DetailOctaves)
    SHADER_PARAMETER(FVector2f, InvNormContinent)
    SHADER_PARAMETER(FVector2f, InvNormMountain)
    SHADER_PARAMETER(FVector2f, InvNormDetail)
    SHADER_PARAMETER(FVector2f, PlanetRadius)
    SHADER_PARAMETER(FVector2f, NoiseAmplitude)
    SHADER_PARAMETER(FVector2f, ContinentWeight)
    SHADER_PARAMETER(FVector2f, MountainWeight)
    SHADER_PARAMETER(FVector2f, MountainStart)
    SHADER_PARAMETER(FVector2f, MountainBlend)
    SHADER_PARAMETER(FVector2f, DetailWeight)
    SHADER_PARAMETER(float, RidgeGain)
    SHADER_PARAMETER(float, SlopeDamping)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanWriteCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanWriteCS);
    using FParameters = FVoxelOceanWriteParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanWriteCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        // The density's double-float code needs exact float rounding (as FVoxelDensityCS).
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanWriteCS, "/Plugin/VoxelPlugin/Private/VoxelOceanPage.usf", "WriteCS", SF_Compute);

static TAutoConsoleVariable<int32> CVarVoxelOceanPageCreates(
    TEXT("voxel.Ocean.PageCreates"), 8,
    TEXT("Ocean page buffer sets created per frame at most (recycled ones don't count)."),
    ECVF_Default);

namespace VoxelOceanPagesDetail
{
    constexpr uint32 JobWords = 6;                 // VOXEL_OCEAN_JOB_WORDS (VoxelOceanPage.usf)
    constexpr int32 TexCoordChannels = 5;          // morph xy | morph z, depth | target depth, shore | grand xy | grand z, grand depth
    constexpr int32 MaxPooled = 48;                // emptied buffer sets kept for reuse
    constexpr uint64 KickFrames = 3;               // frames before a run that didn't happen is asked for again
    constexpr uint64 UnusedFrames = 120;           // an empty page no commit wanted is pooled after
    constexpr uint64 EmptyingTimeoutFrames = 60;   // an emptying run not recorded by then: destroyed instead
    constexpr int32 BytesPerVertex = 12 + 8 + 8 * TexCoordChannels + 4;
    constexpr int32 BytesPerTriangle = 12;

    const FName PageGroupName(TEXT("OCEANPAGE"));

    /** What a page's compute run lacked. Runs missing a Required item are skipped. */
    enum EMissing : uint32
    {
        Shaders = 1 << 0,
        Positions = 1 << 1,
        Tangents = 1 << 2,
        TexCoords = 1 << 3,
        Colors = 1 << 4,
        Triangles = 1 << 5,
        Args = 1 << 6,
        Capacity = 1 << 7,
        Required = Shaders | Positions | Triangles | Args | Capacity
    };

    FString DescribeMissing(uint32 Bits)
    {
        static const TCHAR* Names[] = { TEXT("page shader"), TEXT("Position stream"), TEXT("Tangents stream"), TEXT("TexCoords stream"),
            TEXT("Color stream"), TEXT("Triangles stream"), TEXT("indirect args"), TEXT("capacity") };
        FString Out;
        for (int32 i = 0; i < UE_ARRAY_COUNT(Names); i++)
        {
            if (Bits & (1u << i))
                Out += FString::Printf(TEXT("%s%s"), Out.IsEmpty() ? TEXT("") : TEXT(", "), Names[i]);
        }
        return Out;
    }

    int32 TileVertices(int32 Quads) { return (Quads + 1) * (Quads + 1); }
    int32 TileTriangles(int32 Quads) { return 2 * Quads * Quads; }

    FRealtimeMeshBufferSetKey GroupKey()
    {
        return FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), PageGroupName);
    }

    FRealtimeMeshSectionKey SectionKey()
    {
        return FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey(), 0);
    }

    bool AreShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        return ShaderMap && ShaderMap->GetShader(&FVoxelOceanWriteCS::GetStaticType(), 0).IsValid();
    }

    uint32 FloatBits(float Value)
    {
        uint32 Bits;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        return Bits;
    }

    /** As VoxelBrickPageGPU.usf's PackSnorm8: x, y, z and w in the four bytes. */
    uint32 PackSnorm8(const FVector3f& V, float W)
    {
        auto Byte = [](float X) { return (uint32)(FMath::RoundToInt(FMath::Clamp(X, -1.0f, 1.0f) * 127.0f) & 0xff); };
        return Byte(V.X) | (Byte(V.Y) << 8) | (Byte(V.Z) << 16) | (Byte(W) << 24);
    }
}

namespace VOP = VoxelOceanPagesDetail;

/** The density and edit inputs as graph buffers, uploaded once for every page's runs while
 *  they stay the same (render thread; as the terrain pages' GetInputs). */
class FVoxelOceanInputCache
{
public:
    VoxelBrickMesherGPU::FInputs Get(FRDGBuilder& GraphBuilder, const TSharedPtr<const FVoxelDensityGPUData>& InDensity,
        const TSharedPtr<const FVoxelEditGPUData>& InEdits)
    {
        VoxelBrickMesherGPU::FInputs Inputs;
        if (InDensity != Density || InEdits != Edits || !Octaves.IsValid())
        {
            Inputs = VoxelBrickMesherGPU::UploadInputs(GraphBuilder, *InDensity, *InEdits);
            Octaves = GraphBuilder.ConvertToExternalBuffer(Inputs.Octaves);
            EditCorners = GraphBuilder.ConvertToExternalBuffer(Inputs.EditCorners);
            EditChildren = GraphBuilder.ConvertToExternalBuffer(Inputs.EditChildren);
            EditFlags = GraphBuilder.ConvertToExternalBuffer(Inputs.EditFlags);
            Density = InDensity;
            Edits = InEdits;
            return Inputs;
        }
        Inputs.Octaves = GraphBuilder.RegisterExternalBuffer(Octaves);
        Inputs.EditCorners = GraphBuilder.RegisterExternalBuffer(EditCorners);
        Inputs.EditChildren = GraphBuilder.RegisterExternalBuffer(EditChildren);
        Inputs.EditFlags = GraphBuilder.RegisterExternalBuffer(EditFlags);
        Inputs.EditNodeCount = (uint32)Edits->Flags.Num();
        Inputs.EditRootExtent = Edits->RootExtent;
        return Inputs;
    }

private:
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TRefCountPtr<FRDGPooledBuffer> Octaves;
    TRefCountPtr<FRDGPooledBuffer> EditCorners;
    TRefCountPtr<FRDGPooledBuffer> EditChildren;
    TRefCountPtr<FRDGPooledBuffer> EditFlags;
};

/** Writes one buffer set's commits when the compute driver runs it (render thread). */
class FVoxelOceanPageProvider final : public IRealtimeMeshComputeProvider
{
public:
    /** A commit's work: VOP::JobWords per job and the written tiles' vertex words. */
    struct FState
    {
        TArray<uint32> Jobs;
        TArray<uint32> Words;
        uint32 DrawSlots = 0;
        uint32 TileQuads = 16;
        uint32 Color = 0xffffffffu;
        uint32 FlipWinding = 0;
        uint64 Version = 0;
        /** What the writes sample the depth from (never null). */
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        double HalfFinest = 50.0;
    };

    explicit FVoxelOceanPageProvider(const TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe>& InInputCache)
        : InputCache(InInputCache)
    {
    }

    /** States not yet run accumulate: each is written in order, so none is lost. */
    void AddState_RenderThread(FState&& InState)
    {
        States.Add(MoveTemp(InState));
    }

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    /** The newest state version recorded into a frame's graph. */
    std::atomic<uint64> ShownVersion{ 0 };
    /** Runs skipped (no shader or a required stream missing), runs recorded, and the OR of
     *  VOP::EMissing bits seen. */
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> Missing{ 0 };
    /** For voxel.Debug.Ocean, taken and reset by Flush. */
    std::atomic<uint32> Calls{ 0 };
    std::atomic<uint32> IdleCalls{ 0 };
    std::atomic<uint32> PerfRuns{ 0 };
    std::atomic<uint64> RenderMicros{ 0 };

private:
    TArray<FState> States;
    TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe> InputCache;
};

void FVoxelOceanPageProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanProvider);
    const uint64 StartCycles = FPlatformTime::Cycles64();
    ON_SCOPE_EXIT
    {
        RenderMicros.fetch_add((uint64)(FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles) * 1.0e6), std::memory_order_relaxed);
    };
    Calls.fetch_add(1, std::memory_order_relaxed);
    if (States.Num() == 0)
    {
        IdleCalls.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Every stream is written as 32-bit words.
    FRDGBufferUAVRef Positions = Context.GetStreamUAV(FRealtimeMeshStreams::Position, PF_R32_FLOAT);
    FRDGBufferUAVRef Tangents = Context.GetStreamUAV(FRealtimeMeshStreams::Tangents, PF_R32_UINT);
    FRDGBufferUAVRef TexCoords = Context.GetStreamUAV(FRealtimeMeshStreams::TexCoords, PF_R32_UINT);
    FRDGBufferUAVRef Colors = Context.GetStreamUAV(FRealtimeMeshStreams::Color, PF_R32_UINT);
    FRDGBufferUAVRef Triangles = Context.GetStreamUAV(FRealtimeMeshStreams::Triangles, PF_R32_UINT);
    FRDGBufferUAVRef Args = Context.GetIndirectArgsUAV();
    uint32 MissingNow = 0;
    MissingNow |= VOP::AreShadersReady() ? 0u : (uint32)VOP::Shaders;
    MissingNow |= Positions ? 0u : (uint32)VOP::Positions;
    MissingNow |= Tangents ? 0u : (uint32)VOP::Tangents;
    MissingNow |= TexCoords ? 0u : (uint32)VOP::TexCoords;
    MissingNow |= Colors ? 0u : (uint32)VOP::Colors;
    MissingNow |= Triangles ? 0u : (uint32)VOP::Triangles;
    MissingNow |= Args ? 0u : (uint32)VOP::Args;
    if (Context.GetVertexCapacity() <= 0 || Context.GetIndexCapacity() <= 0)
        MissingNow |= (uint32)VOP::Capacity;
    Missing.fetch_or(MissingNow, std::memory_order_relaxed);
    if (MissingNow & (uint32)VOP::Required)
    {
        // Kept for the next run (the game thread asks again after VOP::KickFrames).
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Runs.fetch_add(1, std::memory_order_relaxed);
    PerfRuns.fetch_add(1, std::memory_order_relaxed);

    // Streams the driver doesn't expose are written to a scratch word and left alone.
    const uint32 StreamMask = (Tangents ? 1u : 0u) | (TexCoords ? 2u : 0u) | (Colors ? 4u : 0u);
    auto Scratch = [&GraphBuilder]()
        {
            return GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 1), TEXT("Voxel.OceanScratch")), PF_R32_UINT);
        };
    Tangents = Tangents ? Tangents : Scratch();
    TexCoords = TexCoords ? TexCoords : Scratch();
    Colors = Colors ? Colors : Scratch();

    const uint32 VertexCapacity = (uint32)FMath::Max(0, Context.GetVertexCapacity());
    const uint32 TriangleCapacity = (uint32)FMath::Max(0, Context.GetIndexCapacity() / 3);

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanPageWrite, "VoxelOceanPage write");
    TShaderMapRef<FVoxelOceanWriteCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    for (const FState& State : States)
    {
        const uint32 NumJobs = (uint32)State.Jobs.Num() / VOP::JobWords;
        const FVoxelDensityGPUData& Density = *State.Density;
        const VoxelBrickMesherGPU::FInputs Inputs = InputCache->Get(GraphBuilder, State.Density, State.Edits);
        FVoxelOceanWriteParameters* Parameters = GraphBuilder.AllocParameters<FVoxelOceanWriteParameters>();
        Parameters->Octaves = GraphBuilder.CreateSRV(Inputs.Octaves);
        Parameters->EditCorners = GraphBuilder.CreateSRV(Inputs.EditCorners);
        Parameters->EditChildren = GraphBuilder.CreateSRV(Inputs.EditChildren);
        Parameters->EditFlags = GraphBuilder.CreateSRV(Inputs.EditFlags);
        Parameters->EditNodeCount = Inputs.EditNodeCount;
        Parameters->EditRootExtent = Inputs.EditRootExtent;
        Parameters->HalfFinest = VoxelDoubleFloat(State.HalfFinest);
        Parameters->ContinentOctaves = Density.ContinentOctaves;
        Parameters->MountainOctaves = Density.MountainOctaves;
        Parameters->DetailOctaves = Density.DetailOctaves;
        Parameters->InvNormContinent = Density.InvNormContinent;
        Parameters->InvNormMountain = Density.InvNormMountain;
        Parameters->InvNormDetail = Density.InvNormDetail;
        Parameters->PlanetRadius = Density.PlanetRadius;
        Parameters->NoiseAmplitude = Density.NoiseAmplitude;
        Parameters->ContinentWeight = Density.ContinentWeight;
        Parameters->MountainWeight = Density.MountainWeight;
        Parameters->MountainStart = Density.MountainStart;
        Parameters->MountainBlend = Density.MountainBlend;
        Parameters->DetailWeight = Density.DetailWeight;
        Parameters->RidgeGain = Density.RidgeGain;
        Parameters->SlopeDamping = Density.SlopeDamping;
        Parameters->Jobs = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.OceanJobs"), State.Jobs));
        Parameters->VertexWords = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.OceanVertexWords"), State.Words));
        Parameters->OutPositions = Positions;
        Parameters->OutTangents = Tangents;
        Parameters->OutTexCoords = TexCoords;
        Parameters->OutColors = Colors;
        Parameters->OutIndices = Triangles;
        Parameters->OutArgs = Args;
        Parameters->NumJobs = NumJobs;
        Parameters->TileQuads = State.TileQuads;
        Parameters->DrawSlots = State.DrawSlots;
        Parameters->VertexCapacity = VertexCapacity;
        Parameters->TriangleCapacity = TriangleCapacity;
        Parameters->Color = State.Color;
        Parameters->StreamMask = StreamMask;
        Parameters->FlipWinding = State.FlipWinding;
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanPage write %u jobs", NumJobs), Shader, Parameters,
            FIntVector((int32)FMath::Max(1u, NumJobs), 1, 1));
    }
    ShownVersion.store(States.Last().Version, std::memory_order_release);
    States.Reset();
}

FVoxelOceanPages::FVoxelOceanPages()
    : NoDensity(MakeShared<FVoxelDensityGPUData>())
    , NoEdits(MakeShared<FVoxelEditGPUData>())
    , InputCache(MakeShared<FVoxelOceanInputCache, ESPMode::ThreadSafe>())
{
}

FVoxelOceanPages::~FVoxelOceanPages()
{
    Reset();
    // The cached graph buffers are let go on the render thread (the providers may still hold
    // the cache there too).
    ENQUEUE_RENDER_COMMAND(VoxelOceanReleaseInputs)([Cache = MoveTemp(InputCache)](FRHICommandListImmediate&) mutable
        {
            Cache.Reset();
        });
}

void FVoxelOceanPages::SetDensityInputs(const FVoxelOceanDensityInputs& InInputs)
{
    DensityInputs = InInputs;
}

void FVoxelOceanPages::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, UMaterialInterface* InMaterial, double InRadius, int32 InTileQuads,
    double InMorphEnd0, bool bInFlipWinding)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Material = InMaterial;
    Radius = InRadius;
    TileQuads = InTileQuads;
    MorphEnd0 = InMorphEnd0;
    bFlipWinding = bInFlipWinding;
}

void FVoxelOceanPages::Reset()
{
    for (TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        DestroyInstance(Pair.Value.Instance);
    Pages.Reset();
    for (FInstance& Instance : Pool)
        DestroyInstance(Instance);
    Pool.Reset();
    for (FReleasing& Entry : Releasing)
        DestroyInstance(Entry.Instance);
    Releasing.Reset();
}

void FVoxelOceanPages::SetMaterial(UMaterialInterface* InMaterial)
{
    Material = InMaterial;
    auto Apply = [InMaterial](const FInstance& Instance)
        {
            if (URealtimeMeshComponent* Component = Instance.Component.Get())
                Component->SetMaterial(0, InMaterial);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        Apply(Pair.Value.Instance);
    for (const FInstance& Instance : Pool)
        Apply(Instance);
    for (const FReleasing& Entry : Releasing)
        Apply(Entry.Instance);
}

void FVoxelOceanPages::SetMorphEnd0(double InMorphEnd0)
{
    MorphEnd0 = InMorphEnd0;
    auto Apply = [this](const FInstance& Instance)
        {
            if (URealtimeMeshComponent* Component = Instance.Component.Get())
                Component->SetCustomPrimitiveDataFloat(1, (float)MorphEnd0);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        Apply(Pair.Value.Instance);
    for (const FInstance& Instance : Pool)
        Apply(Instance);
    for (const FReleasing& Entry : Releasing)
        Apply(Entry.Instance);
}

void FVoxelOceanPages::PackVertex(const VoxelOceanTiles::FTileVertex& Vertex, const FIntVector& Lattice, uint32* OutWords)
{
    OutWords[0] = VOP::FloatBits(Vertex.Position.X);
    OutWords[1] = VOP::FloatBits(Vertex.Position.Y);
    OutWords[2] = VOP::FloatBits(Vertex.Position.Z);
    OutWords[3] = VOP::FloatBits(Vertex.Morph.X);
    OutWords[4] = VOP::FloatBits(Vertex.Morph.Y);
    OutWords[5] = VOP::FloatBits(Vertex.Morph.Z);
    OutWords[6] = VOP::PackSnorm8(Vertex.Tangent, 1.0f);
    OutWords[7] = VOP::PackSnorm8(Vertex.Normal, 1.0f);
    OutWords[8] = VOP::FloatBits(Vertex.Grand.X);
    OutWords[9] = VOP::FloatBits(Vertex.Grand.Y);
    OutWords[10] = VOP::FloatBits(Vertex.Grand.Z);
    OutWords[11] = (uint32)Lattice.X;
    OutWords[12] = (uint32)Lattice.Y;
    OutWords[13] = (uint32)Lattice.Z;
    OutWords[14] = ((uint32)Vertex.TargetIndex & 0xffffu) | ((uint32)Vertex.GrandIndex << 16);
}

FVoxelOceanPages::FInstance FVoxelOceanPages::CreateInstance()
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanCreate);
    const double StartTime = FPlatformTime::Seconds();
    ON_SCOPE_EXIT
    {
        const double Ms = (FPlatformTime::Seconds() - StartTime) * 1000.0;
        Perf.Created++;
        Perf.CreateMs += Ms;
        Perf.MaxCreateMs = FMath::Max(Perf.MaxCreateMs, Ms);
    };
    FInstance Instance;
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return Instance;

    const int32 VertexCapacity = VoxelOceanTiles::PageSlots * VOP::TileVertices(TileQuads);
    const int32 TriangleCapacity = VoxelOceanTiles::PageSlots * VOP::TileTriangles(TileQuads);

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Ocean Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->SetCastShadow(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetVisibleInRayTracing(false);
    // For the morph (VoxelOcean.ush): where level 0's morph ends in world units. Data 0, the
    // level, is set with the page (PlaceInstance).
    Component->SetCustomPrimitiveDataFloat(1, (float)MorphEnd0);

    // The uploaded streams only size the buffers; the provider writes them. Two corner
    // positions give the bounds: in page units a page's tiles and their morph stay within the
    // unit cube around its origin (its node's half-width is about 0.5, a face sags 0.27).
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(VertexCapacity);
    TArrayView<FVector3f> PositionView = Positions.GetArrayView<FVector3f>();
    PositionView[0] = FVector3f(-1.0f);
    PositionView[1] = FVector3f(1.0f);
    // Nothing draws these before the provider writes them: no need to clear them here.
    Streams.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<TRealtimeMeshTexCoords<FVector2f, VOP::TexCoordChannels>>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()).SetNumUninitialized(VertexCapacity);
    // Zeroed: slots never written draw degenerate triangles.
    Streams.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()).SetNumZeroed(TriangleCapacity);
    // No PolyGroups stream: the buffer set then makes one section over every vertex and index.

    Instance.bBuffersReady = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
    const TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> Ready = Instance.bBuffersReady;
    Mesh->CreateBufferSet(VOP::GroupKey(), MoveTemp(Streams), FRealtimeMeshBufferSetConfig(ERealtimeMeshSectionDrawType::Static, true))
        .Next([Ready](ERealtimeMeshProxyUpdateStatus) { Ready->store(true, std::memory_order_release); });
    Mesh->UpdateSectionConfig(VOP::SectionKey(), FRealtimeMeshSectionConfig(0), false);
    // Hidden until the provider is registered: until then the section draws its whole (zero)
    // range. The component stays visible: a hidden component has no render proxy, and the
    // driver runs providers only for meshes with one.
    Mesh->SetSectionVisibility(VOP::SectionKey(), false);

    Instance.Mesh = Mesh;
    Instance.Component = Component;
    Instance.Provider = MakeShared<FVoxelOceanPageProvider, ESPMode::ThreadSafe>(InputCache);
    Instance.CreatedFrame = GFrameCounter;
    return Instance;
}

void FVoxelOceanPages::DestroyInstance(FInstance& Instance)
{
    if (Instance.Handle != 0)
        UnregisterRealtimeMeshComputeProvider(Instance.Handle);
    if (URealtimeMeshComponent* Component = Instance.Component.Get())
        Component->DestroyComponent();
    Instance = FInstance();
}

void FVoxelOceanPages::PlaceInstance(const FVoxelOceanTileKey& PageKey, FInstance& Instance) const
{
    URealtimeMeshComponent* Component = Instance.Component.Get();
    if (!Component) return;
    FVector Origin;
    double Unit = 1.0;
    VoxelOceanTiles::PageFrame(PageKey, Radius, Origin, Unit);
    Component->SetRelativeTransform(FTransform(FQuat::Identity, Origin, FVector(Unit)));
    Component->SetCustomPrimitiveDataFloat(0, (float)PageKey.Level);
}

bool FVoxelOceanPages::IsReady(const FInstance& Instance)
{
    return Instance.IsValid() && Instance.Handle != 0 && GFrameCounter > Instance.RegisteredFrame && GFrameCounter >= Instance.ReadyFrame;
}

void FVoxelOceanPages::BeginCreates()
{
    if (CreatesFrame == GFrameCounter) return;
    CreatesFrame = GFrameCounter;
    CreatesThisFrame = 0;
}

FVoxelOceanPages::FInstance FVoxelOceanPages::AcquireInstance(const FVoxelOceanTileKey& PageKey)
{
    while (Pool.Num() > 0)
    {
        FInstance Instance = Pool.Pop();
        if (!Instance.Component.IsValid() || !Instance.Mesh.IsValid())
        {
            DestroyInstance(Instance);
            continue;
        }
        // Moved now, filled from the next frame on, so the move reaches the render thread no
        // later than the new contents.
        PlaceInstance(PageKey, Instance);
        Instance.ReadyFrame = GFrameCounter + 1;
        Perf.Recycled++;
        return Instance;
    }
    if (CreatesThisFrame >= FMath::Max(1, CVarVoxelOceanPageCreates.GetValueOnGameThread()))
        return FInstance();
    CreatesThisFrame++;
    // A new one is registered with empty contents, so it is ready when a commit needs it.
    FInstance Instance = CreateInstance();
    if (Instance.IsValid())
    {
        PlaceInstance(PageKey, Instance);
        Push(PageKey, Instance, TArray<uint32>(), TArray<uint32>(), 0);
    }
    return Instance;
}

void FVoxelOceanPages::PoolInstance(FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    if (!Instance.Component.IsValid() || Pool.Num() >= VOP::MaxPooled)
    {
        DestroyInstance(Instance);
        return;
    }
    Pool.Add(MoveTemp(Instance));
    Instance = FInstance();
}

bool FVoxelOceanPages::PreparePages(const TSet<FVoxelOceanTileKey>& InPages)
{
    BeginCreates();
    bool bReady = true;
    for (const FVoxelOceanTileKey& PageKey : InPages)
    {
        FPage& Page = Pages.FindOrAdd(PageKey);
        Page.WantedFrame = GFrameCounter;
        if (!Page.Instance.IsValid())
            Page.Instance = AcquireInstance(PageKey);
        bReady &= IsReady(Page.Instance);
    }
    return bReady;
}

bool FVoxelOceanPages::IsPageReady(const FVoxelOceanTileKey& PageKey) const
{
    const FPage* Page = Pages.Find(PageKey);
    return Page && IsReady(Page->Instance);
}

int32 FVoxelOceanPages::FindSlot(const FPage& Page, const FVoxelOceanTileKey& Tile)
{
    for (int32 s = 0; s < VoxelOceanTiles::PageSlots; s++)
    {
        if (Page.Slots[s] == Tile) return s;
    }
    return INDEX_NONE;
}

int32 FVoxelOceanPages::DrawSlotsOf(const FPage& Page)
{
    for (int32 s = VoxelOceanTiles::PageSlots - 1; s >= 0; s--)
    {
        if (Page.Slots[s] != FreeSlot()) return s + 1;
    }
    return 0;
}

void FVoxelOceanPages::Commit(TConstArrayView<FTileWrite> Writes, TConstArrayView<FVoxelOceanTileKey> Clears)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanCommit);
    struct FPageWork
    {
        uint32 Cleared = 0;            // slots emptied by this commit and not refilled
        TArray<uint32> Jobs;
        TArray<uint32> Words;
    };
    TMap<FVoxelOceanTileKey, FPageWork> Work;

    // Departing tiles free their slots first, so new tiles of the same commit can take them.
    for (const FVoxelOceanTileKey& Tile : Clears)
    {
        const FVoxelOceanTileKey PageKey = VoxelOceanTiles::PageOf(Tile);
        FPage* Page = Pages.Find(PageKey);
        const int32 Slot = Page ? FindSlot(*Page, Tile) : INDEX_NONE;
        if (Slot == INDEX_NONE) continue;
        Page->Slots[Slot] = FreeSlot();
        Page->NumTiles--;
        Work.FindOrAdd(PageKey).Cleared |= 1u << Slot;
        Perf.TilesCleared++;
    }

    // New tiles: the lowest free slot (a tile already held is rewritten in place). A slot
    // freed and refilled in this commit is only written: one job per slot, since the jobs of
    // a run go in parallel.
    for (const FTileWrite& Write : Writes)
    {
        const FVoxelOceanTileKey PageKey = VoxelOceanTiles::PageOf(Write.Tile);
        FPage* Page = Pages.Find(PageKey);
        if (!Page || !Page->Instance.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] A tile was committed to a page without a buffer set; it is not drawn until it changes."));
            continue;
        }
        int32 Slot = FindSlot(*Page, Write.Tile);
        if (Slot == INDEX_NONE)
        {
            Slot = FindSlot(*Page, FreeSlot());
            if (Slot == INDEX_NONE)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] A page has no free slot for a tile; it is not drawn."));
                continue;
            }
            Page->Slots[Slot] = Write.Tile;
            Page->NumTiles++;
        }
        FPageWork& PageWork = Work.FindOrAdd(PageKey);
        PageWork.Cleared &= ~(1u << Slot);
        PageWork.Jobs.Append({ 0u, (uint32)Slot, (uint32)PageWork.Words.Num(), VOP::FloatBits(Write.Footprint), VOP::FloatBits(Write.CullHeight), 0u });
        PageWork.Words.Append(Write.Words);
        Perf.TilesWritten++;
    }

    for (TPair<FVoxelOceanTileKey, FPageWork>& Pair : Work)
    {
        FPage* Page = Pages.Find(Pair.Key);
        if (!Page) continue;
        FPageWork& PageWork = Pair.Value;
        for (int32 s = 0; s < VoxelOceanTiles::PageSlots; s++)
        {
            if (PageWork.Cleared & (1u << s))
                PageWork.Jobs.Append({ 1u, (uint32)s, 0u, 0u, 0u, 0u });
        }
        Push(Pair.Key, Page->Instance, MoveTemp(PageWork.Jobs), MoveTemp(PageWork.Words), DrawSlotsOf(*Page));
        Perf.PagesPushed++;
        if (Page->NumTiles > 0) continue;
        // Emptied by this run: pooled once it is recorded.
        FReleasing& Entry = Releasing.AddDefaulted_GetRef();
        Entry.Instance = MoveTemp(Page->Instance);
        Entry.Version = Entry.Instance.PushedVersion;
        Entry.Frame = GFrameCounter;
        Pages.Remove(Pair.Key);
    }
    Perf.Commits++;
}

void FVoxelOceanPages::Push(const FVoxelOceanTileKey& PageKey, FInstance& Instance, TArray<uint32>&& Jobs, TArray<uint32>&& Words, int32 DrawSlots)
{
    if (!Instance.IsValid()) return;
    FVoxelOceanPageProvider::FState State;
    State.Jobs = MoveTemp(Jobs);
    State.Words = MoveTemp(Words);
    State.DrawSlots = (uint32)FMath::Max(0, DrawSlots);
    State.TileQuads = (uint32)TileQuads;
    State.Color = (CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0 ? FVoxelBrickTerrain::LevelColor(PageKey.Level) : FColor::White).DWColor();
    State.FlipWinding = bFlipWinding ? 1u : 0u;
    State.Density = DensityInputs.Density.IsValid() ? DensityInputs.Density : NoDensity;
    State.Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
    State.HalfFinest = DensityInputs.HalfFinest;
    State.Version = NextVersion++;
    Instance.PushedVersion = State.Version;
    // The state reaches the provider before the dirty mark: both are render commands.
    ENQUEUE_RENDER_COMMAND(VoxelOceanPagePush)([Provider = Instance.Provider, State = MoveTemp(State)](FRHICommandListImmediate&) mutable
        {
            Provider->AddState_RenderThread(MoveTemp(State));
        });
    if (Instance.Handle != 0)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelOceanPages::Advance(FInstance& Instance) const
{
    if (!Instance.IsValid()) return;
    URealtimeMeshSimple* Mesh = Instance.Mesh.Get();
    if (!Mesh) return;

    if (Instance.Handle == 0)
    {
        // Registered once the buffers exist and the component has a proxy (two frames after
        // its creation); the driver runs an OnDemand provider when registered.
        if (!Instance.bBuffersReady->load(std::memory_order_acquire) || GFrameCounter < Instance.CreatedFrame + 2 || Instance.PushedVersion == 0)
            return;
        // Shown first: the driver binds the indirect args to the section proxy, and a later
        // section update from the data side would rebuild that proxy without them.
        Mesh->SetSectionVisibility(VOP::SectionKey(), true);
        FRealtimeMeshComputeRegistration Registration;
        Registration.Mesh = Mesh->GetMesh();
        Registration.BufferSetKey = VOP::GroupKey();
        Registration.SectionKey = VOP::SectionKey();
        Registration.Provider = Instance.Provider;
        Registration.Trigger = ERealtimeMeshComputeTrigger::OnDemand;
        Registration.bUseIndirect = true;
        Instance.Handle = RegisterRealtimeMeshComputeProvider(Registration);
        Instance.LastKickFrame = GFrameCounter;
        Instance.RegisteredFrame = GFrameCounter;
        return;
    }

    // A run that didn't happen (no proxy yet, shader still compiling) is asked for again.
    if (Instance.Provider->ShownVersion.load(std::memory_order_acquire) < Instance.PushedVersion && GFrameCounter > Instance.LastKickFrame + VOP::KickFrames)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelOceanPages::CollectCounters(const FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    FVoxelOceanPageProvider& Provider = *Instance.Provider;
    Perf.Calls += Provider.Calls.exchange(0, std::memory_order_relaxed);
    Perf.IdleCalls += Provider.IdleCalls.exchange(0, std::memory_order_relaxed);
    Perf.Runs += Provider.PerfRuns.exchange(0, std::memory_order_relaxed);
    Perf.RenderMicros += Provider.RenderMicros.exchange(0, std::memory_order_relaxed);
}

void FVoxelOceanPages::Flush(int32 SpareTarget)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanFlush);
    Perf.Frames++;
    BeginCreates();

    // Emptied buffer sets, pooled once their emptying run is recorded.
    for (int32 r = 0; r < Releasing.Num(); r++)
    {
        FReleasing& Entry = Releasing[r];
        Advance(Entry.Instance);
        CollectCounters(Entry.Instance);
        if (Entry.Instance.IsValid() && Entry.Instance.Provider->ShownVersion.load(std::memory_order_acquire) >= Entry.Version)
            PoolInstance(Entry.Instance);
        else if (!Entry.Instance.IsValid() || GFrameCounter > Entry.Frame + VOP::EmptyingTimeoutFrames)
            DestroyInstance(Entry.Instance);
        else
            continue;
        Releasing.RemoveAtSwap(r--);
    }

    // Registration and kicks; empty pages no commit wanted lately go back to the pool.
    TArray<FVoxelOceanTileKey> Unused;
    for (TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
    {
        FPage& Page = Pair.Value;
        Advance(Page.Instance);
        CollectCounters(Page.Instance);
        const uint32 MissingBits = Page.Instance.IsValid() ? Page.Instance.Provider->Missing.load(std::memory_order_relaxed) : 0u;
        if (MissingBits != 0 && (MissingBits & ~WarnedMissing) != 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] An ocean page's compute run is missing: %s%s."), *VOP::DescribeMissing(MissingBits),
                (MissingBits & (uint32)VOP::Required) ? TEXT(" (run skipped, retried)") : TEXT(" (written elsewhere, run continues)"));
            WarnedMissing |= MissingBits;
        }
        if (Page.NumTiles == 0 && GFrameCounter > Page.WantedFrame + VOP::UnusedFrames)
            Unused.Add(Pair.Key);
    }
    for (const FVoxelOceanTileKey& Key : Unused)
    {
        PoolInstance(Pages[Key].Instance);
        Pages.Remove(Key);
    }

    // Spare buffer sets, registered ahead so commits don't wait for creation.
    for (FInstance& Instance : Pool)
    {
        Advance(Instance);
        CollectCounters(Instance);
    }
    const int32 Budget = FMath::Max(1, CVarVoxelOceanPageCreates.GetValueOnGameThread());
    while (Pool.Num() < FMath::Min(SpareTarget, VOP::MaxPooled) && CreatesThisFrame < Budget)
    {
        CreatesThisFrame++;
        FInstance Instance = CreateInstance();
        if (!Instance.IsValid()) break;
        Push(FVoxelOceanTileKey(), Instance, TArray<uint32>(), TArray<uint32>(), 0);
        Pool.Add(MoveTemp(Instance));
    }

    SET_DWORD_STAT(STAT_VoxelOceanPages, Pages.Num());
    SET_DWORD_STAT(STAT_VoxelOceanPooled, Pool.Num());
}

FString FVoxelOceanPages::Describe() const
{
    int32 Tiles = 0, Unregistered = 0, Lagging = 0;
    uint32 Runs = 0, Skipped = 0, MissingBits = 0;
    auto Count = [&](const FInstance& Instance)
        {
            if (!Instance.IsValid()) return;
            Unregistered += Instance.Handle == 0 ? 1 : 0;
            Lagging += Instance.Provider->ShownVersion.load(std::memory_order_relaxed) < Instance.PushedVersion ? 1 : 0;
            Runs += Instance.Provider->Runs.load(std::memory_order_relaxed);
            Skipped += Instance.Provider->SkippedRuns.load(std::memory_order_relaxed);
            MissingBits |= Instance.Provider->Missing.load(std::memory_order_relaxed);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
    {
        Tiles += Pair.Value.NumTiles;
        Count(Pair.Value.Instance);
    }
    for (const FInstance& Instance : Pool)
        Count(Instance);
    const int32 Instances = Pages.Num() + Pool.Num() + Releasing.Num();
    const double MB = Instances * (double)VoxelOceanTiles::PageSlots
        * (VOP::TileVertices(TileQuads) * VOP::BytesPerVertex + VOP::TileTriangles(TileQuads) * VOP::BytesPerTriangle) / (1024.0 * 1024.0);
    return FString::Printf(TEXT("Ocean pages: %d (%d tiles), %d pooled, %d being emptied, %.1f MB of buffers; %d not yet registered, %d not yet drawing their newest state; %u runs, %u skipped%s%s"),
        Pages.Num(), Tiles, Pool.Num(), Releasing.Num(), MB, Unregistered, Lagging, Runs, Skipped,
        MissingBits ? TEXT(", missing: ") : TEXT(""), *VOP::DescribeMissing(MissingBits));
}

FString FVoxelOceanPages::TakePerf()
{
    const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
    const FString Out = FString::Printf(TEXT("Ocean pages per frame: %.2f commits, %.1f tiles written, %.1f cleared, %.1f pages pushed; providers %.1f calls (%.1f idle), %.2f runs, %.3f ms render thread; buffer sets %.2f created (%.2f ms each, %.2f ms max), %.2f recycled."),
        Perf.Commits * PerFrame, Perf.TilesWritten * PerFrame, Perf.TilesCleared * PerFrame, Perf.PagesPushed * PerFrame,
        Perf.Calls * PerFrame, Perf.IdleCalls * PerFrame, Perf.Runs * PerFrame, Perf.RenderMicros * PerFrame / 1000.0,
        Perf.Created * PerFrame, Perf.Created > 0 ? Perf.CreateMs / Perf.Created : 0.0, Perf.MaxCreateMs, Perf.Recycled * PerFrame);
    Perf = FPerf();
    return Out;
}
