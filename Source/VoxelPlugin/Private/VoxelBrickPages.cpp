#include "VoxelBrickPages.h"
#include "FAdaptiveOctree.h"
#include "RealtimeMeshSimple.h"
#include "RealtimeMeshComponent.h"
#include "Compute/RealtimeMeshComputeProvider.h"
#include "GameFramework/Actor.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"

using namespace RealtimeMesh;

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelPageParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherCounters)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, MesherVertices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherIndices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, BrickOffsets)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, PageState)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, BrickBase)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, OutPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutArgs)
    SHADER_PARAMETER(uint32, NumBricks)
    SHADER_PARAMETER(uint32, VertexCap)
    SHADER_PARAMETER(uint32, TriangleCap)
    SHADER_PARAMETER(uint32, VertexCapacity)
    SHADER_PARAMETER(uint32, TriangleCapacity)
    SHADER_PARAMETER(uint32, Color)
    SHADER_PARAMETER(uint32, StreamMask)
    SHADER_PARAMETER(FVector3f, UVOffset)
    SHADER_PARAMETER(float, UVScale)
    SHADER_PARAMETER(uint32, GroupsX)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_PAGE_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelPageParameters; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
    };

VOXEL_PAGE_SHADER(FVoxelPageOffsetsCS)
VOXEL_PAGE_SHADER(FVoxelPageVerticesCS)
VOXEL_PAGE_SHADER(FVoxelPageTrianglesCS)
VOXEL_PAGE_SHADER(FVoxelPageArgsCS)
#undef VOXEL_PAGE_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelPageOffsetsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "OffsetsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageVerticesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "VerticesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageTrianglesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "TrianglesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageArgsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "ArgsCS", SF_Compute);

namespace VoxelBrickPagesDetail
{
    constexpr int32 PageShift = 2;                   // pages of 4x4x4 bricks
    constexpr int32 MinVertices = 4096;
    constexpr int32 MinTriangles = 8192;
    constexpr int32 MaxVertices = 1 << 22;
    constexpr int32 MaxTriangles = 1 << 22;
    constexpr uint64 KickFrames = 30;                // frames before a run that didn't happen is asked for again
    constexpr int32 MaxPendingReadbacks = 4;
    constexpr int32 BytesPerVertex = 28;             // position, two packed normals, half2 UV, colour
    constexpr int32 BytesPerTriangle = 12;

    const FName PageGroupName(TEXT("BRICKPAGE"));

    int64 FloorShift(int64 Value, int32 Levels)
    {
        return Value >= 0 ? (Value >> Levels) : -((-Value + (1LL << Levels) - 1) >> Levels);
    }

    FRealtimeMeshBufferSetKey GroupKey()
    {
        return FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), PageGroupName);
    }

    FRealtimeMeshSectionKey SectionKey()
    {
        return FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey(), 0);
    }

    bool ArePageShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        return ShaderMap
            && ShaderMap->GetShader(&FVoxelPageOffsetsCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageVerticesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageTrianglesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageArgsCS::GetStaticType(), 0).IsValid();
    }

    template<typename TShader>
    void AddPagePass(FRDGBuilder& GraphBuilder, const FVoxelPageParameters& Template, uint32 NumThreads, uint32 GroupSize, const TCHAR* Name)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(NumThreads, GroupSize), 1u);
        const uint32 GroupsX = FMath::Min(NumGroups, 32768u);
        const uint32 GroupsY = FMath::DivideAndRoundUp(NumGroups, GroupsX);
        FVoxelPageParameters* Parameters = GraphBuilder.AllocParameters<FVoxelPageParameters>();
        *Parameters = Template;
        Parameters->GroupsX = GroupsX;
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelBrickPage %s", Name), Shader, Parameters, FIntVector((int32)GroupsX, (int32)GroupsY, 1));
    }
}

namespace VBP = VoxelBrickPagesDetail;

/** Render thread. The density octaves and edit tree on the GPU, shared by one terrain's pages
 *  and uploaded again only when the data changes. */
struct FVoxelBrickPageGPUInputs
{
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TRefCountPtr<FRDGPooledBuffer> Octaves;
    TRefCountPtr<FRDGPooledBuffer> EditCorners;
    TRefCountPtr<FRDGPooledBuffer> EditChildren;
    TRefCountPtr<FRDGPooledBuffer> EditFlags;

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
};

/** Meshes one page's bricks into its buffers when the compute driver runs it (render thread). */
class FVoxelBrickPageProvider final : public IRealtimeMeshComputeProvider
{
public:
    struct FState
    {
        TArray<FVoxelMesherBrickGPU> Bricks;
        TArray<FVector4f> Offsets;               // brick min from the page origin
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        FVoxelBrickMesherGPUSettings Settings;
        FVector3f UVOffset = FVector3f::ZeroVector;
        float UVScale = 0.0f;
        uint32 Color = 0xffffffffu;
        int32 VertexCapacity = 0;
        int32 TriangleCapacity = 0;
        uint64 Version = 0;
    };

    explicit FVoxelBrickPageProvider(const TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe>& InInputs)
        : Inputs(InInputs)
    {
    }

    void SetState_RenderThread(FState&& InState) { State = MoveTemp(InState); }

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    void PollStats_RenderThread()
    {
        for (int32 i = 0; i < Readbacks.Num(); i++)
        {
            FPendingReadback& Pending = Readbacks[i];
            if (!Pending.Readback->IsReady()) continue;
            if (const uint32* Data = (const uint32*)Pending.Readback->Lock(8 * sizeof(uint32)))
            {
                FVoxelBrickPageStats Read;
                Read.Version = Pending.Version;
                Read.VerticesNeeded = Data[0];
                Read.TrianglesNeeded = Data[1];
                Read.Errors = Data[2];
                Read.VerticesWritten = Data[3];
                Read.TrianglesWritten = Data[4];
                Read.BricksDropped = Data[5];
                Read.VertexCapacity = Data[6];
                Read.TriangleCapacity = Data[7];
                Read.Bricks = Pending.Bricks;
                Pending.Readback->Unlock();
                FScopeLock Lock(&StatsCS);
                if (!bNewStats || Read.Version >= Stats.Version)
                {
                    Stats = Read;
                    bNewStats = true;
                }
            }
            Readbacks.RemoveAt(i--);
        }
    }

    /** Game thread. The latest counters, once each. */
    bool TakeStats(FVoxelBrickPageStats& Out)
    {
        FScopeLock Lock(&StatsCS);
        if (!bNewStats) return false;
        Out = Stats;
        bNewStats = false;
        return true;
    }

    /** The newest state version recorded into a frame's graph. */
    std::atomic<uint64> ShownVersion{ 0 };
    /** Runs that found no shaders or a required stream missing, runs recorded, and the
     *  OR of EVoxelPageMissing bits seen (optional streams missing don't skip a run). */
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> Missing{ 0 };

private:
    struct FPendingReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Readback;
        uint64 Version = 0;
        int32 Bricks = 0;
    };

    TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe> Inputs;
    FState State;
    TArray<FPendingReadback> Readbacks;
    FCriticalSection StatsCS;
    FVoxelBrickPageStats Stats;
    bool bNewStats = false;
};

void FVoxelBrickPageProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    if (!State.Density.IsValid() || !State.Edits.IsValid()) return;

    // Every stream is written as 32-bit words and packed in the shader.
    FRDGBufferUAVRef Positions = Context.GetStreamUAV(FRealtimeMeshStreams::Position, PF_R32_FLOAT);
    FRDGBufferUAVRef Tangents = Context.GetStreamUAV(FRealtimeMeshStreams::Tangents, PF_R32_UINT);
    FRDGBufferUAVRef TexCoords = Context.GetStreamUAV(FRealtimeMeshStreams::TexCoords, PF_R32_UINT);
    FRDGBufferUAVRef Colors = Context.GetStreamUAV(FRealtimeMeshStreams::Color, PF_R32_UINT);
    FRDGBufferUAVRef Triangles = Context.GetStreamUAV(FRealtimeMeshStreams::Triangles, PF_R32_UINT);
    FRDGBufferUAVRef Args = Context.GetIndirectArgsUAV();
    uint32 MissingNow = 0;
    MissingNow |= VoxelBrickMesherGPU::AreShadersReady() ? 0u : (uint32)EVoxelPageMissing::MesherShaders;
    MissingNow |= VBP::ArePageShadersReady() ? 0u : (uint32)EVoxelPageMissing::PageShaders;
    MissingNow |= Positions ? 0u : (uint32)EVoxelPageMissing::Positions;
    MissingNow |= Tangents ? 0u : (uint32)EVoxelPageMissing::Tangents;
    MissingNow |= TexCoords ? 0u : (uint32)EVoxelPageMissing::TexCoords;
    MissingNow |= Colors ? 0u : (uint32)EVoxelPageMissing::Colors;
    MissingNow |= Triangles ? 0u : (uint32)EVoxelPageMissing::Triangles;
    MissingNow |= Args ? 0u : (uint32)EVoxelPageMissing::Args;
    if (Context.GetVertexCapacity() <= 0 || Context.GetIndexCapacity() <= 0)
        MissingNow |= (uint32)EVoxelPageMissing::Capacity;
    Missing.fetch_or(MissingNow, std::memory_order_relaxed);
    if (MissingNow & (uint32)EVoxelPageMissing::Required)
    {
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Runs.fetch_add(1, std::memory_order_relaxed);

    // Streams the driver doesn't expose are written to a scratch word and left alone.
    const uint32 StreamMask = (Tangents ? 1u : 0u) | (TexCoords ? 2u : 0u) | (Colors ? 4u : 0u);
    auto Scratch = [&GraphBuilder]()
        {
            return GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 1), TEXT("Voxel.PageScratch")), PF_R32_UINT);
        };
    Tangents = Tangents ? Tangents : Scratch();
    TexCoords = TexCoords ? TexCoords : Scratch();
    Colors = Colors ? Colors : Scratch();

    // The smaller of what the page asked for and what the driver allocated. The index
    // capacity counts indices.
    const uint32 VertexCapacity = (uint32)FMath::Max(0, FMath::Min(State.VertexCapacity, Context.GetVertexCapacity()));
    const uint32 TriangleCapacity = (uint32)FMath::Max(0, FMath::Min(State.TriangleCapacity, Context.GetIndexCapacity() / 3));

    FRDGBufferRef PageState = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 8), TEXT("Voxel.PageState"));
    FRDGBufferUAVRef PageStateUAV = GraphBuilder.CreateUAV(PageState);
    AddClearUAVPass(GraphBuilder, PageStateUAV, 0u);

    FVoxelPageParameters Template;
    Template.PageState = PageStateUAV;
    Template.OutPositions = Positions;
    Template.OutTangents = Tangents;
    Template.OutTexCoords = TexCoords;
    Template.OutColors = Colors;
    Template.OutIndices = Triangles;
    Template.OutArgs = Args;
    Template.VertexCap = VoxelBrickMesherGPU::VertexCap;
    Template.TriangleCap = VoxelBrickMesherGPU::TriangleCap;
    Template.VertexCapacity = VertexCapacity;
    Template.TriangleCapacity = TriangleCapacity;
    Template.Color = State.Color;
    Template.StreamMask = StreamMask;
    Template.UVOffset = State.UVOffset;
    Template.UVScale = State.UVScale;

    if (State.Bricks.Num() > 0)
    {
        const VoxelBrickMesherGPU::FInputs MesherInputs = Inputs->Get(GraphBuilder, State.Density, State.Edits);
        for (int32 First = 0; First < State.Bricks.Num(); First += VoxelBrickMesherGPU::MaxBricks)
        {
            const int32 Count = FMath::Min(VoxelBrickMesherGPU::MaxBricks, State.Bricks.Num() - First);
            const VoxelBrickMesherGPU::FOutputs Meshed = VoxelBrickMesherGPU::AddMesherPasses(GraphBuilder, *State.Density, MesherInputs,
                State.Settings, TConstArrayView<FVoxelMesherBrickGPU>(State.Bricks.GetData() + First, Count));
            FRDGBufferRef Offsets = CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.PageBrickOffsets"), sizeof(FVector4f), Count,
                State.Offsets.GetData() + First, sizeof(FVector4f) * Count);
            FRDGBufferRef BrickBase = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 3 * Count), TEXT("Voxel.PageBrickBase"));

            FVoxelPageParameters Batch = Template;
            Batch.MesherCounters = GraphBuilder.CreateSRV(Meshed.Counters);
            Batch.MesherVertices = GraphBuilder.CreateSRV(Meshed.Vertices);
            Batch.MesherIndices = GraphBuilder.CreateSRV(Meshed.Indices);
            Batch.BrickOffsets = GraphBuilder.CreateSRV(Offsets);
            Batch.BrickBase = GraphBuilder.CreateUAV(BrickBase);
            Batch.NumBricks = (uint32)Count;
            VBP::AddPagePass<FVoxelPageOffsetsCS>(GraphBuilder, Batch, 1u, 1u, TEXT("Offsets"));
            VBP::AddPagePass<FVoxelPageVerticesCS>(GraphBuilder, Batch, (uint32)Count * VoxelBrickMesherGPU::VertexCap, 64u, TEXT("Vertices"));
            VBP::AddPagePass<FVoxelPageTrianglesCS>(GraphBuilder, Batch, (uint32)Count * VoxelBrickMesherGPU::TriangleCap, 64u, TEXT("Triangles"));
        }
    }
    VBP::AddPagePass<FVoxelPageArgsCS>(GraphBuilder, Template, 1u, 1u, TEXT("Draw arguments"));

    if (Readbacks.Num() >= VBP::MaxPendingReadbacks)
        Readbacks.RemoveAt(0);
    FPendingReadback& Pending = Readbacks.AddDefaulted_GetRef();
    Pending.Readback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.PageStateReadback"));
    Pending.Version = State.Version;
    Pending.Bricks = State.Bricks.Num();
    AddEnqueueCopyPass(GraphBuilder, Pending.Readback.Get(), PageState, 8 * sizeof(uint32));

    ShownVersion.store(State.Version, std::memory_order_release);
}

FVoxelBrickPages::FVoxelBrickPages()
    : GPUInputs(MakeShared<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe>())
{
}

FVoxelBrickPages::~FVoxelBrickPages()
{
    Reset();
}

void FVoxelBrickPages::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelBrickTerrainSettings& InSettings)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Settings = InSettings;
    // The level-MaxLevel brick spans the root: 2^MaxDepth finest cells.
    MesherSettings.FinestCell = Settings.FinestCell;
    MesherSettings.CellsPerBrick = Settings.CellsPerBrick;
    MesherSettings.MaxDepth = Settings.MaxLevel + FMath::FloorLog2(FMath::Max(1, Settings.CellsPerBrick));
    Density.Reset();
    Edits.Reset();
}

void FVoxelBrickPages::Reset()
{
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        DestroyInstance(Pair.Value.Current);
        DestroyInstance(Pair.Value.Next);
    }
    Pages.Reset();
}

FVoxelBrickKey FVoxelBrickPages::PageOf(const FVoxelBrickKey& Key)
{
    FVoxelBrickKey Page;
    Page.Level = Key.Level;
    Page.Coord = FInt64Vector(VBP::FloorShift(Key.Coord.X, VBP::PageShift), VBP::FloorShift(Key.Coord.Y, VBP::PageShift), VBP::FloorShift(Key.Coord.Z, VBP::PageShift));
    return Page;
}

uint64 FVoxelBrickPages::SetBrick(const FVoxelBrickKey& Key, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours)
{
    FPage& Page = Pages.FindOrAdd(PageOf(Key));
    FMember& Member = Page.Members.FindOrAdd(Key);
    FMemory::Memcpy(Member.Levels, NeighbourLevels, sizeof(Member.Levels));
    Member.Meshed = MeshedNeighbours;
    Page.bDirty = true;
    return Page.NextVersion;
}

void FVoxelBrickPages::RemoveBrick(const FVoxelBrickKey& Key)
{
    if (FPage* Page = Pages.Find(PageOf(Key)))
    {
        if (Page->Members.Remove(Key) > 0)
            Page->bDirty = true;
    }
}

bool FVoxelBrickPages::IsShown(const FVoxelBrickKey& Key, uint64 Version) const
{
    const FPage* Page = Pages.Find(PageOf(Key));
    return Page && Page->Current.IsValid() && Page->Current.Handle != 0
        && Page->Current.Provider->ShownVersion.load(std::memory_order_acquire) >= Version;
}

FIntPoint FVoxelBrickPages::WantedCapacity(const FPage& Page) const
{
    // Per brick: the page's last measured average, else about a busy surface brick.
    double PerVertex = 1024.0, PerTriangle = 2048.0;
    if (Page.Stats.Bricks > 0 && Page.Stats.TrianglesNeeded > 0)
    {
        PerVertex = FMath::Max(64.0, (double)Page.Stats.VerticesNeeded / Page.Stats.Bricks);
        PerTriangle = FMath::Max(128.0, (double)Page.Stats.TrianglesNeeded / Page.Stats.Bricks);
    }
    const double Members = (double)Page.Members.Num();
    const int32 Vertices = (int32)FMath::RoundUpToPowerOfTwo((uint32)FMath::Clamp(Members * PerVertex * 1.5, (double)VBP::MinVertices, (double)VBP::MaxVertices));
    const int32 TrianglesWanted = (int32)FMath::RoundUpToPowerOfTwo((uint32)FMath::Clamp(Members * PerTriangle * 1.5, (double)VBP::MinTriangles, (double)VBP::MaxTriangles));
    return FIntPoint(Vertices, TrianglesWanted);
}

FVoxelBrickPages::FInstance FVoxelBrickPages::CreateInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity)
{
    FInstance Instance;
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return Instance;

    FVoxelBrickGrid Grid;
    Grid.Origin = Settings.Origin;
    Grid.CellSize = Settings.FinestCell * (double)(1LL << PageKey.Level);
    Grid.CellsPerBrick = Settings.CellsPerBrick;
    const FVector PageOrigin = Grid.BrickMin(PageKey.Coord * (1LL << VBP::PageShift));

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Surface Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Settings.Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetRelativeLocation(PageOrigin);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetRenderCustomDepth(true);
    Component->SetVisibleInRayTracing(false);

    // The uploaded streams only size the buffers; the provider writes them. Two corner
    // positions give the page its bounds (bricks reach 4 cells past their box).
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(VertexCapacity);
    const double Margin = 4.0 * Grid.CellSize;
    const double Extent = (double)(1LL << VBP::PageShift) * Grid.BrickSize();
    TArrayView<FVector3f> PositionView = Positions.GetArrayView<FVector3f>();
    PositionView[0] = FVector3f((float)-Margin);
    PositionView[1] = FVector3f((float)(Extent + Margin));
    Streams.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()).SetNumZeroed(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>()).SetNumZeroed(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()).SetNumZeroed(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()).SetNumZeroed(TriangleCapacity);
    // No PolyGroups stream: the buffer set then makes one section over every vertex and index.
    // (Sections built from polygroups take the vertex range the indices use, and drop a
    // section whose indices are all zero, which leaves the section key without a section.)

    Instance.bBuffersReady = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
    const TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> Ready = Instance.bBuffersReady;
    Mesh->CreateBufferSet(VBP::GroupKey(), MoveTemp(Streams), FRealtimeMeshBufferSetConfig(ERealtimeMeshSectionDrawType::Static, true))
        .Next([Ready](ERealtimeMeshProxyUpdateStatus) { Ready->store(true, std::memory_order_release); });
    Mesh->UpdateSectionConfig(VBP::SectionKey(), FRealtimeMeshSectionConfig(0), false);
    // The section stays hidden until the provider is registered: until then it draws its whole
    // (zero) range. Once registered it draws indirect from args the driver creates zeroed, so
    // nothing until the first run. The component must stay visible: a hidden component has no
    // render proxy, and the driver runs providers only for meshes with one.
    Mesh->SetSectionVisibility(VBP::SectionKey(), false);

    Instance.Mesh = Mesh;
    Instance.Component = Component;
    Instance.Provider = MakeShared<FVoxelBrickPageProvider, ESPMode::ThreadSafe>(GPUInputs);
    Instance.VertexCapacity = VertexCapacity;
    Instance.TriangleCapacity = TriangleCapacity;
    Instance.CreatedFrame = GFrameCounter;
    return Instance;
}

void FVoxelBrickPages::DestroyInstance(FInstance& Instance)
{
    if (Instance.Handle != 0)
        UnregisterRealtimeMeshComputeProvider(Instance.Handle);
    if (URealtimeMeshComponent* Component = Instance.Component.Get())
        Component->DestroyComponent();
    Instance = FInstance();
}

void FVoxelBrickPages::Push(const FVoxelBrickKey& PageKey, const FPage& Page, FInstance& Instance) const
{
    if (!Instance.IsValid()) return;

    FVoxelBrickGrid Grid;
    Grid.Origin = Settings.Origin;
    Grid.CellSize = Settings.FinestCell * (double)(1LL << PageKey.Level);
    Grid.CellsPerBrick = Settings.CellsPerBrick;
    const FInt64Vector FirstBrick = PageKey.Coord * (1LL << VBP::PageShift);
    const FVector PageOrigin = Grid.BrickMin(FirstBrick);

    FVoxelBrickPageProvider::FState State;
    State.Density = Density;
    State.Edits = Edits;
    State.Settings = MesherSettings;
    State.VertexCapacity = Instance.VertexCapacity;
    State.TriangleCapacity = Instance.TriangleCapacity;
    State.Version = Page.Published;
    State.UVScale = (float)Settings.TriplanarUVScale;
    const FVector UVOrigin = PageOrigin * Settings.TriplanarUVScale;
    State.UVOffset = FVector3f((float)FMath::Frac(UVOrigin.X), (float)FMath::Frac(UVOrigin.Y), (float)FMath::Frac(UVOrigin.Z));
    State.Color = (CVarVoxelDebugChunkColor.GetValueOnGameThread() > 0 ? FVoxelBrickTerrain::LevelColor(PageKey.Level) : FColor::White).DWColor();
    State.Bricks.Reserve(Page.Members.Num());
    State.Offsets.Reserve(Page.Members.Num());
    for (const TPair<FVoxelBrickKey, FMember>& Pair : Page.Members)
    {
        FVoxelMesherBrickGPU Brick;
        if (!VoxelBrickMesherGPU::MakeBrick(Pair.Key, Pair.Value.Levels, Pair.Value.Meshed, MesherSettings, Brick))
            continue;
        State.Bricks.Add(Brick);
        const FInt64Vector Local = Pair.Key.Coord - FirstBrick;
        State.Offsets.Add(FVector4f((float)(Local.X * Grid.BrickSize()), (float)(Local.Y * Grid.BrickSize()), (float)(Local.Z * Grid.BrickSize()), 0.0f));
    }

    ENQUEUE_RENDER_COMMAND(VoxelBrickPagePush)([Provider = Instance.Provider, State = MoveTemp(State)](FRHICommandListImmediate&) mutable
        {
            Provider->SetState_RenderThread(MoveTemp(State));
        });
    Instance.PushedVersion = Page.Published;
}

void FVoxelBrickPages::Advance(FInstance& Instance) const
{
    if (!Instance.IsValid()) return;
    URealtimeMeshSimple* Mesh = Instance.Mesh.Get();
    if (!Mesh) return;

    if (Instance.Handle == 0)
    {
        // Registered once the buffers exist and the component has a proxy (two frames, as
        // FMeshChunk::ApplyStream); the driver runs an OnDemand provider when registered.
        if (!Instance.bBuffersReady->load(std::memory_order_acquire) || GFrameCounter < Instance.CreatedFrame + 2 || Instance.PushedVersion == 0)
            return;
        // Shown first: the driver binds the indirect args to the section proxy, and a later
        // section update from the data side would rebuild that proxy without them.
        Mesh->SetSectionVisibility(VBP::SectionKey(), true);
        Instance.bVisible = true;
        FRealtimeMeshComputeRegistration Registration;
        Registration.Mesh = Mesh->GetMesh();
        Registration.BufferSetKey = VBP::GroupKey();
        Registration.SectionKey = VBP::SectionKey();
        Registration.Provider = Instance.Provider;
        Registration.Trigger = ERealtimeMeshComputeTrigger::OnDemand;
        Registration.bUseIndirect = true;
        Instance.Handle = RegisterRealtimeMeshComputeProvider(Registration);
        Instance.LastKickFrame = GFrameCounter;
        Instance.RegisteredFrame = GFrameCounter;
        return;
    }


    // A run that didn't happen (no proxy yet, shaders still compiling) is asked for again.
    if (Instance.Provider->ShownVersion.load(std::memory_order_acquire) < Instance.PushedVersion && GFrameCounter > Instance.LastKickFrame + VBP::KickFrames)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelBrickPages::Flush(const TSharedPtr<const FVoxelDensityGPUData>& InDensity, const TSharedPtr<const FVoxelEditGPUData>& InEdits)
{
    Density = InDensity;
    Edits = InEdits;
    if (!Density.IsValid() || !Edits.IsValid()) return;

    TArray<FVoxelBrickKey> Emptied;
    TArray<TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>> Poll;
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        const FVoxelBrickKey& PageKey = Pair.Key;
        FPage& Page = Pair.Value;

        if (Page.Current.IsValid())
        {
            FVoxelBrickPageStats Read;
            if (Page.Current.Provider->TakeStats(Read))
            {
                Page.Stats = Read;
                Page.bStatsFromCurrent = true;
                if (Read.Version >= Page.Published)
                    Page.bAwaitingStats = false;
            }
        }

        if (Page.bDirty && Page.Members.Num() == 0)
        {
            DestroyInstance(Page.Current);
            DestroyInstance(Page.Next);
            Emptied.Add(PageKey);
            continue;
        }

        // Buffers too small (or four times too large) for the members get a replacement,
        // which takes over once it has drawn the newest version.
        const FIntPoint Wanted = WantedCapacity(Page);
        bool bNewNext = false;
        if (!Page.Current.IsValid())
        {
            Page.Current = CreateInstance(PageKey, Wanted.X, Wanted.Y);
        }
        else
        {
            const FInstance& Sized = Page.Next.IsValid() ? Page.Next : Page.Current;
            const bool bTooSmall = Wanted.X > Sized.VertexCapacity || Wanted.Y > Sized.TriangleCapacity;
            const bool bTooLarge = Sized.VertexCapacity >= 4 * Wanted.X && Sized.TriangleCapacity >= 4 * Wanted.Y;
            if (bTooSmall || bTooLarge)
            {
                DestroyInstance(Page.Next);
                Page.Next = CreateInstance(PageKey, Wanted.X, Wanted.Y);
                bNewNext = true;
            }
        }

        if (Page.bDirty)
        {
            Page.Published = Page.NextVersion++;
            Page.bDirty = false;
            Page.bAwaitingStats = true;
            for (FInstance* Instance : { &Page.Current, &Page.Next })
            {
                Push(PageKey, Page, *Instance);
                if (Instance->Handle != 0)
                {
                    MarkRealtimeMeshComputeProviderDirty(Instance->Handle);
                    Instance->LastKickFrame = GFrameCounter;
                }
            }
        }
        else if (bNewNext)
        {
            Push(PageKey, Page, Page.Next);
        }

        Advance(Page.Current);
        Advance(Page.Next);
        if (Page.Next.IsValid() && Page.Next.Handle != 0 && Page.Next.Provider->ShownVersion.load(std::memory_order_acquire) >= Page.Published)
        {
            DestroyInstance(Page.Current);
            Page.Current = MoveTemp(Page.Next);
            Page.bStatsFromCurrent = false;
            Page.Next = FInstance();
            Page.bAwaitingStats = true;
        }

        if (Page.Current.IsValid())
        {
            const uint32 MissingBits = Page.Current.Provider->Missing.load(std::memory_order_relaxed);
            if (MissingBits != 0 && (MissingBits & ~WarnedMissing) != 0)
            {
                UE_LOG(LogTemp, Warning, TEXT("[VoxelBricks] A brick page's compute run is missing: %s%s."), *DescribeMissing(MissingBits),
                    (MissingBits & (uint32)EVoxelPageMissing::Required) ? TEXT(" (run skipped, retried every 30 frames)") : TEXT(" (written elsewhere, run continues)"));
                WarnedMissing |= MissingBits;
            }
            if (Page.bAwaitingStats)
                Poll.Add(Page.Current.Provider);
        }
    }
    for (const FVoxelBrickKey& Key : Emptied)
        Pages.Remove(Key);

    if (Poll.Num() > 0)
    {
        ENQUEUE_RENDER_COMMAND(VoxelBrickPagePoll)([Poll = MoveTemp(Poll)](FRHICommandListImmediate&)
            {
                for (const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>& Provider : Poll)
                    Provider->PollStats_RenderThread();
            });
    }
}

FString FVoxelBrickPages::DescribeMissing(uint32 Bits)
{
    static const TCHAR* Names[] = { TEXT("mesher shaders"), TEXT("page shaders"), TEXT("Position stream"), TEXT("Tangents stream"),
        TEXT("TexCoords stream"), TEXT("Color stream"), TEXT("Triangles stream"), TEXT("indirect args"), TEXT("capacity") };
    FString Out;
    for (int32 i = 0; i < UE_ARRAY_COUNT(Names); i++)
    {
        if (Bits & (1u << i))
            Out += FString::Printf(TEXT("%s%s"), Out.IsEmpty() ? TEXT("") : TEXT(", "), Names[i]);
    }
    return Out;
}

FString FVoxelBrickPages::Describe() const
{
    int32 Instances = 0, Members = 0, Waiting = 0, Hidden = 0, Dropped = 0, Clamped = 0, Lagging = 0;
    uint64 MaxLag = 0;
    uint32 Runs = 0, Skipped = 0, MissingBits = 0;
    int64 Bytes = 0, Triangles = 0;
    uint32 Errors = 0;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        const FPage& Page = Pair.Value;
        Members += Page.Members.Num();
        for (const FInstance* Instance : { &Page.Current, &Page.Next })
        {
            if (!Instance->IsValid()) continue;
            Instances++;
            Bytes += (int64)Instance->VertexCapacity * VBP::BytesPerVertex + (int64)Instance->TriangleCapacity * VBP::BytesPerTriangle;
            Waiting += Instance->Handle == 0 ? 1 : 0;
            Hidden += Instance->Handle != 0 && Instance->Provider->ShownVersion.load(std::memory_order_relaxed) == 0 ? 1 : 0;
            Runs += Instance->Provider->Runs.load(std::memory_order_relaxed);
            Skipped += Instance->Provider->SkippedRuns.load(std::memory_order_relaxed);
            MissingBits |= Instance->Provider->Missing.load(std::memory_order_relaxed);
        }
        Triangles += Page.Stats.TrianglesWritten;
        if (Page.Current.IsValid())
        {
            const uint64 Shown = Page.Current.Provider->ShownVersion.load(std::memory_order_relaxed);
            if (Shown < Page.Published)
            {
                Lagging++;
                MaxLag = FMath::Max(MaxLag, Page.Published - Shown);
            }
        }
        // The provider uses the smaller of the page's and the driver's capacities.
        if (Page.bStatsFromCurrent && Page.Current.IsValid()
            && ((int32)Page.Stats.VertexCapacity < Page.Current.VertexCapacity || (int32)Page.Stats.TriangleCapacity < Page.Current.TriangleCapacity))
            Clamped++;
        Dropped += (int32)Page.Stats.BricksDropped;
        Errors |= Page.Stats.Errors;
    }
    return FString::Printf(TEXT("GPU pages: %d (%d bricks, %d buffer sets, %d not yet registered, %d registered but not yet run, %d behind their newest version by up to %llu), %u runs, %u skipped%s%s, %.1f MB of buffers, %lld triangles drawn, %d bricks dropped (buffers full), mesher errors 0x%x, %d pages with less capacity from the driver than allocated"),
        Pages.Num(), Members, Instances, Waiting, Hidden, Lagging, MaxLag, Runs, Skipped, MissingBits ? TEXT(", missing: ") : TEXT(""), *DescribeMissing(MissingBits),
        Bytes / (1024.0 * 1024.0), Triangles, Dropped, Errors, Clamped);
}
