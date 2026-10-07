#include "VoxelBrickMesherGPU.h"
#include "VoxelDensityGPU.h"
#include "FSparseEditStore.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "Containers/Ticker.h"
#include "Async/Async.h"
#include "Misc/App.h"
#include <atomic>

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelMesherParameters, )
    // Density (VoxelDensitySample.ush)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOctave>, Octaves)
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
    // Edits (VoxelEditSample.ush)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, EditCorners)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, EditChildren)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditFlags)
    SHADER_PARAMETER(uint32, EditNodeCount)
    SHADER_PARAMETER(int32, EditRootExtent)
    // Mesher (VoxelBrickMesherPasses.ush)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelMesherBrick>, Bricks)
    SHADER_PARAMETER(int32, CellsPerBrick)
    SHADER_PARAMETER(uint32, NumBricks)
    SHADER_PARAMETER(uint32, HashSize)
    SHADER_PARAMETER(uint32, FaceCap)
    SHADER_PARAMETER(uint32, VertexCap)
    SHADER_PARAMETER(uint32, TriangleCap)
    SHADER_PARAMETER(float, FinestCell)
    SHADER_PARAMETER(float, MinFootprint)
    SHADER_PARAMETER(uint32, PassMode)
    SHADER_PARAMETER(uint32, GroupsX)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, Dense)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, HashKeys)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, HashValues)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SlotState)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SlotVertex)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Faces)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Counters)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, Vertices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Indices)
END_SHADER_PARAMETER_STRUCT()

// One global shader per pass of VoxelBrickMesherPasses.ush, all on FVoxelMesherParameters.
#define VOXEL_MESHER_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelMesherParameters; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
        static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment) \
        { \
            FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment); \
            OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath); \
        } \
    };

VOXEL_MESHER_SHADER(FVoxelMesherSampleCS)
VOXEL_MESHER_SHADER(FVoxelMesherEdgesCS)
VOXEL_MESHER_SHADER(FVoxelMesherRequestCS)
VOXEL_MESHER_SHADER(FVoxelMesherSolveCS)
VOXEL_MESHER_SHADER(FVoxelMesherTrianglesCS)
#undef VOXEL_MESHER_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelMesherSampleCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "SampleCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherEdgesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "EdgesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherRequestCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "RequestCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherSolveCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "SolveCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherTrianglesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "TrianglesCS", SF_Compute);

namespace VoxelBrickMesherGPUDetail
{
    constexpr uint32 HashSize = 16384;     // per brick; a 32-cell brick requests a few thousand points at most
    constexpr uint32 FaceCap = 8192;
    constexpr uint32 VertexCap = 8192;
    constexpr uint32 TriangleCap = 16384;

    FVector2f MesherDoubleFloat(double V)
    {
        const float Hi = (float)V;
        return FVector2f(Hi, (float)(V - (double)Hi));
    }

    struct FMesherReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Counters;
        TUniquePtr<FRHIGPUBufferReadback> Vertices;
        TUniquePtr<FRHIGPUBufferReadback> Indices;
        int32 NumBricks = 0;
        int32 NumRequests = 0;
        TArray<int32> RequestOfBrick;
        std::atomic<bool> bDone{ false };
        TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone;
    };

    template<typename TShader>
    void AddMesherPass(FRDGBuilder& GraphBuilder, const FVoxelMesherParameters& Template, uint32 Mode, uint32 NumThreads, const TCHAR* Name)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(NumThreads, 64u), 1u);
        const uint32 GroupsX = FMath::Min(NumGroups, 32768u);
        const uint32 GroupsY = FMath::DivideAndRoundUp(NumGroups, GroupsX);
        FVoxelMesherParameters* Parameters = GraphBuilder.AllocParameters<FVoxelMesherParameters>();
        *Parameters = Template;
        Parameters->PassMode = Mode;
        Parameters->GroupsX = GroupsX;
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelMesher %s", Name), Shader, Parameters, FIntVector((int32)GroupsX, (int32)GroupsY, 1));
    }

    template<typename T>
    FRDGBufferRef UploadBuffer(FRDGBuilder& GraphBuilder, const TCHAR* Name, const TArray<T>& Data)
    {
        // A structured buffer can't be empty.
        TArray<T> Copy = Data;
        if (Copy.Num() == 0)
            Copy.AddZeroed();
        return CreateStructuredBuffer(GraphBuilder, Name, sizeof(T), Copy.Num(), Copy.GetData(), sizeof(T) * Copy.Num());
    }
}

void VoxelBrickMesherGPU::MeshBricksAsync(TSharedRef<const FVoxelDensityGPUData> Density, TSharedRef<const FVoxelEditGPUData> Edits,
    const FVoxelBrickMesherGPUSettings& Settings, TArray<FVoxelBrickMesherGPURequest> Requests,
    TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone)
{
    using namespace VoxelBrickMesherGPUDetail;
    check(!IsInRenderingThread());

    const TSharedRef<FMesherReadback, ESPMode::ThreadSafe> State = MakeShared<FMesherReadback, ESPMode::ThreadSafe>();
    State->OnDone = MoveTemp(OnDone);
    // The caps below are sized for 32-cell bricks; the hash key holds points of up to 250.
    const int32 N = FMath::Clamp(Settings.CellsPerBrick / 4 * 4, 8, 64);

    // Brick records, one per request (the first MaxBricks). A brick whose lattice indices,
    // including the points sampled around it, don't fit 32 bits gets an empty mesh and
    // ErrorNotMeshed.
    TArray<FVoxelMesherBrickGPU> Bricks;
    TArray<int32> RequestOfBrick;
    State->NumRequests = Requests.Num();
    for (int32 r = 0; r < Requests.Num(); r++)
    {
        const FVoxelBrickMesherGPURequest& Request = Requests[r];
        if (Bricks.Num() >= MaxBricks) break;
        FVoxelMesherBrickGPU Brick;
        const int64 Unit = 1LL << Request.Key.Level;
        const int64 Scale = 2LL * N * Unit;
        const int64 Reach = (2LL * N + 16) * Unit;
        const int64 Origin = 1LL << Settings.MaxDepth;
        const int64 Min[3] = { Request.Key.Coord.X * Scale - Origin, Request.Key.Coord.Y * Scale - Origin, Request.Key.Coord.Z * Scale - Origin };
        if (FMath::Abs(Min[0]) + Reach >= MAX_int32 || FMath::Abs(Min[1]) + Reach >= MAX_int32 || FMath::Abs(Min[2]) + Reach >= MAX_int32)
            continue;
        Brick.MinIndex = FIntVector((int32)Min[0], (int32)Min[1], (int32)Min[2]);
        Brick.Level = Request.Key.Level;
        for (int32 d = 0; d < 27; d++)
        {
            const uint32 Bits = d == 13 ? 1u : (uint32)(FMath::Clamp((int32)Request.NeighbourLevels[d], -1, 1) + 1);
            if (d < 16)
                Brick.LevelsLo |= Bits << (2 * d);
            else
                Brick.LevelsHi |= Bits << (2 * (d - 16));
        }
        Brick.Meshed = Request.MeshedNeighbours;
        Bricks.Add(Brick);
        RequestOfBrick.Add(r);
    }
    State->NumBricks = Bricks.Num();
    State->RequestOfBrick = RequestOfBrick;
    if (Bricks.Num() == 0 || !FApp::CanEverRender())
    {
        AsyncTask(ENamedThreads::GameThread, [State]() { State->OnDone(TArray<FVoxelBrickMesh>(), TArray<FVoxelBrickMesherGPUStats>()); });
        return;
    }

    ENQUEUE_RENDER_COMMAND(VoxelMesherGPU)([Density, Edits, Settings, N, Bricks = MoveTemp(Bricks), State](FRHICommandListImmediate& RHICmdList)
        {
            // GetShader by type returns an empty reference when the shader isn't compiled for
            // this platform (TShaderMapRef would assert).
            if (!GetGlobalShaderMap(GMaxRHIFeatureLevel)->GetShader(&FVoxelMesherTrianglesCS::GetStaticType(), 0).IsValid())
            {
                State->bDone.store(true, std::memory_order_release);
                AsyncTask(ENamedThreads::GameThread, [State]() { State->OnDone(TArray<FVoxelBrickMesh>(), TArray<FVoxelBrickMesherGPUStats>()); });
                return;
            }

            const uint32 NB = (uint32)Bricks.Num();
            const uint32 UN = (uint32)N;
            const uint32 DenseSide = UN + 4u;
            const uint32 DenseTotal = DenseSide * DenseSide * DenseSide;
            const uint32 OwnSide = UN + 2u;
            const uint32 CoarseSide = UN / 2u + 2u;
            const uint32 NumSlots = OwnSide * OwnSide * OwnSide + CoarseSide * CoarseSide * CoarseSide;

            FRDGBuilder GraphBuilder(RHICmdList);
            auto MakeBuffer = [&GraphBuilder](uint32 Stride, uint32 Count, const TCHAR* Name)
                {
                    return GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(Stride, FMath::Max(Count, 1u)), Name);
                };
            FRDGBufferRef DenseBuffer = MakeBuffer(4, NB * DenseTotal, TEXT("Voxel.MesherDense"));
            FRDGBufferRef HashKeysBuffer = MakeBuffer(4, NB * HashSize, TEXT("Voxel.MesherHashKeys"));
            FRDGBufferRef HashValuesBuffer = MakeBuffer(4, NB * HashSize, TEXT("Voxel.MesherHashValues"));
            FRDGBufferRef SlotStateBuffer = MakeBuffer(4, NB * NumSlots, TEXT("Voxel.MesherSlotState"));
            FRDGBufferRef SlotVertexBuffer = MakeBuffer(4, NB * NumSlots, TEXT("Voxel.MesherSlotVertex"));
            FRDGBufferRef FacesBuffer = MakeBuffer(4, NB * FaceCap * 4u, TEXT("Voxel.MesherFaces"));
            FRDGBufferRef CountersBuffer = MakeBuffer(4, NB * 4u, TEXT("Voxel.MesherCounters"));
            FRDGBufferRef VerticesBuffer = MakeBuffer(4, NB * VertexCap * 6u, TEXT("Voxel.MesherVertices"));
            FRDGBufferRef IndicesBuffer = MakeBuffer(4, NB * TriangleCap * 3u, TEXT("Voxel.MesherIndices"));

            FVoxelMesherParameters Template;
            Template.Octaves = GraphBuilder.CreateSRV(UploadBuffer(GraphBuilder, TEXT("Voxel.MesherOctaves"), Density->Octaves));
            Template.HalfFinest = MesherDoubleFloat(0.5 * Settings.FinestCell);
            Template.ContinentOctaves = Density->ContinentOctaves;
            Template.MountainOctaves = Density->MountainOctaves;
            Template.DetailOctaves = Density->DetailOctaves;
            Template.InvNormContinent = Density->InvNormContinent;
            Template.InvNormMountain = Density->InvNormMountain;
            Template.InvNormDetail = Density->InvNormDetail;
            Template.PlanetRadius = Density->PlanetRadius;
            Template.NoiseAmplitude = Density->NoiseAmplitude;
            Template.ContinentWeight = Density->ContinentWeight;
            Template.MountainWeight = Density->MountainWeight;
            Template.MountainStart = Density->MountainStart;
            Template.MountainBlend = Density->MountainBlend;
            Template.DetailWeight = Density->DetailWeight;
            Template.RidgeGain = Density->RidgeGain;
            Template.SlopeDamping = Density->SlopeDamping;
            Template.EditCorners = GraphBuilder.CreateSRV(UploadBuffer(GraphBuilder, TEXT("Voxel.EditCorners"), Edits->Corners));
            Template.EditChildren = GraphBuilder.CreateSRV(UploadBuffer(GraphBuilder, TEXT("Voxel.EditChildren"), Edits->Children));
            Template.EditFlags = GraphBuilder.CreateSRV(UploadBuffer(GraphBuilder, TEXT("Voxel.EditFlags"), Edits->Flags));
            Template.EditNodeCount = (uint32)Edits->Flags.Num();
            Template.EditRootExtent = Edits->RootExtent;
            Template.Bricks = GraphBuilder.CreateSRV(UploadBuffer(GraphBuilder, TEXT("Voxel.MesherBricks"), Bricks));
            Template.CellsPerBrick = N;
            Template.NumBricks = NB;
            Template.HashSize = HashSize;
            Template.FaceCap = FaceCap;
            Template.VertexCap = VertexCap;
            Template.TriangleCap = TriangleCap;
            Template.FinestCell = (float)Settings.FinestCell;
            Template.MinFootprint = (float)Density->MinFootprint;
            Template.Dense = GraphBuilder.CreateUAV(DenseBuffer);
            Template.HashKeys = GraphBuilder.CreateUAV(HashKeysBuffer);
            Template.HashValues = GraphBuilder.CreateUAV(HashValuesBuffer);
            Template.SlotState = GraphBuilder.CreateUAV(SlotStateBuffer);
            Template.SlotVertex = GraphBuilder.CreateUAV(SlotVertexBuffer);
            Template.Faces = GraphBuilder.CreateUAV(FacesBuffer);
            Template.Counters = GraphBuilder.CreateUAV(CountersBuffer);
            Template.Vertices = GraphBuilder.CreateUAV(VerticesBuffer);
            Template.Indices = GraphBuilder.CreateUAV(IndicesBuffer);

            AddClearUAVPass(GraphBuilder, Template.HashKeys, 0u);
            AddClearUAVPass(GraphBuilder, Template.HashValues, 0xffffffffu);
            AddClearUAVPass(GraphBuilder, Template.SlotState, 0u);
            AddClearUAVPass(GraphBuilder, Template.SlotVertex, 0xffffffffu);
            AddClearUAVPass(GraphBuilder, Template.Counters, 0u);

            AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, Template, 0u, NB * DenseTotal, TEXT("Sample lattice"));
            AddMesherPass<FVoxelMesherEdgesCS>(GraphBuilder, Template, 0u, NB * 3u * UN * (UN + 1u) * (UN + 1u), TEXT("Edges"));
            AddMesherPass<FVoxelMesherRequestCS>(GraphBuilder, Template, 0u, NB * NumSlots, TEXT("Request edge ends"));
            AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, Template, 1u, NB * HashSize, TEXT("Sample edge ends"));
            AddMesherPass<FVoxelMesherRequestCS>(GraphBuilder, Template, 1u, NB * NumSlots, TEXT("Request gradients"));
            AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, Template, 1u, NB * HashSize, TEXT("Sample gradients"));
            AddMesherPass<FVoxelMesherSolveCS>(GraphBuilder, Template, 0u, NB * NumSlots, TEXT("Solve"));
            AddMesherPass<FVoxelMesherTrianglesCS>(GraphBuilder, Template, 0u, NB * FaceCap, TEXT("Triangles"));

            State->Counters = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherCountersReadback"));
            State->Vertices = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherVerticesReadback"));
            State->Indices = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherIndicesReadback"));
            AddEnqueueCopyPass(GraphBuilder, State->Counters.Get(), CountersBuffer, NB * 4u * 4u);
            AddEnqueueCopyPass(GraphBuilder, State->Vertices.Get(), VerticesBuffer, NB * VertexCap * 6u * 4u);
            AddEnqueueCopyPass(GraphBuilder, State->Indices.Get(), IndicesBuffer, NB * TriangleCap * 3u * 4u);
            GraphBuilder.Execute();
        });

    // Poll the readbacks once per frame on the render thread; build the meshes there and hand
    // them to the game thread.
    FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([State](float) -> bool
        {
            if (State->bDone.load(std::memory_order_acquire)) return false;
            ENQUEUE_RENDER_COMMAND(VoxelMesherGPUPoll)([State](FRHICommandListImmediate&)
                {
                    if (State->bDone.load(std::memory_order_acquire) || !State->Counters.IsValid()
                        || !State->Counters->IsReady() || !State->Vertices->IsReady() || !State->Indices->IsReady())
                        return;

                    const uint32 NB = (uint32)State->NumBricks;
                    TArray<FVoxelBrickMesh> Meshes;
                    TArray<FVoxelBrickMesherGPUStats> Stats;
                    Meshes.SetNum(State->NumRequests);
                    Stats.SetNum(State->NumRequests);
                    for (FVoxelBrickMesherGPUStats& S : Stats)
                        S.Errors = VoxelBrickMesherGPU::ErrorNotMeshed;
                    const uint32* Counters = (const uint32*)State->Counters->Lock(NB * 4u * 4u);
                    const float* Vertices = (const float*)State->Vertices->Lock(NB * VertexCap * 6u * 4u);
                    const uint32* Indices = (const uint32*)State->Indices->Lock(NB * TriangleCap * 3u * 4u);
                    if (Counters && Vertices && Indices)
                    {
                        for (uint32 b = 0; b < NB; b++)
                        {
                            const int32 r = State->RequestOfBrick[b];
                            FVoxelBrickMesherGPUStats& S = Stats[r];
                            S.Faces = Counters[b * 4u];
                            S.Vertices = Counters[b * 4u + 1u];
                            S.Triangles = Counters[b * 4u + 2u];
                            S.Errors = Counters[b * 4u + 3u];

                            FVoxelBrickMesh& Mesh = Meshes[r];
                            const uint32 NumVertices = FMath::Min(S.Vertices, VertexCap);
                            const uint32 NumTriangles = FMath::Min(S.Triangles, TriangleCap);
                            Mesh.Positions.SetNumUninitialized(NumVertices);
                            Mesh.Normals.SetNumUninitialized(NumVertices);
                            for (uint32 v = 0; v < NumVertices; v++)
                            {
                                const float* V = Vertices + (b * VertexCap + v) * 6u;
                                Mesh.Positions[v] = FVector3f(V[0], V[1], V[2]);
                                Mesh.Normals[v] = FVector3f(V[3], V[4], V[5]);
                            }
                            Mesh.Indices.SetNumUninitialized(NumTriangles * 3u);
                            for (uint32 i = 0; i < NumTriangles * 3u; i++)
                                Mesh.Indices[i] = (int32)Indices[b * TriangleCap * 3u + i];
                        }
                    }
                    if (Counters) State->Counters->Unlock();
                    if (Vertices) State->Vertices->Unlock();
                    if (Indices) State->Indices->Unlock();
                    State->Counters.Reset();
                    State->Vertices.Reset();
                    State->Indices.Reset();
                    State->bDone.store(true, std::memory_order_release);
                    AsyncTask(ENamedThreads::GameThread, [State, Meshes = MoveTemp(Meshes), Stats = MoveTemp(Stats)]() mutable
                        {
                            State->OnDone(MoveTemp(Meshes), MoveTemp(Stats));
                        });
                });
            return true;
        }));
}
