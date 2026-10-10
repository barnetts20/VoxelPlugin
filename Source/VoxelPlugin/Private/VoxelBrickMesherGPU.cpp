#include "VoxelBrickMesherGPU.h"
#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandGPU.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RHIGPUReadback.h"
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
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandShaderParameters, Land)
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
    SHADER_PARAMETER(uint32, TargetHashSize)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TargetState)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, TargetData)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TargetHashKeys)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TargetHashValues)
    // Land debug views (VoxelLandDebug.ush)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandDebugParameters, LandDebug)
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
VOXEL_MESHER_SHADER(FVoxelMesherTargetRequestCS)
VOXEL_MESHER_SHADER(FVoxelMesherTargetSolveCS)
VOXEL_MESHER_SHADER(FVoxelMesherVertexTargetsCS)
VOXEL_MESHER_SHADER(FVoxelMesherLandDebugCS)
#undef VOXEL_MESHER_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelMesherSampleCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "SampleCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherEdgesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "EdgesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherRequestCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "RequestCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherSolveCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "SolveCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherTrianglesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "TrianglesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherTargetRequestCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "TargetRequestCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherTargetSolveCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "TargetSolveCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherVertexTargetsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "VertexTargetsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelMesherLandDebugCS, "/Plugin/VoxelPlugin/Private/VoxelBrickMesherGPU.usf", "LandDebugCS", SF_Compute);

namespace VoxelBrickMesherGPUDetail
{
    struct FMesherReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Counters;
        TUniquePtr<FRHIGPUBufferReadback> Vertices;
        TUniquePtr<FRHIGPUBufferReadback> Indices;
        int32 NumBricks = 0;
        int32 NumRequests = 0;
        VoxelBrickMesherGPU::FCaps Caps;
        TArray<int32> RequestOfBrick;
        std::atomic<bool> bDone{ false };
        TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone;
    };

    template<typename TShader>
    void AddMesherPass(FRDGBuilder& GraphBuilder, ERDGPassFlags PassFlags, const FVoxelMesherParameters& Template, uint32 Mode, uint32 NumThreads, const TCHAR* Name)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(NumThreads, 64u), 1u);
        const uint32 GroupsX = FMath::Min(NumGroups, 32768u);
        const uint32 GroupsY = FMath::DivideAndRoundUp(NumGroups, GroupsX);
        FVoxelMesherParameters* Parameters = GraphBuilder.AllocParameters<FVoxelMesherParameters>();
        *Parameters = Template;
        Parameters->PassMode = Mode;
        Parameters->GroupsX = GroupsX;
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelMesher %s", Name), PassFlags, Shader, Parameters, FIntVector((int32)GroupsX, (int32)GroupsY, 1));
    }

    /** How far the sampled points reach past a brick's min corner, in half cells of the
     *  brick's level: its own cells and TargetReachCells beyond them. */
    int64 ReachUnits(int32 Cells)
    {
        return 2LL * Cells + 2LL * VoxelBrickMesherGPU::TargetReachCells;
    }
}

int32 VoxelBrickMesherGPU::CellsFor(const FVoxelBrickMesherGPUSettings& Settings)
{
    // A multiple of 8, so that the cells of the target sets (up to 8 cells across) line up
    // with the coarser bricks'. The caps are sized for 32-cell bricks; the hash key holds
    // points of up to 250.
    return FMath::Clamp(Settings.CellsPerBrick / 8 * 8, 8, 64);
}

VoxelBrickMesherGPU::FCaps VoxelBrickMesherGPU::CapsFor(const FVoxelBrickMesherGPUSettings& Settings)
{
    // A 32-cell brick has used up to a few thousand of each; the hash also holds the points
    // around the brick.
    const uint32 Cells = (uint32)FMath::Cube(CellsFor(Settings));
    FCaps Caps;
    Caps.VertexCap = FMath::Clamp(FMath::RoundUpToPowerOfTwo(Cells / 4u), 2048u, 32768u);
    Caps.FaceCap = Caps.VertexCap;
    Caps.TriangleCap = 2u * Caps.VertexCap;
    Caps.HashSize = FMath::Clamp(FMath::RoundUpToPowerOfTwo(Cells / 2u), 4096u, 65536u);
    Caps.TargetHashSize = Caps.HashSize;   // the target cells' corner and gradient points
    return Caps;
}

bool VoxelBrickMesherGPU::MakeBrick(const FVoxelBrickKey& Key, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours,
    const FVoxelBrickMesherGPUSettings& Settings, FVoxelMesherBrickGPU& Out)
{
    // The brick's lattice indices, with the points sampled around it, must fit 32 bits.
    const int32 N = CellsFor(Settings);
    const int64 Unit = 1LL << Key.Level;
    const int64 Scale = 2LL * N * Unit;
    const int64 Reach = VoxelBrickMesherGPUDetail::ReachUnits(N) * Unit;
    const int64 Origin = 1LL << Settings.MaxDepth;
    const int64 Min[3] = { Key.Coord.X * Scale - Origin, Key.Coord.Y * Scale - Origin, Key.Coord.Z * Scale - Origin };
    if (FMath::Abs(Min[0]) + Reach >= MAX_int32 || FMath::Abs(Min[1]) + Reach >= MAX_int32 || FMath::Abs(Min[2]) + Reach >= MAX_int32)
        return false;

    Out = FVoxelMesherBrickGPU();
    Out.MinIndex = FIntVector((int32)Min[0], (int32)Min[1], (int32)Min[2]);
    Out.Level = Key.Level;
    for (int32 d = 0; d < 27; d++)
    {
        const uint32 Bits = d == 13 ? 1u : (uint32)(FMath::Clamp((int32)NeighbourLevels[d], -1, 1) + 1);
        if (d < 16)
            Out.LevelsLo |= Bits << (2 * d);
        else
            Out.LevelsHi |= Bits << (2 * (d - 16));
    }
    Out.Meshed = MeshedNeighbours;
    return true;
}

int32 VoxelBrickMesherGPU::CoarsestMeshableLevel(const FVoxelBrickMesherGPUSettings& Settings)
{
    // The farthest brick of a level starts at the root's edge, 2^MaxDepth units from the
    // centre: MakeBrick's test for it, level by level.
    const int64 Origin = 1LL << FMath::Clamp(Settings.MaxDepth, 0, 62);
    const int64 Reach = VoxelBrickMesherGPUDetail::ReachUnits(CellsFor(Settings));
    int32 Level = -1;
    while (Level < 62 && Origin + Reach * (1LL << (Level + 1)) < MAX_int32)
        Level++;
    return Level;
}

bool VoxelBrickMesherGPU::AreShadersReady()
{
    // GetShader by type returns an empty reference when the shader isn't compiled for this
    // platform (TShaderMapRef would assert).
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    return ShaderMap
        && ShaderMap->GetShader(&FVoxelMesherSampleCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherEdgesCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherRequestCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherSolveCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherTrianglesCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherTargetRequestCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherTargetSolveCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelMesherVertexTargetsCS::GetStaticType(), 0).IsValid();
}

VoxelBrickMesherGPU::FInputs VoxelBrickMesherGPU::UploadInputs(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, const FVoxelEditGPUData& Edits)
{
    using VoxelGPUCommon::UploadStructuredBuffer;
    FInputs Inputs;
    Inputs.Octaves = UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.MesherOctaves"), MakeArrayView(Density.Octaves));
    Inputs.EditCorners = UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.EditCorners"), MakeArrayView(Edits.Corners));
    Inputs.EditChildren = UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.EditChildren"), MakeArrayView(Edits.Children));
    Inputs.EditFlags = UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.EditFlags"), MakeArrayView(Edits.Flags));
    Inputs.EditNodeCount = (uint32)Edits.Flags.Num();
    Inputs.EditRootExtent = Edits.RootExtent;
    return Inputs;
}

VoxelBrickMesherGPU::FOutputs VoxelBrickMesherGPU::AddMesherPasses(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density,
    const FInputs& Inputs, const FVoxelBrickMesherGPUSettings& Settings, TConstArrayView<FVoxelMesherBrickGPU> Bricks, ERDGPassFlags PassFlags,
    const FVoxelBrickMesherGPUDebugView& DebugView)
{
    using namespace VoxelBrickMesherGPUDetail;
    check(Bricks.Num() > 0);

    const uint32 NB = (uint32)Bricks.Num();
    const uint32 UN = (uint32)CellsFor(Settings);
    const uint32 DenseSide = UN + 1u;   // VoxelMesherDenseSide
    const uint32 DenseTotal = DenseSide * DenseSide * DenseSide;
    const uint32 OwnSide = UN + 2u;
    const uint32 CoarseSide = UN / 2u + 2u;
    const uint32 NumSlots = OwnSide * OwnSide * OwnSide + CoarseSide * CoarseSide * CoarseSide;
    // Target sets 0 to 3 (VoxelMesherTargetSetSize): unstitched solves of own cells (the shell
    // within two cells of the faces), then parents and grandparents two cells around the brick.
    uint32 NumTargets = FMath::Cube(UN + 2u) - FMath::Cube(UN - 2u);
    for (uint32 K = 1u; K <= 3u; K++)
        NumTargets += FMath::Cube((UN >> K) + 4u);

    auto MakeBuffer = [&GraphBuilder](uint32 Stride, uint32 Count, const TCHAR* Name)
        {
            return GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(Stride, FMath::Max(Count, 1u)), Name);
        };
    const FCaps Caps = CapsFor(Settings);
    FOutputs Outputs;
    Outputs.Caps = Caps;
    Outputs.Counters = MakeBuffer(4, NB * 4u, TEXT("Voxel.MesherCounters"));
    Outputs.Vertices = MakeBuffer(4, NB * Caps.VertexCap * VertexStride, TEXT("Voxel.MesherVertices"));
    Outputs.Indices = MakeBuffer(4, NB * Caps.TriangleCap * 3u, TEXT("Voxel.MesherIndices"));

    FVoxelMesherParameters Template;
    Template.Octaves = GraphBuilder.CreateSRV(Inputs.Octaves);
    Template.HalfFinest = VoxelDoubleFloat(0.5 * Settings.FinestCell);
    Template.ContinentOctaves = Density.ContinentOctaves;
    Template.MountainOctaves = Density.MountainOctaves;
    Template.DetailOctaves = Density.DetailOctaves;
    Template.InvNormContinent = Density.InvNormContinent;
    Template.InvNormMountain = Density.InvNormMountain;
    Template.InvNormDetail = Density.InvNormDetail;
    Template.PlanetRadius = Density.PlanetRadius;
    Template.NoiseAmplitude = Density.NoiseAmplitude;
    Template.ContinentWeight = Density.ContinentWeight;
    Template.MountainWeight = Density.MountainWeight;
    Template.MountainStart = Density.MountainStart;
    Template.MountainBlend = Density.MountainBlend;
    Template.DetailWeight = Density.DetailWeight;
    Template.RidgeGain = Density.RidgeGain;
    Template.SlopeDamping = Density.SlopeDamping;
    Template.Land = VoxelLandGPU::GetParameters(GraphBuilder, Density);
    Template.EditCorners = GraphBuilder.CreateSRV(Inputs.EditCorners);
    Template.EditChildren = GraphBuilder.CreateSRV(Inputs.EditChildren);
    Template.EditFlags = GraphBuilder.CreateSRV(Inputs.EditFlags);
    Template.EditNodeCount = Inputs.EditNodeCount;
    Template.EditRootExtent = Inputs.EditRootExtent;
    Template.Bricks = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.MesherBricks"), Bricks));
    Template.CellsPerBrick = (int32)UN;
    Template.NumBricks = NB;
    Template.HashSize = Caps.HashSize;
    Template.FaceCap = Caps.FaceCap;
    Template.VertexCap = Caps.VertexCap;
    Template.TriangleCap = Caps.TriangleCap;
    Template.FinestCell = (float)Settings.FinestCell;
    Template.MinFootprint = (float)Density.MinFootprint;
    Template.Dense = GraphBuilder.CreateUAV(MakeBuffer(4, NB * DenseTotal, TEXT("Voxel.MesherDense")));
    Template.HashKeys = GraphBuilder.CreateUAV(MakeBuffer(4, NB * Caps.HashSize, TEXT("Voxel.MesherHashKeys")));
    Template.HashValues = GraphBuilder.CreateUAV(MakeBuffer(4, NB * Caps.HashSize, TEXT("Voxel.MesherHashValues")));
    Template.SlotState = GraphBuilder.CreateUAV(MakeBuffer(4, NB * NumSlots, TEXT("Voxel.MesherSlotState")));
    Template.SlotVertex = GraphBuilder.CreateUAV(MakeBuffer(4, NB * NumSlots, TEXT("Voxel.MesherSlotVertex")));
    Template.Faces = GraphBuilder.CreateUAV(MakeBuffer(4, NB * Caps.FaceCap * 4u, TEXT("Voxel.MesherFaces")));
    Template.Counters = GraphBuilder.CreateUAV(Outputs.Counters);
    Template.Vertices = GraphBuilder.CreateUAV(Outputs.Vertices);
    Template.Indices = GraphBuilder.CreateUAV(Outputs.Indices);
    Template.TargetHashSize = Caps.TargetHashSize;
    Template.TargetState = GraphBuilder.CreateUAV(MakeBuffer(4, NB * NumTargets, TEXT("Voxel.MesherTargetState")));
    Template.TargetData = GraphBuilder.CreateUAV(MakeBuffer(4, NB * NumTargets * 6u, TEXT("Voxel.MesherTargetData")));
    Template.TargetHashKeys = GraphBuilder.CreateUAV(MakeBuffer(4, NB * Caps.TargetHashSize, TEXT("Voxel.MesherTargetHashKeys")));
    Template.TargetHashValues = GraphBuilder.CreateUAV(MakeBuffer(4, NB * Caps.TargetHashSize, TEXT("Voxel.MesherTargetHashValues")));
    // The debug view's pass, when its shader is ready (it isn't part of AreShadersReady, so the
    // terrain never waits for it).
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    const bool bDebugView = DebugView.Mode > 0 && ShaderMap && ShaderMap->GetShader(&FVoxelMesherLandDebugCS::GetStaticType(), 0).IsValid();
    Template.LandDebug = VoxelLandGPU::GetDebugParameters(GraphBuilder, Density, bDebugView ? DebugView.Mode : 0, DebugView.SeaRadius);

    // Only the buffers whose zero means "empty" are cleared. HashValues and TargetHashValues
    // are set to unsampled by the thread that takes a slot (VoxelMesherRequest), and SlotVertex
    // to none where VoxelMesherEdgesPass marks a slot; nothing reads them elsewhere.
    AddClearUAVPass(GraphBuilder, Template.HashKeys, 0u, PassFlags);
    AddClearUAVPass(GraphBuilder, Template.SlotState, 0u, PassFlags);
    AddClearUAVPass(GraphBuilder, Template.Counters, 0u, PassFlags);
    AddClearUAVPass(GraphBuilder, Template.TargetState, 0u, PassFlags);
    AddClearUAVPass(GraphBuilder, Template.TargetHashKeys, 0u, PassFlags);

    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 0u, NB * DenseTotal, TEXT("Sample lattice"));
    AddMesherPass<FVoxelMesherEdgesCS>(GraphBuilder, PassFlags, Template, 0u, NB * 3u * UN * (UN + 1u) * (UN + 1u), TEXT("Edges"));
    AddMesherPass<FVoxelMesherRequestCS>(GraphBuilder, PassFlags, Template, 0u, NB * NumSlots, TEXT("Request edge ends"));
    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 1u, NB * Caps.HashSize, TEXT("Sample edge ends"));
    AddMesherPass<FVoxelMesherRequestCS>(GraphBuilder, PassFlags, Template, 1u, NB * NumSlots, TEXT("Request gradients"));
    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 1u, NB * Caps.HashSize, TEXT("Sample gradients"));
    AddMesherPass<FVoxelMesherSolveCS>(GraphBuilder, PassFlags, Template, 0u, NB * NumSlots, TEXT("Solve"));
    AddMesherPass<FVoxelMesherTrianglesCS>(GraphBuilder, PassFlags, Template, 0u, NB * Caps.FaceCap, TEXT("Triangles"));
    AddMesherPass<FVoxelMesherTargetRequestCS>(GraphBuilder, PassFlags, Template, 0u, NB * NumTargets, TEXT("Request target corners"));
    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 2u, NB * Caps.TargetHashSize, TEXT("Sample target corners"));
    AddMesherPass<FVoxelMesherTargetRequestCS>(GraphBuilder, PassFlags, Template, 2u, NB * NumTargets, TEXT("Mark target neighbours"));
    AddMesherPass<FVoxelMesherTargetRequestCS>(GraphBuilder, PassFlags, Template, 3u, NB * NumTargets, TEXT("Request neighbour corners"));
    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 2u, NB * Caps.TargetHashSize, TEXT("Sample neighbour corners"));
    AddMesherPass<FVoxelMesherTargetRequestCS>(GraphBuilder, PassFlags, Template, 1u, NB * NumTargets, TEXT("Request target gradients"));
    AddMesherPass<FVoxelMesherSampleCS>(GraphBuilder, PassFlags, Template, 2u, NB * Caps.TargetHashSize, TEXT("Sample target gradients"));
    AddMesherPass<FVoxelMesherTargetSolveCS>(GraphBuilder, PassFlags, Template, 0u, NB * NumTargets, TEXT("Solve targets"));
    AddMesherPass<FVoxelMesherVertexTargetsCS>(GraphBuilder, PassFlags, Template, 0u, NB * NumSlots, TEXT("Vertex targets"));
    if (bDebugView)
        AddMesherPass<FVoxelMesherLandDebugCS>(GraphBuilder, PassFlags, Template, 0u, NB * Caps.VertexCap, TEXT("Land debug colours"));
    return Outputs;
}

void VoxelBrickMesherGPU::MeshBricksAsync(TSharedRef<const FVoxelDensityGPUData> Density, TSharedRef<const FVoxelEditGPUData> Edits,
    const FVoxelBrickMesherGPUSettings& Settings, TArray<FVoxelBrickMesherGPURequest> Requests,
    TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone)
{
    using namespace VoxelBrickMesherGPUDetail;
    check(!IsInRenderingThread());

    const TSharedRef<FMesherReadback, ESPMode::ThreadSafe> State = MakeShared<FMesherReadback, ESPMode::ThreadSafe>();
    State->OnDone = MoveTemp(OnDone);

    // Brick records, one per request (the first MaxBricks). A brick out of the 32-bit lattice
    // range gets an empty mesh and ErrorNotMeshed.
    TArray<FVoxelMesherBrickGPU> Bricks;
    TArray<int32> RequestOfBrick;
    State->NumRequests = Requests.Num();
    for (int32 r = 0; r < Requests.Num() && Bricks.Num() < MaxBricks; r++)
    {
        FVoxelMesherBrickGPU Brick;
        if (!MakeBrick(Requests[r].Key, Requests[r].NeighbourLevels, Requests[r].MeshedNeighbours, Settings, Brick))
            continue;
        Bricks.Add(Brick);
        RequestOfBrick.Add(r);
    }
    State->NumBricks = Bricks.Num();
    State->RequestOfBrick = RequestOfBrick;

    // Nothing meshed: one empty entry per request, so callers can index them.
    auto FinishNotMeshed = [State]()
        {
            State->bDone.store(true, std::memory_order_release);
            AsyncTask(ENamedThreads::GameThread, [State]()
                {
                    TArray<FVoxelBrickMesh> Meshes;
                    TArray<FVoxelBrickMesherGPUStats> Stats;
                    Meshes.SetNum(State->NumRequests);
                    Stats.SetNum(State->NumRequests);
                    for (FVoxelBrickMesherGPUStats& S : Stats)
                        S.Errors = VoxelBrickMesherGPU::ErrorNotMeshed;
                    State->OnDone(MoveTemp(Meshes), MoveTemp(Stats));
                });
        };
    if (Bricks.Num() == 0 || !FApp::CanEverRender())
    {
        FinishNotMeshed();
        return;
    }

    ENQUEUE_RENDER_COMMAND(VoxelMesherGPU)([Density, Edits, Settings, Bricks = MoveTemp(Bricks), State, FinishNotMeshed](FRHICommandListImmediate& RHICmdList)
        {
            if (!AreShadersReady())
            {
                FinishNotMeshed();
                return;
            }

            const uint32 NB = (uint32)Bricks.Num();
            FRDGBuilder GraphBuilder(RHICmdList);
            const FInputs Inputs = UploadInputs(GraphBuilder, *Density, *Edits);
            const FOutputs Outputs = AddMesherPasses(GraphBuilder, *Density, Inputs, Settings, Bricks);
            State->Caps = Outputs.Caps;
            const uint32 VertexCap = Outputs.Caps.VertexCap;
            const uint32 TriangleCap = Outputs.Caps.TriangleCap;

            State->Counters = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherCountersReadback"));
            State->Vertices = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherVerticesReadback"));
            State->Indices = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.MesherIndicesReadback"));
            AddEnqueueCopyPass(GraphBuilder, State->Counters.Get(), Outputs.Counters, NB * 4u * 4u);
            AddEnqueueCopyPass(GraphBuilder, State->Vertices.Get(), Outputs.Vertices, NB * VertexCap * VertexStride * 4u);
            AddEnqueueCopyPass(GraphBuilder, State->Indices.Get(), Outputs.Indices, NB * TriangleCap * 3u * 4u);
            GraphBuilder.Execute();
        });

    // Poll the readbacks once per frame on the render thread; build the meshes there and hand
    // them to the game thread.
    VoxelGPUCommon::PollRenderThread([State]() -> bool
        {
            if (State->bDone.load(std::memory_order_acquire)) return true;
            if (!State->Counters.IsValid() || !State->Counters->IsReady() || !State->Vertices->IsReady() || !State->Indices->IsReady())
                return false;

            const uint32 NB = (uint32)State->NumBricks;
            const uint32 VertexCap = State->Caps.VertexCap;
            const uint32 TriangleCap = State->Caps.TriangleCap;
            TArray<FVoxelBrickMesh> Meshes;
            TArray<FVoxelBrickMesherGPUStats> Stats;
            Meshes.SetNum(State->NumRequests);
            Stats.SetNum(State->NumRequests);
            for (FVoxelBrickMesherGPUStats& S : Stats)
                S.Errors = VoxelBrickMesherGPU::ErrorNotMeshed;
            const uint32* Counters = (const uint32*)State->Counters->Lock(NB * 4u * 4u);
            const float* Vertices = (const float*)State->Vertices->Lock(NB * VertexCap * VertexStride * 4u);
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
                        const float* V = Vertices + (b * VertexCap + v) * VertexStride;
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
            return true;
        });
}
