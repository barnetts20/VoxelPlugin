#include "VoxelLandRiverBuild.h"
#include "VoxelLandRivers.h"
#include "VoxelPlugin.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"

DECLARE_GPU_STAT(VoxelLandRivers);

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandRiverBuildParameters, )
    SHADER_PARAMETER(uint32, RiverN)
    SHADER_PARAMETER(uint32, RiverPad)
    SHADER_PARAMETER(float, RiverRadiusM)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float2>, RiverTan)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, RiverHeight)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, RiverWater)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, RiverFlow)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, RiverRec)
    SHADER_PARAMETER(float, RiverMinLog2)
    SHADER_PARAMETER(float, RiverWidthScale)
    SHADER_PARAMETER(float, RiverValleyScale)
    SHADER_PARAMETER(float, RiverReachScale)
    SHADER_PARAMETER(float, RiverMarginCells)
    SHADER_PARAMETER(float, RiverSimplifyCells)
    SHADER_PARAMETER(float, RiverSimplifyFloorM)
    SHADER_PARAMETER(float, RiverSimplifyFloorShare)
    SHADER_PARAMETER(float, RiverSimplifyLog2Area)
    SHADER_PARAMETER(uint32, RiverTilesPerFace)
    SHADER_PARAMETER(uint32, RiverNodeCap)
    SHADER_PARAMETER(uint32, RiverSegCap)
    SHADER_PARAMETER(uint32, RiverCandCap)
    SHADER_PARAMETER(uint32, RiverDataCap)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, RiverCounts)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, RiverCellFlags)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, RiverCellNode)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeCell)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeDown)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodePred)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeUp)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeKind)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeFloor)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeArea)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeSeg)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodePos)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodePosOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeJumpIn)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeJumpOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodeSIn)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodeSOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodeEndIn)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, NodeEndOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeIdxIn)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, NodeIdxOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SegNode)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SegEnd)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TileStart)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TileFill)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TileBlock)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, TileCand)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, RiverTiles)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, RiverData)
    SHADER_PARAMETER(uint32, GroupsX)
    SHADER_PARAMETER(uint32, ScanCount)
    SHADER_PARAMETER(uint32, ScanGroups)
    SHADER_PARAMETER(uint32, ScanTotalSlot)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ScanData)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ScanGroupSums)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_LAND_RIVER_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelLandRiverBuildParameters; \
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

#define VOXEL_LAND_RIVER_SHADERS(X) \
    X(FVoxelRiverScanSumCS, "ScanSumCS") \
    X(FVoxelRiverScanTopCS, "ScanTopCS") \
    X(FVoxelRiverScanApplyCS, "ScanApplyCS") \
    X(FVoxelRiverSeedCS, "SeedCS") \
    X(FVoxelRiverClosureCS, "ClosureCS") \
    X(FVoxelRiverRolesCS, "RolesCS") \
    X(FVoxelRiverNodeCountCS, "NodeCountCS") \
    X(FVoxelRiverNodeInitCS, "NodeInitCS") \
    X(FVoxelRiverNodeLinkCS, "NodeLinkCS") \
    X(FVoxelRiverSmoothCS, "SmoothCS") \
    X(FVoxelRiverFloorInitCS, "FloorInitCS") \
    X(FVoxelRiverFloorPushCS, "FloorPushCS") \
    X(FVoxelRiverJumpCS, "JumpCS") \
    X(FVoxelRiverChainInitCS, "ChainInitCS") \
    X(FVoxelRiverChainJumpCS, "ChainJumpCS") \
    X(FVoxelRiverEndInitCS, "EndInitCS") \
    X(FVoxelRiverEndJumpCS, "EndJumpCS") \
    X(FVoxelRiverJumpCheckCS, "JumpCheckCS") \
    X(FVoxelRiverTrunkClearCS, "TrunkClearCS") \
    X(FVoxelRiverTrunkAreaCS, "TrunkAreaCS") \
    X(FVoxelRiverTrunkMainCS, "TrunkMainCS") \
    X(FVoxelRiverTrunkInitCS, "TrunkInitCS") \
    X(FVoxelRiverTrunkJumpCS, "TrunkJumpCS") \
    X(FVoxelRiverTrunkCheckCS, "TrunkCheckCS") \
    X(FVoxelRiverSegCountCS, "SegCountCS") \
    X(FVoxelRiverSegmentCS, "SegmentCS") \
    X(FVoxelRiverCoverCountCS, "CoverCountCS") \
    X(FVoxelRiverCoverFillCS, "CoverFillCS") \
    X(FVoxelRiverTileSortCS, "TileSortCS") \
    X(FVoxelRiverTileWriteCS, "TileWriteCS")

#define VOXEL_LAND_RIVER_DECLARE(ShaderName, Entry) VOXEL_LAND_RIVER_SHADER(ShaderName)
VOXEL_LAND_RIVER_SHADERS(VOXEL_LAND_RIVER_DECLARE)
#undef VOXEL_LAND_RIVER_DECLARE

#define VOXEL_LAND_RIVER_IMPLEMENT(ShaderName, Entry) IMPLEMENT_GLOBAL_SHADER(ShaderName, "/Plugin/VoxelPlugin/Private/VoxelLandRiverBuild.usf", Entry, SF_Compute);
VOXEL_LAND_RIVER_SHADERS(VOXEL_LAND_RIVER_IMPLEMENT)
#undef VOXEL_LAND_RIVER_IMPLEMENT

namespace VoxelLandRiverBuildDetail
{
    constexpr int32 JumpRounds = 20;          // VOXEL_RIVER_JUMP_ROUNDS
    constexpr int32 SmoothIterations = 4;     // VOXEL_RIVER_SMOOTH_ITERATIONS
    constexpr float MarginCells = 0.25f;
    // R1's simplification tolerances (VoxelLandRiverBuild.ush).
    constexpr float SimplifyCells = 0.3f;
    constexpr float SimplifyFloorM = 3.0f;
    constexpr float SimplifyFloorShare = 0.05f;
    constexpr float SimplifyLog2Area = 0.25f;

    static_assert(JumpRounds % 2 == 0 && SmoothIterations % 2 == 0, "the double buffers end where they began");

    FRDGBufferRef Make(FRDGBuilder& GraphBuilder, uint32 Stride, uint32 Count, const TCHAR* Name, bool bReadback = false)
    {
        FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(Stride, FMath::Max(Count, 1u));
        if (bReadback) Desc.Usage |= EBufferUsageFlags::SourceCopy;
        return GraphBuilder.CreateBuffer(Desc, Name);
    }

    /** One pass recorder: the buffers' views made once, the double buffers bound by parity. */
    struct FRecorder
    {
        FRDGBuilder& GraphBuilder;
        FVoxelLandRiverBuildParameters Base;
        FRDGBufferUAVRef Pos[2] = {}, Jump[2] = {}, S[2] = {}, End[2] = {}, Idx[2] = {};
        FRDGBufferUAVRef GroupSums = nullptr;
        // The parities: which of each pair is "in" (Pos: the current positions).
        int32 P = 0, J = 1, SI = 1, EI = 1, II = 1;

        explicit FRecorder(FRDGBuilder& InGraphBuilder) : GraphBuilder(InGraphBuilder) {}

        FVoxelLandRiverBuildParameters* Params(uint32 Threads)
        {
            FVoxelLandRiverBuildParameters* Out = GraphBuilder.AllocParameters<FVoxelLandRiverBuildParameters>();
            *Out = Base;
            Out->NodePos = Pos[P];
            Out->NodePosOut = Pos[1 - P];
            Out->NodeJumpIn = Jump[J];
            Out->NodeJumpOut = Jump[1 - J];
            Out->NodeSIn = S[SI];
            Out->NodeSOut = S[1 - SI];
            Out->NodeEndIn = End[EI];
            Out->NodeEndOut = End[1 - EI];
            Out->NodeIdxIn = Idx[II];
            Out->NodeIdxOut = Idx[1 - II];
            const uint32 Groups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
            Out->GroupsX = FMath::Min(Groups, 32768u);
            return Out;
        }

        static FIntVector GroupsFor(uint32 Threads, uint32 GroupsX)
        {
            const uint32 Groups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
            return FIntVector((int32)GroupsX, (int32)FMath::DivideAndRoundUp(Groups, GroupsX), 1);
        }

        template<typename TShader>
        void Pass(uint32 Threads, const TCHAR* Name)
        {
            FVoxelLandRiverBuildParameters* Parameters = Params(Threads);
            TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand rivers %s", Name), Shader, Parameters, GroupsFor(Threads, Parameters->GroupsX));
        }

        /** The exclusive prefix sum of Count words of Data, in place; the total to Slot. */
        void Scan(FRDGBufferUAVRef Data, uint32 Count, uint32 Slot, const TCHAR* Name)
        {
            const uint32 Groups = FMath::Max(FMath::DivideAndRoundUp(Count, 64u), 1u);
            auto Setup = [&](FVoxelLandRiverBuildParameters* Parameters) {
                Parameters->ScanCount = Count;
                Parameters->ScanGroups = Groups;
                Parameters->ScanTotalSlot = Slot;
                Parameters->ScanData = Data;
                Parameters->ScanGroupSums = GroupSums;
            };
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            {
                FVoxelLandRiverBuildParameters* Parameters = Params(Count);
                Setup(Parameters);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand rivers %s scan sums", Name), TShaderMapRef<FVoxelRiverScanSumCS>(ShaderMap),
                    Parameters, GroupsFor(Count, Parameters->GroupsX));
            }
            {
                FVoxelLandRiverBuildParameters* Parameters = Params(1);
                Setup(Parameters);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand rivers %s scan", Name), TShaderMapRef<FVoxelRiverScanTopCS>(ShaderMap),
                    Parameters, FIntVector(1, 1, 1));
            }
            {
                FVoxelLandRiverBuildParameters* Parameters = Params(Count);
                Setup(Parameters);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand rivers %s scan apply", Name), TShaderMapRef<FVoxelRiverScanApplyCS>(ShaderMap),
                    Parameters, GroupsFor(Count, Parameters->GroupsX));
            }
        }
    };
}

namespace VLRB = VoxelLandRiverBuildDetail;

bool VoxelLandRiverBuild::AreShadersReady()
{
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    if (!ShaderMap) return false;
#define VOXEL_LAND_RIVER_READY(ShaderName, Entry) if (!ShaderMap->GetShader(&ShaderName::GetStaticType(), 0).IsValid()) return false;
    VOXEL_LAND_RIVER_SHADERS(VOXEL_LAND_RIVER_READY)
#undef VOXEL_LAND_RIVER_READY
    return true;
}

VoxelLandRiverBuild::FCaps VoxelLandRiverBuild::EstimateCaps(const FSettings& S)
{
    // Keep equal to river_build_cpu.h (Tools/LandRiverHarness), which measured it: the river
    // cells are about 0.36 (RiverMinArea / cell area)^-0.45 of the land on the test bakes (6% at
    // 1024 cells a face on a 1,000 km planet), a node a river cell; 0.77-0.82 segments a node;
    // 2-3 tile candidates a segment.
    const double Side = (double)S.N + 2.0 * S.Pad;
    const double Cells = 6.0 * Side * Side;
    const double Land = S.LandCells > 0 ? (double)S.LandCells : Cells / 3.0;
    const double CellKm = 0.5 * UE_DOUBLE_PI * S.RadiusM / FMath::Max(S.N, 1) * 1.0e-3;
    const double Ratio = FMath::Max((double)S.MinAreaKm2, 1e-3) / FMath::Max(CellKm * CellKm, 1e-9);
    const double Share = FMath::Min(1.0, 0.5 * FMath::Pow(FMath::Max(Ratio, 1.0), -0.45));
    const double Tiles = 6.0 * FMath::Square((double)TilesPerFace(S.N));
    FCaps C;
    C.Nodes = (uint32)FMath::Min(Cells, Land * Share * 1.1 + 4096.0);
    C.Segments = (uint32)((double)C.Nodes * 0.9 + 1024.0);
    C.Candidates = (uint32)((double)C.Segments * 4.0 + 4096.0);
    C.Data = (uint32)((double)C.Segments * SegFloats + FMath::Min(Tiles, (double)C.Candidates / 4.0) * FrameFloats + (double)C.Candidates * EntryFloats + 4096.0);
    return C;
}

VoxelLandRiverBuild::FCaps VoxelLandRiverBuild::GrowCaps(const FCaps& Caps, const uint32* Counts)
{
    // Keep equal to river_build_cpu.h. A stage that was cut undercounts the ones after it, so
    // they grow by its shortfall too (else each would take a build of its own).
    const double NodeShort = FMath::Max(1.0, (double)Counts[CountNodes] / FMath::Max(Caps.Nodes, 1u));
    const double SegShort = FMath::Max(NodeShort, (double)Counts[CountSegments] * NodeShort / FMath::Max(Caps.Segments, 1u));
    const double CandShort = FMath::Max(SegShort, (double)Counts[CountCandidates] * SegShort / FMath::Max(Caps.Candidates, 1u));
    const double Used = (double)SegFloats * Counts[CountSegments] + Counts[CountBlocks];
    const double DataShort = FMath::Max(CandShort, Used * CandShort / FMath::Max(Caps.Data, 1u));
    auto Grow = [](uint32 Have, double Short) { return Short <= 1.0 ? Have : (uint32)FMath::Min(Have * Short * 1.125 + 1024.0, (double)MAX_uint32); };
    FCaps Out;
    Out.Nodes = Grow(Caps.Nodes, NodeShort);
    Out.Segments = Grow(Caps.Segments, SegShort);
    Out.Candidates = Grow(Caps.Candidates, CandShort);
    Out.Data = Grow(Caps.Data, DataShort);
    return Out;
}

VoxelLandRiverBuild::FOutputs VoxelLandRiverBuild::AddPasses(FRDGBuilder& GraphBuilder, const FInputs& Inputs, const FSettings& S, const FCaps& Caps)
{
    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelLandRivers, "VoxelLand rivers");
    const uint32 Side = (uint32)(S.N + 2 * S.Pad);
    const uint32 NumCells = 6u * Side * Side;
    const uint32 T = (uint32)TilesPerFace(S.N);
    const uint32 TileCount = 6u * T * T;
    const uint32 NodeCap = FMath::Max(Caps.Nodes, 1u);
    const uint32 SegCap = FMath::Max(Caps.Segments, 1u);
    const uint32 CandCap = FMath::Max(Caps.Candidates, 1u);
    const uint32 DataCap = FMath::Max(Caps.Data, 1u);

    FOutputs Out;
    Out.Tiles = VLRB::Make(GraphBuilder, 4, TileCount + 1, TEXT("Voxel.LandRiverTiles"), true);
    Out.Data = VLRB::Make(GraphBuilder, 4, DataCap, TEXT("Voxel.LandRiverData"), true);
    Out.Counts = VLRB::Make(GraphBuilder, 4, NumCounts, TEXT("Voxel.LandRiverCounts"), true);
    FRDGBufferRef CellFlags = Inputs.CellFlags ? Inputs.CellFlags : VLRB::Make(GraphBuilder, 4, NumCells, TEXT("Voxel.LandRiverCellFlags"));
    FRDGBufferRef CellNode = Inputs.CellNode ? Inputs.CellNode : VLRB::Make(GraphBuilder, 4, NumCells, TEXT("Voxel.LandRiverCellNode"));
    FRDGBufferRef TileStart = VLRB::Make(GraphBuilder, 4, TileCount, TEXT("Voxel.LandRiverTileStart"));
    FRDGBufferRef TileFill = VLRB::Make(GraphBuilder, 4, TileCount, TEXT("Voxel.LandRiverTileFill"));

    VLRB::FRecorder Rec(GraphBuilder);
    FVoxelLandRiverBuildParameters& B = Rec.Base;
    B.RiverN = (uint32)S.N;
    B.RiverPad = (uint32)S.Pad;
    B.RiverRadiusM = S.RadiusM;
    B.RiverTan = GraphBuilder.CreateSRV(Inputs.Tan);
    B.RiverHeight = GraphBuilder.CreateSRV(Inputs.Height);
    B.RiverWater = GraphBuilder.CreateSRV(Inputs.Water);
    B.RiverFlow = GraphBuilder.CreateSRV(Inputs.Flow);
    B.RiverRec = GraphBuilder.CreateSRV(Inputs.Rec);
    B.RiverMinLog2 = (float)FMath::Log2(FMath::Max((double)S.MinAreaKm2, 1e-6) * 1.0e6);
    B.RiverWidthScale = FMath::Max(S.WidthScale, 0.1f);
    B.RiverValleyScale = FMath::Max(S.ValleyScale, 0.1f);
    B.RiverReachScale = (float)VoxelLandRivers::ReachScale;
    B.RiverMarginCells = VLRB::MarginCells;
    B.RiverSimplifyCells = VLRB::SimplifyCells;
    B.RiverSimplifyFloorM = VLRB::SimplifyFloorM;
    B.RiverSimplifyFloorShare = VLRB::SimplifyFloorShare;
    B.RiverSimplifyLog2Area = VLRB::SimplifyLog2Area;
    B.RiverTilesPerFace = T;
    B.RiverNodeCap = NodeCap;
    B.RiverSegCap = SegCap;
    B.RiverCandCap = CandCap;
    B.RiverDataCap = DataCap;
    B.RiverCounts = GraphBuilder.CreateUAV(Out.Counts);
    B.RiverCellFlags = GraphBuilder.CreateUAV(CellFlags);
    B.RiverCellNode = GraphBuilder.CreateUAV(CellNode);
    B.NodeCell = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeCell")));
    B.NodeDown = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeDown")));
    B.NodePred = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodePred")));
    B.NodeUp = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeUp")));
    B.NodeKind = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeKind")));
    B.NodeFloor = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeFloor")));
    B.NodeArea = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeArea")));
    FRDGBufferRef NodeSeg = VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeSeg"));
    B.NodeSeg = GraphBuilder.CreateUAV(NodeSeg);
    for (int32 i = 0; i < 2; i++)
    {
        Rec.Pos[i] = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, 4 * NodeCap, TEXT("Voxel.LandRiverNodePos")));
        Rec.Jump[i] = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeJump")));
        // The places, then the meanders' phase (R3).
        Rec.S[i] = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, 2 * NodeCap, TEXT("Voxel.LandRiverNodeS")));
        Rec.End[i] = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeEnd")));
        Rec.Idx[i] = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, NodeCap, TEXT("Voxel.LandRiverNodeIdx")));
    }
    B.SegNode = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, SegCap, TEXT("Voxel.LandRiverSegNode")));
    B.SegEnd = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, SegCap, TEXT("Voxel.LandRiverSegEnd")));
    B.TileStart = GraphBuilder.CreateUAV(TileStart);
    B.TileFill = GraphBuilder.CreateUAV(TileFill);
    FRDGBufferRef TileBlock = VLRB::Make(GraphBuilder, 4, TileCount, TEXT("Voxel.LandRiverTileBlock"));
    B.TileBlock = GraphBuilder.CreateUAV(TileBlock);
    B.TileCand = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, CandCap, TEXT("Voxel.LandRiverTileCand")));
    B.RiverTiles = GraphBuilder.CreateUAV(Out.Tiles);
    B.RiverData = GraphBuilder.CreateUAV(Out.Data);
    const uint32 MaxScan = FMath::Max3(NumCells, NodeCap, TileCount);
    Rec.GroupSums = GraphBuilder.CreateUAV(VLRB::Make(GraphBuilder, 4, FMath::DivideAndRoundUp(MaxScan, 64u), TEXT("Voxel.LandRiverScanGroups")));
    B.ScanData = B.RiverCellNode;
    B.ScanGroupSums = Rec.GroupSums;

    AddClearUAVPass(GraphBuilder, B.RiverCounts, 0u);
    AddClearUAVPass(GraphBuilder, B.TileStart, 0u);
    AddClearUAVPass(GraphBuilder, B.TileFill, 0u);

    // The cells: seeds, the rivers below them, the terminals and outlets, the nodes.
    Rec.Pass<FVoxelRiverSeedCS>(NumCells, TEXT("seeds"));
    Rec.Pass<FVoxelRiverClosureCS>(NumCells, TEXT("closure"));
    Rec.Pass<FVoxelRiverRolesCS>(NumCells, TEXT("roles"));
    Rec.Pass<FVoxelRiverNodeCountCS>(NumCells, TEXT("node count"));
    Rec.Scan(B.RiverCellNode, NumCells, CountNodes, TEXT("nodes"));
    Rec.Pass<FVoxelRiverNodeInitCS>(NumCells, TEXT("node init"));
    Rec.Pass<FVoxelRiverNodeLinkCS>(NodeCap, TEXT("node links"));
    // The positions smoothed (the current buffer ends where it began).
    for (int32 k = 0; k < VLRB::SmoothIterations; k++)
    {
        Rec.Pass<FVoxelRiverSmoothCS>(NodeCap, TEXT("smoothing"));
        Rec.P ^= 1;
    }
    // The floors: the upstream minimum.
    Rec.J = 1;
    Rec.Pass<FVoxelRiverFloorInitCS>(NodeCap, TEXT("floor init"));
    Rec.J = 0;
    for (int32 r = 0; r < VLRB::JumpRounds; r++)
    {
        Rec.Pass<FVoxelRiverFloorPushCS>(NodeCap, TEXT("floor push"));
        Rec.Pass<FVoxelRiverJumpCS>(NodeCap, TEXT("floor jump"));
        Rec.J ^= 1;
    }
    Rec.Pass<FVoxelRiverJumpCheckCS>(NodeCap, TEXT("floor check"));
    // The places along the chains.
    Rec.J = 1; Rec.SI = 1; Rec.II = 1;
    Rec.Pass<FVoxelRiverChainInitCS>(NodeCap, TEXT("chain init"));
    Rec.J = 0; Rec.SI = 0; Rec.II = 0;
    for (int32 r = 0; r < VLRB::JumpRounds; r++)
    {
        Rec.Pass<FVoxelRiverChainJumpCS>(NodeCap, TEXT("chain jump"));
        Rec.J ^= 1; Rec.SI ^= 1; Rec.II ^= 1;
    }
    Rec.Pass<FVoxelRiverJumpCheckCS>(NodeCap, TEXT("chain check"));
    // The chains' lengths (the places stay in SI).
    Rec.J = 1; Rec.EI = 1;
    Rec.Pass<FVoxelRiverEndInitCS>(NodeCap, TEXT("chain length init"));
    Rec.J = 0; Rec.EI = 0;
    for (int32 r = 0; r < VLRB::JumpRounds; r++)
    {
        Rec.Pass<FVoxelRiverEndJumpCS>(NodeCap, TEXT("chain length jump"));
        Rec.J ^= 1; Rec.EI ^= 1;
    }
    Rec.Pass<FVoxelRiverJumpCheckCS>(NodeCap, TEXT("chain length check"));
    // The trunks (N1b), on the free jump buffers: In the main donors then the tops, Out the
    // largest drainage in; the segments read the tops.
    Rec.J = 0;
    Rec.Pass<FVoxelRiverTrunkClearCS>(NodeCap, TEXT("trunk clear"));
    Rec.Pass<FVoxelRiverTrunkAreaCS>(NodeCap, TEXT("trunk area"));
    Rec.Pass<FVoxelRiverTrunkMainCS>(NodeCap, TEXT("trunk main donor"));
    Rec.Pass<FVoxelRiverTrunkInitCS>(NodeCap, TEXT("trunk init"));
    for (int32 r = 0; r < VLRB::JumpRounds; r++)
        Rec.Pass<FVoxelRiverTrunkJumpCS>(NodeCap, TEXT("trunk jump"));
    Rec.Pass<FVoxelRiverTrunkCheckCS>(NodeCap, TEXT("trunk check"));
    // The segments.
    Rec.Pass<FVoxelRiverSegCountCS>(NodeCap, TEXT("segment count"));
    Rec.Scan(B.NodeSeg, NodeCap, CountSegments, TEXT("segments"));
    Rec.Pass<FVoxelRiverSegmentCS>(NodeCap, TEXT("segments"));
    // The tiles.
    Rec.Pass<FVoxelRiverCoverCountCS>(SegCap, TEXT("cover count"));
    Rec.Scan(B.TileStart, TileCount, CountCandidates, TEXT("tile lists"));
    Rec.Pass<FVoxelRiverCoverFillCS>(SegCap, TEXT("cover fill"));
    Rec.Pass<FVoxelRiverTileSortCS>(TileCount, TEXT("tile sort"));
    Rec.Scan(B.TileBlock, TileCount, CountBlocks, TEXT("tile blocks"));
    Rec.Pass<FVoxelRiverTileWriteCS>(TileCount + 1, TEXT("tile write"));
    return Out;
}

// ---------------------------------------------------------------------------------------------
// FPending
// ---------------------------------------------------------------------------------------------

VoxelLandRiverBuild::FPending::FPending() = default;
VoxelLandRiverBuild::FPending::~FPending() = default;
VoxelLandRiverBuild::FPending::FPending(FPending&&) = default;
VoxelLandRiverBuild::FPending& VoxelLandRiverBuild::FPending::operator=(FPending&&) = default;

void VoxelLandRiverBuild::FPending::Enqueue(FRDGBuilder& GraphBuilder, const FOutputs& Outputs, const FSettings& Settings, const FCaps& InCaps)
{
    Reset();
    N = Settings.N;
    Caps = InCaps;
    const uint32 T = (uint32)TilesPerFace(N);
    TileWords = 6u * T * T + 1u;
    Tiles = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandRiverTilesReadback"));
    Data = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandRiverDataReadback"));
    Counts = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandRiverCountsReadback"));
    AddEnqueueCopyPass(GraphBuilder, Tiles.Get(), Outputs.Tiles, TileWords * 4u);
    AddEnqueueCopyPass(GraphBuilder, Data.Get(), Outputs.Data, FMath::Max(Caps.Data, 1u) * 4u);
    AddEnqueueCopyPass(GraphBuilder, Counts.Get(), Outputs.Counts, (uint32)NumCounts * 4u);
    // The tables stay on the GPU for the density (a build that doesn't fit drops them).
    GPUTiles = GraphBuilder.ConvertToExternalBuffer(Outputs.Tiles);
    GPUData = GraphBuilder.ConvertToExternalBuffer(Outputs.Data);
}

bool VoxelLandRiverBuild::FPending::IsValid() const
{
    return Tiles.IsValid() && Data.IsValid() && Counts.IsValid();
}

bool VoxelLandRiverBuild::FPending::IsReady() const
{
    return IsValid() && Tiles->IsReady() && Data->IsReady() && Counts->IsReady();
}

void VoxelLandRiverBuild::FPending::Reset()
{
    Tiles.Reset();
    Data.Reset();
    Counts.Reset();
    GPUTiles.SafeRelease();
    GPUData.SafeRelease();
}

bool VoxelLandRiverBuild::FPending::Read(FResult& Out)
{
    Out = FResult();
    if (!IsReady())
    {
        Reset();
        return false;
    }
    bool bOk = true;
    {
        const void* Source = Counts->Lock((uint32)NumCounts * 4u);
        if (Source) { FMemory::Memcpy(Out.Counts, Source, sizeof(Out.Counts)); Counts->Unlock(); }
        else bOk = false;
    }
    const uint32 Segments = Out.Counts[CountSegments];
    const uint64 Used = (uint64)SegFloats * Segments + Out.Counts[CountBlocks];
    Out.bFits = bOk && Out.Counts[CountNodes] <= Caps.Nodes && Segments <= Caps.Segments && Out.Counts[CountCandidates] <= Caps.Candidates
        && Used <= (uint64)Caps.Data && (Out.Counts[CountErrors] == 0);
    Out.Needed = GrowCaps(Caps, Out.Counts);
    Out.TilesPerFace = TilesPerFace(N);
    if (bOk && Out.bFits)
    {
        const void* TileSource = Tiles->Lock(TileWords * 4u);
        if (TileSource)
        {
            Out.Tiles.SetNumUninitialized((int32)TileWords);
            FMemory::Memcpy(Out.Tiles.GetData(), TileSource, TileWords * 4u);
            Tiles->Unlock();
        }
        else bOk = false;
        const void* DataSource = Data->Lock(FMath::Max(Caps.Data, 1u) * 4u);
        if (DataSource)
        {
            Out.Data.SetNumUninitialized((int32)Used);
            FMemory::Memcpy(Out.Data.GetData(), DataSource, Used * 4u);
            Data->Unlock();
        }
        else bOk = false;
        Out.GPUTiles = GPUTiles;
        Out.GPUData = GPUData;
    }
    Reset();
    return bOk;
}
