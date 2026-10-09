// voxel.Debug.MesherGPUCheck: meshes bricks of the current brick terrain on the GPU and on the
// CPU (MeshBrickStitched) with the same neighbour levels, and logs how the meshes compare.

#include "AdaptiveVoxelActor.h"
#include "VoxelBrickMesherGPU.h"
#include "VoxelBrickTerrain.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelPlugin.h"
#include "FSparseEditStore.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

namespace VoxelBrickMesherGPUCheck
{
    /** Welds the bricks' vertices by position and counts edges used by one triangle only. */
    struct FSeamCounter
    {
        double Tolerance = 1.0;
        FVector Origin = FVector::ZeroVector;   // positions are welded relative to it (keeps the cell keys small)
        TMap<FIntVector, TArray<int32>> Cells;
        TArray<FVector> Points;
        TMap<TPair<int32, int32>, int32> Edges;

        int32 Weld(const FVector& P)
        {
            const FIntVector Q((int32)FMath::FloorToDouble(P.X / Tolerance), (int32)FMath::FloorToDouble(P.Y / Tolerance), (int32)FMath::FloorToDouble(P.Z / Tolerance));
            for (int32 a = -1; a <= 1; a++)
                for (int32 b = -1; b <= 1; b++)
                    for (int32 c = -1; c <= 1; c++)
                    {
                        if (const TArray<int32>* Found = Cells.Find(Q + FIntVector(a, b, c)))
                        {
                            for (const int32 i : *Found)
                            {
                                if (FVector::Dist(Points[i], P) < Tolerance)
                                    return i;
                            }
                        }
                    }
            Cells.FindOrAdd(Q).Add(Points.Num());
            return Points.Add(P);
        }

        void AddMesh(const FVoxelBrickMesh& Mesh, const FVector& BrickMin)
        {
            TArray<int32> Map;
            Map.SetNum(Mesh.Positions.Num());
            for (int32 v = 0; v < Mesh.Positions.Num(); v++)
                Map[v] = Weld(BrickMin - Origin + FVector(Mesh.Positions[v]));
            for (int32 t = 0; t + 2 < Mesh.Indices.Num(); t += 3)
            {
                for (int32 e = 0; e < 3; e++)
                {
                    const int32 A = Map[Mesh.Indices[t + e]];
                    const int32 B = Map[Mesh.Indices[t + (e + 1) % 3]];
                    Edges.FindOrAdd(TPair<int32, int32>(FMath::Min(A, B), FMath::Max(A, B)))++;
                }
            }
        }

        int32 OpenEdges() const
        {
            int32 Open = 0;
            for (const TPair<TPair<int32, int32>, int32>& Edge : Edges)
                Open += Edge.Value == 1 ? 1 : 0;
            return Open;
        }

        /** Open edges away from the set's outer boundary: every point within Reach of the
         *  edge's midpoint lies in one of Boxes (relative to Origin). These are cracks. */
        int32 InteriorOpenEdges(const TArray<FBox>& Boxes, double Reach) const
        {
            int32 Interior = 0;
            for (const TPair<TPair<int32, int32>, int32>& Edge : Edges)
            {
                if (Edge.Value != 1) continue;
                const FVector Mid = 0.5 * (Points[Edge.Key.Key] + Points[Edge.Key.Value]);
                bool bInside = true;
                for (int32 d = 0; d < 27 && bInside; d++)
                {
                    const FVector Probe = Mid + FVector(d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1) * Reach;
                    bool bInBox = false;
                    for (const FBox& Box : Boxes)
                        bInBox |= Box.IsInsideOrOn(Probe);
                    bInside = bInBox;
                }
                Interior += bInside ? 1 : 0;
            }
            return Interior;
        }
    };

    /** Largest distance from a vertex of Mesh to the nearest vertex of Reference, and how many
     *  are farther than Far. */
    double MaxNearestDistance(const FVoxelBrickMesh& Mesh, const FVoxelBrickMesh& Reference, double CellSize, double Far, int32& OutFar)
    {
        TMap<FIntVector, TArray<int32>> Grid;
        for (int32 i = 0; i < Reference.Positions.Num(); i++)
        {
            const FVector P(Reference.Positions[i]);
            Grid.FindOrAdd(FIntVector((int32)FMath::FloorToDouble(P.X / CellSize), (int32)FMath::FloorToDouble(P.Y / CellSize), (int32)FMath::FloorToDouble(P.Z / CellSize))).Add(i);
        }
        double MaxDistance = 0.0;
        OutFar = 0;
        for (const FVector3f& V : Mesh.Positions)
        {
            const FVector P(V);
            const FIntVector Q((int32)FMath::FloorToDouble(P.X / CellSize), (int32)FMath::FloorToDouble(P.Y / CellSize), (int32)FMath::FloorToDouble(P.Z / CellSize));
            double Best = TNumericLimits<double>::Max();
            for (int32 a = -1; a <= 1; a++)
                for (int32 b = -1; b <= 1; b++)
                    for (int32 c = -1; c <= 1; c++)
                    {
                        if (const TArray<int32>* Cell = Grid.Find(Q + FIntVector(a, b, c)))
                        {
                            for (const int32 i : *Cell)
                                Best = FMath::Min(Best, FVector::Dist(P, FVector(Reference.Positions[i])));
                        }
                    }
            Best = FMath::Min(Best, 2.0 * CellSize);
            MaxDistance = FMath::Max(MaxDistance, Best);
            OutFar += Best > Far ? 1 : 0;
        }
        return MaxDistance;
    }

    void Run(const TArray<FString>& Args, UWorld* World)
    {
        const int32 Count = Args.Num() > 0 ? FMath::Clamp(FCString::Atoi(*Args[0]), 1, VoxelBrickMesherGPU::MaxBricks) : 24;

        AAdaptiveVoxelActor* Terrain = nullptr;
        for (TActorIterator<AAdaptiveVoxelActor> It(World); It; ++It)
        {
            const TSharedPtr<FVoxelBrickTerrain> Bricks = It->GetBrickTerrain();
            if (Bricks.IsValid() && Bricks->GetWantedSignatures().Num() > 0 && It->GetDensityCompositor().IsValid()
                && It->GetDensityCompositor()->GetBaseDensity().IsValid())
            {
                Terrain = *It;
                break;
            }
        }
        if (!Terrain)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelMesherGPU] No brick terrain with bricks in this world."));
            return;
        }

        const TSharedPtr<FVoxelBrickTerrain> BrickTerrain = Terrain->GetBrickTerrain();
        const TSharedPtr<FDensitySampleCompositor> Compositor = Terrain->GetDensityCompositor();
        const FVoxelBrickTerrainSettings& Settings = BrickTerrain->GetSettings();
        const TMap<FVoxelBrickKey, uint64>& Wanted = BrickTerrain->GetWantedSignatures();
        const FVector Camera = Terrain->GetCameraPositionLocal();

        // Bricks that show surface: the nearest at a level transition, then the nearest others;
        // bricks without surface only if there are too few.
        struct FCandidate
        {
            FVoxelBrickKey Key;
            uint64 Signature;
            double Distance;
            bool bTransition;
            bool bSurface;
        };
        TArray<FCandidate> Candidates;
        for (const TPair<FVoxelBrickKey, uint64>& Pair : Wanted)
        {
            int8 Levels[27];
            uint32 Meshed = 0;
            FVoxelBrickTerrain::DecodeSignature(Pair.Value, Levels, Meshed);
            bool bTransition = false;
            for (const int8 Level : Levels)
                bTransition |= Level != 0;
            const FVoxelBrickGrid Grid = Settings.GridFor(Pair.Key.Level);
            const FVector Min = Grid.BrickMin(Pair.Key.Coord);
            const FBox Box(Min, Min + FVector(Grid.BrickSize()));
            Candidates.Add({ Pair.Key, Pair.Value, FMath::Sqrt(Box.ComputeSquaredDistanceToPoint(Camera)), bTransition, BrickTerrain->HoldsSurface(Pair.Key) });
        }
        Candidates.Sort([](const FCandidate& A, const FCandidate& B) { return A.Distance < B.Distance; });
        TArray<FCandidate> Chosen;
        for (const FCandidate& C : Candidates)
        {
            if (C.bSurface && C.bTransition && Chosen.Num() < Count / 2)
                Chosen.Add(C);
        }
        for (int32 Pass = 0; Pass < 2; Pass++)
        {
            for (const FCandidate& C : Candidates)
            {
                if (Chosen.Num() >= Count) break;
                if ((Pass == 0) != C.bSurface) continue;
                if (!Chosen.ContainsByPredicate([&C](const FCandidate& X) { return X.Key == C.Key; }))
                    Chosen.Add(C);
            }
        }

        // CPU reference meshes (with edits, no empty test), and the GPU requests.
        TArray<FVoxelBrickMesherGPURequest> Requests;
        TArray<FVoxelBrickMesh> CpuMeshes;
        TArray<FVector> BrickMins;
        TArray<double> CellSizes;
        const double CpuStart = FPlatformTime::Seconds();
        for (const FCandidate& C : Chosen)
        {
            FVoxelBrickMesherGPURequest& Request = Requests.AddDefaulted_GetRef();
            Request.Key = C.Key;
            FVoxelBrickTerrain::DecodeSignature(C.Signature, Request.NeighbourLevels, Request.MeshedNeighbours);

            const FVoxelBrickGrid Grid = Settings.GridFor(C.Key.Level);
            FVoxelBrickStats Stats;
            VoxelBrickMesher::MeshBrickStitched(*Compositor, Grid, C.Key.Coord, Request.NeighbourLevels, Request.MeshedNeighbours,
                Settings.SlopeBound, true, CpuMeshes.AddDefaulted_GetRef(), Stats);
            BrickMins.Add(Grid.BrickMin(C.Key.Coord));
            CellSizes.Add(Grid.CellSize);
        }
        const double CpuMs = (FPlatformTime::Seconds() - CpuStart) * 1000.0;

        // The terrain's own GPU settings, so the check meshes what the pages mesh.
        const FVoxelBrickMesherGPUSettings GpuSettings = BrickTerrain->GetMesherSettings();
        const TSharedRef<FVoxelDensityGPUData> DensityData = MakeShared<FVoxelDensityGPUData>();
        Compositor->GetBaseDensity()->BuildGPUData(*DensityData);
        const TSharedPtr<FSparseEditStore> EditStore = Compositor->GetEditStore();
        const TSharedRef<FVoxelEditGPUData> EditData = FVoxelEditGPUData::Build(EditStore.Get(), 0.5 * GpuSettings.FinestCell, GpuSettings.MaxDepth);

        int32 Transitions = 0, Levels = 0, WithSurface = 0;
        for (const FCandidate& C : Chosen)
        {
            Transitions += C.bTransition ? 1 : 0;
            WithSurface += C.bSurface ? 1 : 0;
            Levels |= 1 << FMath::Clamp(C.Key.Level, 0, 30);
        }
        FString LevelList;
        for (int32 L = 0; L < 31; L++)
        {
            if (Levels & (1 << L))
                LevelList += FString::Printf(TEXT("%s%d"), LevelList.IsEmpty() ? TEXT("") : TEXT(","), L);
        }
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelMesherGPU] %d bricks (%d showing surface, %d at level transitions, levels %s), %d edit nodes; CPU %.1f ms. Waiting for the GPU..."),
            Requests.Num(), WithSurface, Transitions, *LevelList, EditData->Flags.Num(), CpuMs);

        const double GpuStart = FPlatformTime::Seconds();
        TArray<FVoxelBrickKey> Keys;
        for (const FCandidate& C : Chosen)
            Keys.Add(C.Key);
        VoxelBrickMesherGPU::MeshBricksAsync(DensityData, EditData, GpuSettings, MoveTemp(Requests),
            [Keys, CpuMeshes = MoveTemp(CpuMeshes), BrickMins, CellSizes, GpuStart, CellsPerBrick = Settings.CellsPerBrick](TArray<FVoxelBrickMesh>&& GpuMeshes, TArray<FVoxelBrickMesherGPUStats>&& GpuStats)
            {
                if (GpuMeshes.Num() != CpuMeshes.Num())
                {
                    UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelMesherGPU] GPU returned %d meshes, expected %d."), GpuMeshes.Num(), CpuMeshes.Num());
                    return;
                }
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelMesherGPU] GPU result after %.0f ms (dispatch to readback)."), (FPlatformTime::Seconds() - GpuStart) * 1000.0);

                int32 CountMismatches = 0, ErrorBricks = 0, FarTotal = 0;
                int64 GpuTriangles = 0, CpuTriangles = 0;
                double WorstCells = 0.0;
                double MaxCell = 0.0;
                for (int32 b = 0; b < GpuMeshes.Num(); b++)
                {
                    const FVoxelBrickMesh& G = GpuMeshes[b];
                    const FVoxelBrickMesh& C = CpuMeshes[b];
                    int32 Far = 0;
                    const double Distance = MaxNearestDistance(G, C, CellSizes[b], 0.25 * CellSizes[b], Far);
                    WorstCells = FMath::Max(WorstCells, Distance / CellSizes[b]);
                    MaxCell = FMath::Max(MaxCell, CellSizes[b]);
                    FarTotal += Far;
                    GpuTriangles += G.NumTriangles();
                    CpuTriangles += C.NumTriangles();
                    const bool bMismatch = G.NumTriangles() != C.NumTriangles() || G.Positions.Num() != C.Positions.Num();
                    CountMismatches += bMismatch ? 1 : 0;
                    ErrorBricks += GpuStats[b].Errors != 0 ? 1 : 0;
                    if (bMismatch || GpuStats[b].Errors != 0 || Far > 0)
                    {
                        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelMesherGPU]   brick L%d (%lld,%lld,%lld): GPU %d verts %d tris, CPU %d verts %d tris; max vertex offset %.3g cm, %d farther than a quarter cell; errors 0x%x"),
                            Keys[b].Level, Keys[b].Coord.X, Keys[b].Coord.Y, Keys[b].Coord.Z, G.Positions.Num(), G.NumTriangles(), C.Positions.Num(), C.NumTriangles(),
                            Distance, Far, GpuStats[b].Errors);
                    }
                }

                // Seams: the GPU meshes, welded, should open only where the CPU meshes do (the
                // outer boundary of the chosen set).
                FSeamCounter GpuSeams, CpuSeams;
                GpuSeams.Tolerance = CpuSeams.Tolerance = 1e-3 * MaxCell;
                GpuSeams.Origin = CpuSeams.Origin = BrickMins.Num() > 0 ? BrickMins[0] : FVector::ZeroVector;
                TArray<FBox> Boxes;
                for (int32 b = 0; b < GpuMeshes.Num(); b++)
                {
                    GpuSeams.AddMesh(GpuMeshes[b], BrickMins[b]);
                    CpuSeams.AddMesh(CpuMeshes[b], BrickMins[b]);
                    const FVector Min = BrickMins[b] - GpuSeams.Origin;
                    Boxes.Add(FBox(Min, Min + FVector(CellSizes[b] * CellsPerBrick)));
                }
                const int32 GpuCracks = GpuSeams.InteriorOpenEdges(Boxes, 1.5 * MaxCell);
                const int32 CpuCracks = CpuSeams.InteriorOpenEdges(Boxes, 1.5 * MaxCell);
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelMesherGPU] %d bricks: triangles GPU %lld / CPU %lld; %d bricks with different counts; worst vertex offset %.3g cells; %d vertices farther than a quarter cell; %d bricks with GPU errors; open edges after welding GPU %d / CPU %d, of them inside the set (cracks) GPU %d / CPU %d"),
                    GpuMeshes.Num(), GpuTriangles, CpuTriangles, CountMismatches, WorstCells, FarTotal, ErrorBricks, GpuSeams.OpenEdges(), CpuSeams.OpenEdges(), GpuCracks, CpuCracks);
            });
    }

    FAutoConsoleCommandWithWorldAndArgs Command(
        TEXT("voxel.Debug.MesherGPUCheck"),
        TEXT("Meshes bricks of the brick terrain near the camera on the GPU and the CPU and logs how they compare. Arg: number of bricks (default 24, max 32)."),
        FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&Run));
}
