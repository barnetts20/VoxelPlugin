// voxel.Debug.DensityGPUCheck: samples the density on the GPU and on the CPU at the same
// lattice points around the surface below the camera, and logs how far they differ.

#include "AdaptiveVoxelActor.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelPlugin.h"
#include "FSparseEditStore.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

namespace VoxelDensityGPUCheck
{
    constexpr int32 CellsPerBrick = 32;

    struct FBlockInfo
    {
        int32 Level;
        double CellSize;
    };

    void Run(const TArray<FString>& Args, UWorld* World)
    {
        const int32 NumLevels = Args.Num() > 0 ? FMath::Clamp(FCString::Atoi(*Args[0]), 1, 30) : 8;
        const FString Mode = Args.Num() > 1 ? Args[1] : FString();
        const EVoxelDensityGPUOutput Output = Mode.Equals(TEXT("radial"), ESearchCase::IgnoreCase) ? EVoxelDensityGPUOutput::Radial
            : (Mode.Equals(TEXT("edits"), ESearchCase::IgnoreCase) ? EVoxelDensityGPUOutput::Edits : EVoxelDensityGPUOutput::Density);

        AAdaptiveVoxelActor* Terrain = nullptr;
        for (TActorIterator<AAdaptiveVoxelActor> It(World); It; ++It)
        {
            const TSharedPtr<FDensitySampleCompositor> Compositor = It->GetDensityCompositor();
            if (Compositor.IsValid() && Compositor->GetBaseDensity().IsValid())
            {
                Terrain = *It;
                break;
            }
        }
        if (!Terrain)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelDensityGPU] No initialized terrain in this world."));
            return;
        }

        const TSharedPtr<const FVoxelDensityCPU> Density = Terrain->GetDensityCompositor()->GetBaseDensity();
        const TSharedRef<FVoxelDensityGPUData> Data = MakeShared<FVoxelDensityGPUData>();
        Density->BuildGPUData(*Data);
        const double HalfFinest = 0.5 * Terrain->ActualPrecision;
        const TSharedPtr<FSparseEditStore> EditStore = Terrain->GetDensityCompositor()->GetEditStore();
        const TSharedRef<FVoxelEditGPUData> EditData = FVoxelEditGPUData::Build(EditStore.Get(), HalfFinest, Terrain->MaxDepth);

        // The surface point below the camera, in half finest cells.
        const FVector Camera = Terrain->GetCameraPositionLocal();
        const FVector Dir = Camera.IsNearlyZero() ? FVector::UpVector : Camera.GetSafeNormal();
        const FVector Surface = Dir * (Density->GetPlanetRadius() + Density->HeightAt(Dir, Terrain->ActualPrecision)) / HalfFinest;

        // Per level, the 2x2x2 bricks around the brick corner nearest that point, every
        // lattice point of each brick (33^3), at the level's cell size.
        TArray<FVoxelDensityGPUBlock> Blocks;
        TArray<FBlockInfo> Infos;
        for (int32 Level = 0; Level < NumLevels && Level <= Terrain->MaxDepth - 5; Level++)
        {
            const int64 Step = 2LL << Level;
            const int64 BrickUnits = Step * CellsPerBrick;
            for (int32 b = 0; b < 8; b++)
            {
                FVoxelDensityGPUBlock Block;
                int64 Min[3];
                for (int32 Axis = 0; Axis < 3; Axis++)
                {
                    const int64 Corner = (int64)FMath::RoundToDouble(Surface[Axis] / (double)BrickUnits);
                    Min[Axis] = (Corner - 1 + ((b >> Axis) & 1)) * BrickUnits;
                }
                if (FMath::Abs(Min[0]) + BrickUnits >= MAX_int32 || FMath::Abs(Min[1]) + BrickUnits >= MAX_int32 || FMath::Abs(Min[2]) + BrickUnits >= MAX_int32)
                    continue;
                Block.Min = FIntVector((int32)Min[0], (int32)Min[1], (int32)Min[2]);
                Block.Step = (int32)Step;
                Block.Dim = FIntVector(CellsPerBrick + 1);
                Block.Footprint = HalfFinest * (double)Step;
                Blocks.Add(Block);
                Infos.Add({ Level, Block.Footprint });
            }
        }

        // CPU reference, from the same lattice indices.
        const double CpuStart = FPlatformTime::Seconds();
        TArray<float> Cpu;
        for (const FVoxelDensityGPUBlock& Block : Blocks)
        {
            TArray<double> X, Y, Z;
            X.Reserve(Block.NumPoints());
            Y.Reserve(Block.NumPoints());
            Z.Reserve(Block.NumPoints());
            for (int32 k = 0; k < Block.Dim.Z; k++)
                for (int32 j = 0; j < Block.Dim.Y; j++)
                    for (int32 i = 0; i < Block.Dim.X; i++)
                    {
                        X.Add((double)(Block.Min.X + i * Block.Step) * HalfFinest);
                        Y.Add((double)(Block.Min.Y + j * Block.Step) * HalfFinest);
                        Z.Add((double)(Block.Min.Z + k * Block.Step) * HalfFinest);
                    }
            const int32 First = Cpu.AddUninitialized(Block.NumPoints());
            if (Output == EVoxelDensityGPUOutput::Radial)
            {
                // |p| - R in double: the GPU's double-float position path alone.
                for (int32 i = 0; i < X.Num(); i++)
                    Cpu[First + i] = (float)(FVector(X[i], Y[i], Z[i]).Size() - Density->GetPlanetRadius());
            }
            else if (Output == EVoxelDensityGPUOutput::Edits)
            {
                for (int32 i = 0; i < X.Num(); i++)
                    Cpu[First + i] = EditStore.IsValid() ? (float)EditStore->Sample(FVector(X[i], Y[i], Z[i])) : 0.0f;
            }
            else
            {
                Density->Sample(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), X.Num(), Block.Footprint), Cpu.GetData() + First);
            }
        }
        const double CpuMs = (FPlatformTime::Seconds() - CpuStart) * 1000.0;

        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelDensityGPU] %s: %d blocks, %d samples, %d edit nodes; CPU %.1f ms. Waiting for the GPU..."),
            Output == EVoxelDensityGPUOutput::Radial ? TEXT("|p| - R only") : (Output == EVoxelDensityGPUOutput::Edits ? TEXT("edit deltas only") : TEXT("density")),
            Blocks.Num(), Cpu.Num(), EditData->Flags.Num(), CpuMs);
        const double GpuStart = FPlatformTime::Seconds();
        VoxelDensityGPU::SampleBlocksAsync(Data, HalfFinest, Blocks,
            [Blocks, Infos, Cpu = MoveTemp(Cpu), GpuStart](TArray<float>&& Gpu)
            {
                if (Gpu.Num() != Cpu.Num())
                {
                    UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelDensityGPU] GPU returned %d samples, expected %d."), Gpu.Num(), Cpu.Num());
                    return;
                }
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelDensityGPU] GPU result after %.0f ms (dispatch to readback)."), (FPlatformTime::Seconds() - GpuStart) * 1000.0);

                int32 First = 0;
                for (int32 b = 0; b < Blocks.Num();)
                {
                    // One log line per level (its 8 blocks).
                    const int32 Level = Infos[b].Level;
                    const double Cell = Infos[b].CellSize;
                    double MaxDiff = 0.0, MaxNear = 0.0, SumSq = 0.0;
                    int32 Count = 0, Near = 0, SignFlips = 0, NonFinite = 0, NonZero = 0;
                    for (; b < Blocks.Num() && Infos[b].Level == Level; b++)
                    {
                        for (int32 i = 0; i < Blocks[b].NumPoints(); i++, First++)
                        {
                            const float C = Cpu[First];
                            const float G = Gpu[First];
                            if (!FMath::IsFinite(G))
                            {
                                NonFinite++;
                                continue;
                            }
                            const double Diff = FMath::Abs((double)G - (double)C);
                            MaxDiff = FMath::Max(MaxDiff, Diff);
                            SumSq += Diff * Diff;
                            Count++;
                            SignFlips += (G < 0.0f) != (C < 0.0f) ? 1 : 0;
                            NonZero += C != 0.0f ? 1 : 0;
                            if (FMath::Abs(C) < Cell)
                            {
                                Near++;
                                MaxNear = FMath::Max(MaxNear, Diff);
                            }
                        }
                    }
                    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelDensityGPU] level %2d (cell %.1f cm): max |gpu-cpu| %.4g cm, rms %.3g; within a cell of the surface %d (max %.4g cm); sign flips %d; non-finite %d; CPU non-zero %d"),
                        Level, Cell, MaxDiff, Count > 0 ? FMath::Sqrt(SumSq / Count) : 0.0, Near, MaxNear, SignFlips, NonFinite, NonZero);
                }
            }, Output, EditData);
    }

    FAutoConsoleCommandWithWorldAndArgs Command(
        TEXT("voxel.Debug.DensityGPUCheck"),
        TEXT("Samples the density on the GPU and the CPU around the surface below the camera and logs the differences. Args: number of levels (default 8); 'radial' compares |p| - R only (no noise); 'edits' compares the edit deltas only."),
        FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&Run));
}
