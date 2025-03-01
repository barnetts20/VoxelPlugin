// Fill out your copyright notice in the Description page of Project Settings.


#include "FAdaptiveOctreeMT.h"

FAdaptiveOctreeMT::FAdaptiveOctreeMT(TFunction<double(FVector)> InDensityFunction, FVector InCenterPosition, double InExtent, int InChunkMinMaxDepths[3])
{
	Root = MakeShared<FAdaptiveOctreeNodeMT>(InDensityFunction, InCenterPosition, InExtent, InChunkMinMaxDepths);
	FAdaptiveOctreeNodeMT::SplitToDepth(Root, InChunkMinMaxDepths[0]);
	ChunkNodes = FAdaptiveOctreeNodeMT::GetSurfaceLeafNodes(Root);
}

FAdaptiveOctreeMT::~FAdaptiveOctreeMT()
{
}

void FAdaptiveOctreeMT::InitializeChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial)
{
    for (TSharedPtr<FAdaptiveOctreeNodeMT> Chunk : ChunkNodes) {
        TSharedPtr<FMeshChunkMT> NewChunk = MakeShared<FMeshChunkMT>();
        NewChunk->Initialize(InParentActor, InMaterial, Chunk->Center.Position, Chunk->Extent);
        MeshChunks.Add(NewChunk);
    }
    MeshChunksInitialized = true;
}

bool FAdaptiveOctreeMT::UpdateLOD(FCameraInfo InCameraData, double InLODFactor)
{
    bool UpdateOccurred = false;
    if (!MeshChunksInitialized) return UpdateOccurred;
    
    ParallelFor(ChunkNodes.Num(), [&](int32 idx) {
        if (FAdaptiveOctreeNodeMT::UpdateLOD(ChunkNodes[idx], InCameraData, InLODFactor)) {
            FInternalMeshBuffer ChunkBuffer;
            FAdaptiveOctreeNodeMT::RegenerateMeshData(ChunkNodes[idx], ChunkBuffer);
            MeshChunks[idx]->UpdateMeshData(ChunkBuffer);
            MeshChunks[idx]->UpdateComponent();
            UpdateOccurred = true;
        }
    });
    return UpdateOccurred;
}
