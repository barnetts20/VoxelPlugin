// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FVoxelStructure.h>

/**
 * 
 */
class VOXELPLUGIN_API FAdaptiveOctreeNodeMT
{
public:
	FAdaptiveOctreeNodeMT();
	~FAdaptiveOctreeNodeMT();

	bool IsLeaf();
	bool IsSurface();

	bool UpdateLOD(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FVector InCameraPosition, FVector InCameraDirection, double InLODFactor, double InFOV);
	void UpdateNeighborData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);
	FInternalMeshBuffer RegenerateMeshData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);

private:
	bool bIsLeaf;
	bool bIsSurface;

	FVector CameraPosition;
	FVector CameraDirection;
	double LODFactor;
	double FOV;

	FSamplePosition Corners[8];
	FEdge Edges[12];
	FQuadFace Faces[6];
	FTetrahedron Tetra[6];

	TWeakPtr<FAdaptiveOctreeNodeMT> Parent;
	TSharedPtr<FAdaptiveOctreeNodeMT> Children[8];

	FInternalMeshBuffer MeshData;

	void ShouldSplit();
	void Split();
	void ShouldMerge();
	void Merge();
};
