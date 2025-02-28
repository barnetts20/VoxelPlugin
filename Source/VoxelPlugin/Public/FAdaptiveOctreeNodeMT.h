// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FVoxelStructure.h>

/**
 * 
 */
class VOXELPLUGIN_API FAdaptiveOctreeNodeMT : public TSharedFromThis<FAdaptiveOctreeNodeMT>
{
public:
	// Root Constructor
	FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth);
	// Child Constructor
	FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNodeMT> InParent, uint8 InChildIndex);

	FVector Center;
	double Extent;

	bool IsLeaf() const;
	bool IsRoot() const;
	bool IsSurface() const;

	//Return the set of surface+leaf nodes who are the InNode or its descendants
	static TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> GetSurfaceNodes(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);

	//Update LOD for every surface/leaf node who is the InNode or its descendant - if lod changes mark the node for mesh regeneration
	static bool UpdateLOD(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FCameraInfo InCameraData);
	//After LOD update stage, update neighbor information for each surface/leaf node - if neighbors change mark the node for mesh regeneration
	static void UpdateNeighborData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);
	
	//Update all surface/leaf node mesh data that needs an update who are the passed in node or its children and are marked for regeneration. 
	//Populate a composite mesh data buffer to return the composite of all visited suface/leaf node mesh data, including any updated mesh data
	static void RegenerateMeshData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FInternalMeshBuffer& OutMeshBuffer);

private:
	bool bIsLeaf = true;
	bool bIsRoot = true;
	bool bIsSurface = false;
	bool bNeedsMeshUpdate = true;

	TArray<uint8> TreeIndex; //Path to the node in the tree
	TFunction<double(FVector)> DensityFunction;

	FCameraInfo CameraData;
	double LODFactor;

	FSamplePosition Corners[8];
	FEdge Edges[12];
	FQuadFace Faces[6];
	FTetrahedron Tetra[6];

	TWeakPtr<FAdaptiveOctreeNodeMT> Parent;
	TSharedPtr<FAdaptiveOctreeNodeMT> Children[8];
	TArray<TWeakPtr<FAdaptiveOctreeNodeMT>> Neighbors;

	int MinMaxDepth[2];

	FInternalMeshBuffer MeshData;

	void ShouldSplit();
	void Split();
	void ShouldMerge();
	void Merge();

	FInternalMeshBuffer ComputeGeometry(); //Populate and return node specific FInternalMeshBuffer
};
