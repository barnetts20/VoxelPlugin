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
	FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinMaxDepth[2]);
	// Child Constructor
	FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNodeMT> InParent, uint8 InChildIndex);

	FSamplePosition Center;
	double Extent;

	bool IsLeaf() const;
	bool IsRoot() const;
	bool IsSurface() const;
	bool IsSurfaceLeaf() const;

	//Return the set of surface+leaf nodes who are the InNode or its descendants
	static TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> GetSurfaceLeafNodes(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);

	//Sample a surface leaf node at a given position if one exists, used in neighbor population
	TSharedPtr<FAdaptiveOctreeNodeMT> SampleSurfaceLeafByPosition(FVector SamplePosition);

	//Split the passed node until it is at least the passed depth
	static void SplitToDepth(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, int InDepth);
	
	//Update LOD for every surface/leaf node who is the InNode or its descendant - if lod changes mark the node for mesh regeneration
	static bool UpdateLOD(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FCameraInfo InCameraData, double InLODFactor);
	
	//After LOD update stage, update neighbor information for each surface/leaf node - if neighbors change mark the node for mesh regeneration
	static void UpdateNeighborData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode);
	
	//Update all surface/leaf node mesh data that needs an update who are the passed in node or its children and are marked for regeneration. 
	//Populate a composite mesh data buffer to return the composite of all visited suface/leaf node mesh data, including any updated mesh data
	static void RegenerateMeshData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FInternalMeshBuffer& OutMeshBuffer);

private:
	TArray<uint8> TreeIndex; //Path to the node in the tree
	TFunction<double(FVector)> DensityFunction;

	//State data
	bool bIsLeaf = true;
	bool bIsRoot = true;
	bool bIsSurface = false;
	bool bNeedsMeshUpdate = true;
	int MinMaxDepth[2];

	//Structure Data
	FSamplePosition Corners[8];
	FVoxelEdge Edges[12];
	FQuadFace Faces[6];
	FTetrahedron Tetra[6];

	//Family & neighbor data
	TWeakPtr<FAdaptiveOctreeNodeMT> Parent;
	TSharedPtr<FAdaptiveOctreeNodeMT> Children[8];
	TMap<int, TArray<TWeakPtr<FAdaptiveOctreeNodeMT>>> FaceNeighborMap;

	//LOD data
	FCameraInfo CameraData;
	double LODFactor = 10;
	
	//Mesh Data
	FInternalMeshBuffer MeshData;
	
	//Conditional for valid split case
	bool ShouldSplit();
	
	//Split the node
	void Split();
	
	//Conditional for valid merge case
	bool ShouldMerge();
	
	//Merge the node
	void Merge();

	//Helper method to compute the normals for all sample points, must be invoked after densities are set and before edges are generated
	void ComputeSampleNormals();

	//Update the face id/neighbor pointer map for the individual node
	void UpdateNeighbors();

	//Update the mesh data for the individual node
	FInternalMeshBuffer ComputeGeometry();
};
