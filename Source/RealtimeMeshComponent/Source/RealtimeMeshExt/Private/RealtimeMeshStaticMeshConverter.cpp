// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshStaticMeshConverter.h"

#include "MaterialDomain.h"
#include "RealtimeMeshComponentModule.h"
#include "RealtimeMeshSimple.h"
#include "StaticMeshLODResourcesAdapter.h"
#include "UDynamicMesh.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "Engine/StaticMesh.h"
#include "Mesh/RealtimeMeshBlueprintMeshBuilder.h"
#include "Core/RealtimeMeshBuilder.h"
#include "RenderProxy/RealtimeMeshNaniteProxyInterface.h"

using namespace UE::Geometry;
using namespace RealtimeMesh;

bool URealtimeMeshStaticMeshConverter::CopyStreamSetToStaticMesh(const FRealtimeMeshStreamSet& InStreamSet, UStaticMesh* OutStaticMesh,
	const FStreamSetStaticMeshConversionOptions& Options)
{
	return true;
}

bool URealtimeMeshStaticMeshConverter::CopyStreamSetFromStaticMesh(const UStaticMesh* InStaticMesh, FRealtimeMeshStreamSet& OutStreamSet,
	const FStreamSetStaticMeshConversionOptions& Options)
{
	OutStreamSet = FRealtimeMeshStreamSet();
	if (InStaticMesh == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: InStaticMesh is null"));
		return false;
	}

/*#if WITH_EDITOR
	if (Options.LODType != ERealtimeMeshCopyStaticMeshLODType::RenderData)
	{
		return CopyStreamSetToStaticMesh_SourceData(InStaticMesh, OutStreamSet, Options);
	}
#endif*/

	return CopyStreamSetToStaticMesh_RenderData(InStaticMesh, OutStreamSet, Options);
}


URealtimeMeshStreamSet* URealtimeMeshStaticMeshConverter::CopyStreamSetFromStaticMesh(UStaticMesh* FromStaticMeshAsset, URealtimeMeshStreamSet* ToStreamSet,
	FStreamSetStaticMeshConversionOptions Options, ERealtimeMeshOutcomePins& Outcome)
{
	if (FromStaticMeshAsset == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: FromStaticMeshAsset is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToStreamSet;
	}

	if (ToStreamSet == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: ToStreamSet is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToStreamSet;
	}

	const bool bSuccess = CopyStreamSetFromStaticMesh(FromStaticMeshAsset, ToStreamSet->GetStreamSet(), Options);

	Outcome = bSuccess? ERealtimeMeshOutcomePins::Success : ERealtimeMeshOutcomePins::Failure;
	return ToStreamSet;
}

UStaticMesh* URealtimeMeshStaticMeshConverter::CopyStreamSetToStaticMesh(URealtimeMeshStreamSet* FromStreamSet, UStaticMesh* ToStaticMeshAsset,
	FStreamSetStaticMeshConversionOptions Options, ERealtimeMeshOutcomePins& Outcome)
{
	if (FromStreamSet == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyToStaticMesh failed: FromStreamSet is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToStaticMeshAsset;
	}

	if (ToStaticMeshAsset == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyToStaticMesh failed: ToStaticMeshAsset is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToStaticMeshAsset;
	}
	
	const bool bSuccess = CopyStreamSetToStaticMesh(FromStreamSet->GetStreamSet(), ToStaticMeshAsset, Options);

	Outcome = bSuccess? ERealtimeMeshOutcomePins::Success : ERealtimeMeshOutcomePins::Failure;
	return ToStaticMeshAsset;
}

URealtimeMeshSimple* URealtimeMeshStaticMeshConverter::CopyRealtimeMeshFromStaticMesh(UStaticMesh* FromStaticMeshAsset, URealtimeMeshSimple* ToRealtimeMesh,
	FRealtimeMeshStaticMeshConversionOptions Options, ERealtimeMeshOutcomePins& Outcome)
{
	if (FromStaticMeshAsset == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: FromStaticMeshAsset is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToRealtimeMesh;
	}

	if (ToRealtimeMesh == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: ToRealtimeMesh is null"));
		Outcome = ERealtimeMeshOutcomePins::Failure;
		return ToRealtimeMesh;
	}

	// Grab the materials
	if (Options.bWantsMaterials)
	{
		const auto& Materials = FromStaticMeshAsset->GetStaticMaterials();
		for (int32 MatID = 0; MatID < Materials.Num(); MatID++)
		{
			ToRealtimeMesh->SetupMaterialSlot(MatID, Materials[MatID].MaterialSlotName, Materials[MatID].MaterialInterface);
		}		
	}

	// Grab all the LODs
	const int32 MinLOD = FMath::Clamp(Options.MinLODIndex, 0, FromStaticMeshAsset->GetNumLODs() - 1);
	const int32 MaxLOD = FMath::Clamp(Options.MaxLODIndex, 0, FromStaticMeshAsset->GetNumLODs() - 1);
	
	for (int32 LODIndex = MinLOD; LODIndex <= MaxLOD; LODIndex++)
	{
		FStreamSetStaticMeshConversionOptions SectionOptions;
		//SectionOptions.LODType = Options.LODType;
		SectionOptions.LODIndex = LODIndex;
		SectionOptions.bWantTangents = Options.bWantTangents;
		SectionOptions.bWantUVs = Options.bWantUVs;
		SectionOptions.bWantVertexColors = Options.bWantVertexColors;
		SectionOptions.bWantPolyGroups = Options.bWantPolyGroups;
		
		FRealtimeMeshStreamSet Streams;

		if (!CopyStreamSetFromStaticMesh(FromStaticMeshAsset, Streams, SectionOptions))
		{
			UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: Failed to copy LOD %d"), LODIndex);
			ToRealtimeMesh->Reset();
			return nullptr;
		}

		const FRealtimeMeshLODKey LODKey = LODIndex > MinLOD? ToRealtimeMesh->AddLOD(FRealtimeMeshLODConfig()) : FRealtimeMeshLODKey(0);
		const FRealtimeMeshSectionGroupKey SectionGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, "Default");

		if (FromStaticMeshAsset->GetRenderData())
		{
			ToRealtimeMesh->UpdateLODConfig(LODKey, FRealtimeMeshLODConfig(FromStaticMeshAsset->GetRenderData()->ScreenSize[LODIndex].GetValue()));
		}
		ToRealtimeMesh->CreateSectionGroup(SectionGroupKey, MoveTemp(Streams));
	}

	// Copy distance field if requested
	if (Options.bWantsDistanceField && FromStaticMeshAsset->GetRenderData() && FromStaticMeshAsset->GetRenderData()->LODResources.IsValidIndex(0))
	{
		// Grab a copy of the data
		FRealtimeMeshDistanceField DistancField(*FromStaticMeshAsset->GetRenderData()->LODResources[0].DistanceFieldData);
		ToRealtimeMesh->SetDistanceField(MoveTemp(DistancField));
	}

	// Copy card representation if requested
	if (Options.bWantsLumenCards && FromStaticMeshAsset->GetRenderData() && FromStaticMeshAsset->GetRenderData()->LODResources.IsValidIndex(0))
	{
		// Grab a copy of the data
		if (FromStaticMeshAsset->GetRenderData()->LODResources[0].CardRepresentationData)
		{
			FRealtimeMeshCardRepresentation CardRepresentation(*FromStaticMeshAsset->GetRenderData()->LODResources[0].CardRepresentationData);
			ToRealtimeMesh->SetCardRepresentation(MoveTemp(CardRepresentation));			
		}
	}

	// Copies the nanite resources. This relies on functionality only available in RMC-Pro
	/*if (IRealtimeMeshNaniteSceneProxyManager::IsNaniteSupportAvailable())
	{
		if (FromStaticMeshAsset->HasValidNaniteData())
		{
			auto NewResources= IRealtimeMeshNaniteSceneProxyManager::GetNaniteModule().CreateNewResources(*FromStaticMeshAsset->GetRenderData()->NaniteResourcesPtr);
			ToRealtimeMesh->GetMesh()->SetNaniteResources(NewResources);			
		}		
	}*/
	
	Outcome = ERealtimeMeshOutcomePins::Success;
	return ToRealtimeMesh;
}

/*UStaticMesh* URealtimeMeshStaticMeshConverter::CopyRealtimeMeshToStaticMesh(URealtimeMeshSimple* FromRealtimeMesh, UStaticMesh* ToStaticMeshAsset,
	FRealtimeMeshStaticMeshConversionOptions Options, ERealtimeMeshOutcomePins& Outcome)
{
	check(false);
	Outcome = ERealtimeMeshOutcomePins::Success;
	return ToStaticMeshAsset;
}*/




bool URealtimeMeshStaticMeshConverter::CopyStreamSetToStaticMesh_RenderData(const UStaticMesh* InStaticMesh, FRealtimeMeshStreamSet& OutStreamSet,
                                                                            const FStreamSetStaticMeshConversionOptions& Options)
{
	OutStreamSet = FRealtimeMeshStreamSet();
#if RMC_ENGINE_ABOVE_5_1
	/*if (Options.LODType != ERealtimeMeshCopyStaticMeshLODType::MaxAvailable && Options.LODType != ERealtimeMeshCopyStaticMeshLODType::RenderData)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyStreamSetToStaticMesh failed: Requested LOD Type is not available"));
		return false;
	}*/

	// TODO: I've heard this is also unusable in a dedicated server, so need to look into that and make sure we handle that failure correctly
#if !WITH_EDITOR
	if (InStaticMesh->bAllowCPUAccess == false)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: StaticMesh bAllowCPUAccess must be set to true to read mesh data at Runtime"));
		return false;
	}
#endif

	const int32 UseLODIndex = FMath::Clamp(Options.LODIndex, 0, InStaticMesh->GetNumLODs() - 1);

	const FStaticMeshLODResources* LODResources = nullptr;
	if (const FStaticMeshRenderData* RenderData = InStaticMesh->GetRenderData())
	{
		LODResources = &RenderData->LODResources[UseLODIndex];
	}
	if (LODResources == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyFromStaticMesh failed: Request LOD is not available"));
		return false;
	}
	
#if WITH_EDITOR
	// respect BuildScale build setting
	const FMeshBuildSettings& LODBuildSettings = InStaticMesh->GetSourceModel(UseLODIndex).BuildSettings;
	const FVector3d BuildScale = LODBuildSettings.BuildScale3D;
#else
	const FVector3d BuildScale = FVector3d::One();	
#endif

	TRealtimeMeshStreamBuilder<FVector3f> PositionData(OutStreamSet.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>()));
	TRealtimeMeshStreamBuilder<TIndex3<uint32>> TriangleData(OutStreamSet.AddStream(FRealtimeMeshStreams::Triangles,
		GetRealtimeMeshBufferLayout<TIndex3<uint32>>()));

	FStaticMeshLODResourcesMeshAdapter Adapter(LODResources);
	Adapter.SetBuildScale(BuildScale, false);

	// Copy vertices. LODMesh is dense so this should be 1-1
	const int32 VertexCount = Adapter.VertexCount();
	PositionData.SetNumUninitialized(VertexCount);
	for (int32 VertID = 0; VertID < VertexCount; VertID++)
	{
		const FVector3f Position = static_cast<FVector3f>(Adapter.GetVertex(VertID));
		PositionData.Set(VertID, Position);
	}

	// Copy triangles. LODMesh is dense so this should be 1-1 unless there is a duplicate tri or non-manifold edge (currently aborting in that case)
	const int32 TriangleCount = Adapter.TriangleCount();
	TriangleData.SetNumUninitialized(TriangleCount);
	for (int32 TriID = 0; TriID < TriangleCount; TriID++)
	{
		const FIndex3i Tri = Adapter.GetTriangle(TriID);
		TriangleData.Set(TriID, TIndex3<uint32>(Tri.A, Tri.B, Tri.C));
	}
	
	// transfer sections to PolyGroups
	if (Options.bWantPolyGroups)
	{
		TRealtimeMeshStreamBuilder<uint16> PolyGroupData(OutStreamSet.AddStream(FRealtimeMeshStreams::PolyGroups,
		GetRealtimeMeshBufferLayout<uint16>()));
		PolyGroupData.SetNumUninitialized(TriangleCount);

		for (int32 SectionIdx = 0; SectionIdx < LODResources->Sections.Num(); SectionIdx++)
		{
			const FStaticMeshSection& Section = LODResources->Sections[SectionIdx];
			for (uint32 TriIdx = 0; TriIdx < Section.NumTriangles; TriIdx++)
			{
				const uint32 TriangleID = Section.FirstIndex / 3 + TriIdx;
				PolyGroupData.Set(TriangleID, SectionIdx);
			}
		}
	}

	// copy tangents
	if (Adapter.HasNormals() && Options.bWantTangents)
	{
		TRealtimeMeshStreamBuilder<TRealtimeMeshTangents<FVector4f>, TRealtimeMeshTangents<FPackedNormal>> TangentData(OutStreamSet.AddStream(FRealtimeMeshStreams::Tangents,
		GetRealtimeMeshBufferLayout<TRealtimeMeshTangents<FPackedNormal>>()));
		TangentData.SetNumUninitialized(Adapter.VertexCount());
		
		for (int32 VertID = 0; VertID < VertexCount; VertID++)
		{
			const FVector3f N = Adapter.GetNormal(VertID);
			const FVector3f T = Adapter.GetTangentX(VertID);
			const FVector3f B = Adapter.GetTangentY(VertID);			
			TangentData.Set(VertID, TRealtimeMeshTangents<FVector4f>(N, B, T));
		}
	}

	// copy UV layers
	if (Adapter.HasUVs() && Options.bWantUVs && Adapter.NumUVLayers() > 0)
	{
		const int32 NumUVLayers = Adapter.NumUVLayers();

		FRealtimeMeshStream& TexCoordStream = OutStreamSet.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2f>(NumUVLayers));
		TexCoordStream.SetNumUninitialized(Adapter.VertexCount());
		
		for (int32 UVLayerIndex = 0; UVLayerIndex < NumUVLayers; UVLayerIndex++)
		{
			TRealtimeMeshStridedStreamBuilder<FVector2f> TexCoordData(TexCoordStream, UVLayerIndex);
			for (int32 VertID = 0; VertID < VertexCount; VertID++)
			{
				const FVector2f UV = Adapter.GetUV(VertID, UVLayerIndex);
				TexCoordData.Set(VertID, UV);
			}
		}
	}

	// copy colors
	if ( Adapter.HasColors() && Options.bWantVertexColors )
	{
		TRealtimeMeshStreamBuilder<FColor> ColorData(OutStreamSet.AddStream(FRealtimeMeshStreams::Color,
		GetRealtimeMeshBufferLayout<FColor>()));
		ColorData.SetNumUninitialized(Adapter.VertexCount());
		for (int32 VertID = 0; VertID < VertexCount; VertID++)
		{
			const FColor Color = Adapter.GetColor(VertID);
			ColorData.Set(VertID, Color);
		}
	}

	return true;
#else
	return false;
#endif
}

/*bool URealtimeMeshConversion::CopyStreamSetToStaticMesh_SourceData(const UStaticMesh* InStaticMesh, RealtimeMesh::FRealtimeMeshStreamSet& OutStreamSet,
	const FStreamSetStaticMeshConversionOptions& Options)
{
	OutStreamSet = FRealtimeMeshStreamSet();
	if (Options.LODType != ERealtimeMeshCopyStaticMeshLODType::MaxAvailable && Options.LODType != ERealtimeMeshCopyStaticMeshLODType::SourceModel)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyStreamSetToStaticMesh failed: Requested LOD Type is not available"));
		return false;
	}
	
	const int32 UseLODIndex = FMath::Clamp(Options.LODIndex, 0, InStaticMesh->GetNumSourceModels() - 1);

	const FMeshDescription* SourceMesh = InStaticMesh->GetMeshDescription(UseLODIndex);
	if (SourceMesh == nullptr)
	{
		UE_LOG(LogRealtimeMesh, Warning, TEXT("RealtimeMeshWarning: CopyStreamSetToStaticMesh failed: Request LOD is not available"));
		return false;
	}
	
	const FStaticMeshSourceModel& SourceModel = InStaticMesh->GetSourceModel(UseLODIndex);
	const FMeshBuildSettings& BuildSettings = SourceModel.BuildSettings;

	const bool bHasDirtyBuildSettings = BuildSettings.bRecomputeNormals
		|| (BuildSettings.bRecomputeTangents && Options.bWantTangents);

	FMeshDescription LocalSourceMeshCopy;
	if (Options.bApplyBuildSettings && bHasDirtyBuildSettings )
	{
		LocalSourceMeshCopy = *SourceMesh;

		FStaticMeshAttributes Attributes(LocalSourceMeshCopy);
		if (!Attributes.GetTriangleNormals().IsValid() || !Attributes.GetTriangleTangents().IsValid())
		{
			// If these attributes don't exist, create them and compute their values for each triangle
			FStaticMeshOperations::ComputeTriangleTangentsAndNormals(LocalSourceMeshCopy);
		}

		EComputeNTBsFlags ComputeNTBsOptions = EComputeNTBsFlags::BlendOverlappingNormals;
		ComputeNTBsOptions |= BuildSettings.bRecomputeNormals ? EComputeNTBsFlags::Normals : EComputeNTBsFlags::None;
		if (Options.bWantTangents)
		{
			ComputeNTBsOptions |= BuildSettings.bRecomputeTangents ? EComputeNTBsFlags::Tangents : EComputeNTBsFlags::None;
			ComputeNTBsOptions |= BuildSettings.bUseMikkTSpace ? EComputeNTBsFlags::UseMikkTSpace : EComputeNTBsFlags::None;
		}
		ComputeNTBsOptions |= BuildSettings.bComputeWeightedNormals ? EComputeNTBsFlags::WeightedNTBs : EComputeNTBsFlags::None;
		if (Options.bIgnoreRemoveDegenerates == false)
		{
			ComputeNTBsOptions |= BuildSettings.bRemoveDegenerates ? EComputeNTBsFlags::IgnoreDegenerateTriangles : EComputeNTBsFlags::None;
		}

		FStaticMeshOperations::ComputeTangentsAndNormals(LocalSourceMeshCopy, ComputeNTBsOptions);

		SourceMesh = &LocalSourceMeshCopy;
	}





	
	check(false); // not implemented yet
	
	return false;
}*/
