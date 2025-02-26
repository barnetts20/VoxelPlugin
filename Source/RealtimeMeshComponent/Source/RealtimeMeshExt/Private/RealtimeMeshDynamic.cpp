/*// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshDynamic.h"

#include "RealtimeMeshDynamicMeshConverter.h"
#include "RealtimeMeshSimple.h"
#include "Data/RealtimeMeshUpdateBuilder.h"
#include "RenderProxy/RealtimeMeshProxyCommandBatch.h"

#define LOCTEXT_NAMESPACE "RealtimeMeshDynamic"

namespace RealtimeMesh
{
	void FRealtimeMeshSectionGroupDynamic::InitializeProxy(FRealtimeMeshUpdateContext& UpdateContext)
	{
		FRealtimeMeshSectionGroup::InitializeProxy(UpdateContext);
		if (Mesh)
		{
			GenerateUpdate(UpdateContext, Mesh->GetMeshRef());
		}
	}

	TOptional<FBoxSphereBounds3f> FRealtimeMeshSectionGroupDynamic::CalculateBounds(const FRealtimeMeshLockContext& LockContext) const
	{
		if (CachedBounds.IsSet())
		{
			return CachedBounds.GetValue();
		}

		if (IsValid(Mesh))
		{
			auto MeshBounds = Mesh->GetMeshRef().GetBounds(true);
			CachedBounds = FBoxSphereBounds3f(FBox3f(FVector3f(MeshBounds.Min), FVector3f(MeshBounds.Max)));
			return CachedBounds.GetValue();
		}
		
		return FBoxSphereBounds3f(FVector3f::Zero(), FVector3f::One(), 1.0f);
	}

	bool FRealtimeMeshSectionGroupDynamic::Serialize(FArchive& Ar)
	{
		return FRealtimeMeshSectionGroup::Serialize(Ar);

		
	}

	void FRealtimeMeshSectionGroupDynamic::SetMesh(FRealtimeMeshUpdateContext& UpdateContext, UDynamicMesh* MeshData)
	{
		FRealtimeMeshScopeGuardWrite ScopeGuard(SharedResources);
		
		Mesh = MeshData;
		CachedBounds.Reset();

		GenerateUpdate(UpdateContext, Mesh->GetMeshRef());
	}

	bool FRealtimeMeshSectionGroupDynamic::GenerateUpdate(FRealtimeMeshUpdateContext& UpdateContext, const FDynamicMesh3& InMesh)
	{
		// TODO: We need to get this data down from the parent mesh
		const int32 NumMaterials = 1;
		

		FRealtimeMeshStreamSet MeshDataStreams;
		FStreamSetDynamicMeshConversionOptions Options;
		Options.bWantNormals = true;
		Options.bWantTangents = true;
		Options.bWantUVs = true;
		Options.bWantVertexColors = true;
		Options.bWantMaterialIDs = true;

		Options.bShouldFastConvert = !bOptimizedTranslate;

		if (MaterialHandling == ERealtimeMeshDynamicMaterialHandling::DropUnknownGroups)
		{
			Options.TriangleFilterFunction = [](const FDynamicMesh3& Mesh, int32 TriangleID) -> bool
			{
				return Mesh.GetTriangleGroup(TriangleID) < 2;
			};
		}

		if (MaterialHandling == ERealtimeMeshDynamicMaterialHandling::MapExtraGroup)
		{
			Options.PolyGroupRemapFunction = [NumMaterials](const FDynamicMesh3& Mesh, int32 TriangleID) -> int32
			{
				const int32 TriangleGroup = Mesh.GetTriangleGroup(TriangleID);

				return TriangleGroup >= 0? FMath::Min<int32>(TriangleGroup, NumMaterials) : NumMaterials;
			};
		}
		else if (MaterialHandling == ERealtimeMeshDynamicMaterialHandling::MapToFirstGroup)
		{
			Options.PolyGroupRemapFunction = [NumMaterials](const FDynamicMesh3& Mesh, int32 TriangleID) -> int32
			{
				const int32 TriangleGroup = Mesh.GetTriangleGroup(TriangleID);

				return TriangleGroup < NumMaterials? FMath::Max<int32>(TriangleGroup, 0) : 0;
			};
		}

		TMap<int32, FRealtimeMeshStreamRange> Ranges;
		if (URealtimeMeshDynamicMeshConverter::CopyStreamSetFromDynamicMesh(InMesh, MeshDataStreams, Options, false, &Ranges))
		{
			SetAllStreams(UpdateContext, MoveTemp(MeshDataStreams));

			// Add/Update current sections
			TSet<FRealtimeMeshSectionKey> CurrentSections;
			for (const auto& Range : Ranges)
			{
				const FRealtimeMeshSectionKey SectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(Key, Range.Key);
				CurrentSections.Add(SectionKey);
				CreateOrUpdateSection(UpdateContext, SectionKey, FRealtimeMeshSectionConfig(Range.Key), Range.Value);
			}

			// Remove old sections
			for (const auto& SectionKey : GetSectionKeys(UpdateContext))
			{
				if (!CurrentSections.Contains(SectionKey))
				{
					RemoveSection(UpdateContext, SectionKey);
				}
			}			

			return true;
		}

		return false;		
	}

	FRealtimeMeshRef FRealtimeMeshSharedResourcesDynamic::CreateRealtimeMesh() const
	{
		return MakeShared<FRealtimeMeshDynamic>(ConstCastSharedRef<FRealtimeMeshSharedResources>(this->AsShared()));
	}
}

URealtimeMeshDynamic::URealtimeMeshDynamic(const FObjectInitializer& ObjectInitializer)
	: URealtimeMesh(ObjectInitializer)
{
	Initialize(MakeShared<RealtimeMesh::FRealtimeMeshSharedResourcesDynamic>());

	FRealtimeMeshUpdateContext UpdateContext(GetMesh());
	MeshRef->InitializeLODs(UpdateContext, RealtimeMesh::TFixedLODArray<FRealtimeMeshLODConfig>{FRealtimeMeshLODConfig()});
}

using namespace RealtimeMesh;

TFuture<ERealtimeMeshProxyUpdateStatus> URealtimeMeshDynamic::CreateSectionGroup(const FRealtimeMeshSectionGroupKey& SectionGroupKey, UDynamicMesh* MeshData)
{
	FRealtimeMeshUpdateContext UpdateContext(SharedResources.ToSharedRef());
	if (const auto LOD = GetMesh()->GetLOD(UpdateContext, SectionGroupKey.LOD()))
	{
		
		LOD->CreateOrUpdateSectionGroup(UpdateContext, SectionGroupKey, FRealtimeMeshSectionGroupConfig());
		const auto SectionGroup = LOD->GetSectionGroupAs<FRealtimeMeshSectionGroupDynamic>(UpdateContext, SectionGroupKey);
		SectionGroup->SetMesh(UpdateContext, MeshData);
		return UpdateContext.Commit();
	}
	else
	{
		FMessageLog("RealtimeMesh").Error(
			FText::Format(LOCTEXT("CreateSectionGroup_InvalidLODKey", "CreateSectionGroup: Invalid LODKey key {0}"),
						  FText::FromString(SectionGroupKey.LOD().ToString())));
		return MakeFulfilledPromise<ERealtimeMeshProxyUpdateStatus>(ERealtimeMeshProxyUpdateStatus::NoUpdate).GetFuture();
	}
}

void URealtimeMeshDynamic::CreateSectionGroup(const FRealtimeMeshSectionGroupKey& SectionGroupKey, UDynamicMesh* MeshData,
                                              const FRealtimeMeshDynamicCompletionCallback& CompletionCallback)
{
	TFuture<ERealtimeMeshProxyUpdateStatus> Continuation = IsValid(MeshData)? CreateSectionGroup(SectionGroupKey, MeshData) : CreateSectionGroup(SectionGroupKey);	
	Continuation.Next([CompletionCallback](ERealtimeMeshProxyUpdateStatus Status)
	{
		if (CompletionCallback.IsBound())
		{
			CompletionCallback.Execute(Status);
		}
	});
}

#undef LOCTEXT_NAMESPACE*/