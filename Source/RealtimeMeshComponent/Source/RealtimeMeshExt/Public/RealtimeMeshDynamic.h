/*// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "Core/RealtimeMeshInterfaceFwd.h"
#include "UDynamicMesh.h"
#include "RealtimeMesh.h"
#include "Data/RealtimeMeshUpdateBuilder.h"
#include "RealtimeMeshDynamic.generated.h"


enum class ERealtimeMeshDynamicMaterialHandling
{
	MapToFirstGroup,
	MapExtraGroup,
	DropUnknownGroups,
};

namespace RealtimeMesh
{
	class REALTIMEMESHEXT_API FRealtimeMeshSectionGroupDynamic : public FRealtimeMeshSectionGroup, public FGCObject
	{
		TObjectPtr<UDynamicMesh> Mesh;
		FDynamicMesh3 RenderMesh;
		ERealtimeMeshDynamicMaterialHandling MaterialHandling;
		mutable TOptional<FBoxSphereBounds3f> CachedBounds;
		uint32 bOptimizedTranslate : 1;

	public:
		FRealtimeMeshSectionGroupDynamic(const FRealtimeMeshSharedResourcesRef& InSharedResources, const FRealtimeMeshSectionGroupKey& InKey)
			: FRealtimeMeshSectionGroup(InSharedResources, InKey)
			, MaterialHandling(ERealtimeMeshDynamicMaterialHandling::MapToFirstGroup)
			, bOptimizedTranslate(true)
		{
		}


		virtual void InitializeProxy(FRealtimeMeshUpdateContext& UpdateContext) override;
		//virtual TOptional<FBoxSphereBounds3f> CalculateBounds(const FRealtimeMeshLockContext& LockContext) const override;
		virtual bool Serialize(FArchive& Ar) override;
		
		void SetMesh(FRealtimeMeshUpdateContext& UpdateContext, UDynamicMesh* MeshData);
		

	protected:
		virtual FString GetReferencerName() const override { return SharedResources->GetMeshName().ToString(); }
		virtual void AddReferencedObjects(FReferenceCollector& Collector) override
		{
			Collector.AddReferencedObject(Mesh);
		}

		bool GenerateUpdate(FRealtimeMeshUpdateContext& UpdateContext, const FDynamicMesh3& InMesh);
	};


	class REALTIMEMESHEXT_API FRealtimeMeshSharedResourcesDynamic : public FRealtimeMeshSharedResources
	{
	public:

		virtual FRealtimeMeshSectionGroupRef CreateSectionGroup(const FRealtimeMeshSectionGroupKey& InKey) const override
		{
			return MakeShared<FRealtimeMeshSectionGroupDynamic>(ConstCastSharedRef<FRealtimeMeshSharedResources>(this->AsShared()), InKey);
		}

		virtual FRealtimeMeshRef CreateRealtimeMesh() const override;
		virtual FRealtimeMeshSharedResourcesRef CreateSharedResources() const override { return MakeShared<FRealtimeMeshSharedResourcesDynamic>(); }
	};
	
	struct REALTIMEMESHEXT_API FRealtimeMeshDynamic : public FRealtimeMesh
	{

		FRealtimeMeshDynamic(const FRealtimeMeshSharedResourcesRef& InSharedResources)
			: FRealtimeMesh(InSharedResources)
		{
		}
		
	};
}

// ReSharper disable UnrealHeaderToolError

DECLARE_DYNAMIC_DELEGATE_OneParam(FRealtimeMeshDynamicCompletionCallback, ERealtimeMeshProxyUpdateStatus, ProxyUpdateResult);

DECLARE_DYNAMIC_DELEGATE_OneParam(FRealtimeMeshDynamicCollisionCompletionCallback, ERealtimeMeshCollisionUpdateResult, CollisionResult);


UCLASS()
class REALTIMEMESHEXT_API URealtimeMeshDynamic : public URealtimeMesh
{
	GENERATED_UCLASS_BODY()
	
	TSharedRef<RealtimeMesh::FRealtimeMeshDynamic> GetMeshData() const { return StaticCastSharedRef<RealtimeMesh::FRealtimeMeshDynamic>(GetMesh()); }

	TFuture<ERealtimeMeshProxyUpdateStatus> CreateSectionGroup(const FRealtimeMeshSectionGroupKey& SectionGroupKey, UDynamicMesh* MeshData = nullptr);

	
	//UFUNCTION(BlueprintCallable, Category = "Components|RealtimeMesh", DisplayName="CreateSectionGroup", meta = (AutoCreateRefTerm = "SectionGroupKey"))
	//void CreateSectionGroup(const FRealtimeMeshSectionGroupKey& SectionGroupKey, UDynamicMesh* MeshData, const FRealtimeMeshDynamicCompletionCallback& CompletionCallback);
};

// ReSharper restore UnrealHeaderToolError*/