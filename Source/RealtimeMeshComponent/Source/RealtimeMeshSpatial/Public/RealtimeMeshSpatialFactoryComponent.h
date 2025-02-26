// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "InstancedStruct.h"
#include "RealtimeMeshSpatialComponent.h"
#include "RealtimeMeshSpatialFactoryComponent.generated.h"


namespace RealtimeMesh
{
	class FRealtimeMeshFactory;
	class IRealtimeMeshSpatialStreamingFactoryStructureProvider;
}


UCLASS()
class REALTIMEMESHSPATIAL_API URealtimeMeshSpatialFactoryComponent : public URealtimeMeshSpatialComponent
{
	GENERATED_BODY()

	TSharedPtr<RealtimeMesh::FRealtimeMeshFactory> Factory;
public:
	
	URealtimeMeshSpatialFactoryComponent();
	virtual ~URealtimeMeshSpatialFactoryComponent() override;

	void Initialize(const TSharedRef<RealtimeMesh::IRealtimeMeshSpatialStreamingFactoryStructureProvider>& InChunkProvider, const TSharedRef<RealtimeMesh::FRealtimeMeshFactory>& InFactory);
	using URealtimeMeshSpatialComponent::Reset;


protected:
	virtual TFuture<URealtimeMesh*> LoadCell(const FRealtimeMeshSpatialComponentLocation& Cell, RealtimeMesh::FRealtimeMeshCancellationToken CancellationToken) override;
	virtual TFuture<bool> UpdateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh, RealtimeMesh::FRealtimeMeshCancellationToken CancellationToken) override;
	virtual void UnloadCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh) override;
};


