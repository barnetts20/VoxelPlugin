// Copyright TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "CoreFwd.h"
#include "RealtimeMeshSpatialComponentLocation.h"
#include "RealtimeMeshSpatialStreamingCellState.h"
#include "RealtimeMeshSpatialStreamingSource.h"
#include "Spatial/RealtimeMeshSpatialValidChunkProvider.h"
#include "RealtimeMeshSpatialComponent.generated.h"

namespace RealtimeMesh
{
	class IRealtimeMeshSpatialStreamingStructureProvider;
	class FRealtimeMeshCancellationToken;
}

class URealtimeMeshPool;
class URealtimeMeshComponent;

UCLASS(Abstract)
class REALTIMEMESHSPATIAL_API URealtimeMeshSpatialComponent : public USceneComponent
{
	GENERATED_BODY()
private:
	/*
	 *	All the meshes that are active in this actor
	 */
	UPROPERTY(Transient)
	TMap<FRealtimeMeshSpatialComponentLocation, FRealtimeMeshSpatialStreamingCellState> ActiveCells;

	UPROPERTY(Transient)
	TArray<TObjectPtr<URealtimeMeshComponent>> RecycledMeshes;

	UPROPERTY(EditAnywhere, Category = "Realtime Mesh")
	TArray<UMaterialInterface*> Materials;

	TWeakPtr<RealtimeMesh::IRealtimeMeshSpatialStreamingStructureProvider> ChunkProviderWeak;
	
public:
	// Sets default values for this actor's properties
	URealtimeMeshSpatialComponent();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void OnRegister() override;
	virtual void OnUnregister() override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	
protected:
	TSharedPtr<RealtimeMesh::IRealtimeMeshSpatialStreamingStructureProvider> GetChunkProvider() const { return ChunkProviderWeak.Pin(); }

	void Initialize(const TSharedRef<RealtimeMesh::IRealtimeMeshSpatialStreamingStructureProvider>& InChunkProvider);
	void Reset();	

	void UpdateStreaming(const TArray<FRealtimeMeshSpatialStreamingSource>& Sources);
	void GetWantedStreamingState(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources, const TMap<FRealtimeMeshSpatialComponentLocation, FRealtimeMeshSpatialStreamingCellState>& ExistingCells,
		TSet<FRealtimeMeshSpatialComponentLocation>& CellsToDeActivate, TSet<FRealtimeMeshSpatialComponentLocation>& CellsToUnload, TSet<FRealtimeMeshSpatialComponentLocation>& CellsToActivate, TSet<FRealtimeMeshSpatialComponentLocation>& CellsToLoad);

	virtual void ActivateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMeshComponent* MeshComponent);
	virtual void DeactivateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMeshComponent* MeshComponent);
	virtual TFuture<URealtimeMesh*> LoadCell(const FRealtimeMeshSpatialComponentLocation& Cell, RealtimeMesh::FRealtimeMeshCancellationToken CancellationToken);

	virtual TFuture<bool> UpdateCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh, RealtimeMesh::FRealtimeMeshCancellationToken CancellationToken);
	virtual void UnloadCell(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* Mesh);

	FSphere3d GetStreamingSphereForSourceAndLOD(const FRealtimeMeshSpatialStreamingSource& StreamingSource, int32 LOD);
	TArray<TArray<FSphere3d>> GetStreamingSpheres(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources, float ScaleFactor);
	TSet<FRealtimeMeshSpatialComponentLocation> CalculatedDesiredTree(const TArrayView<const FRealtimeMeshSpatialStreamingSource> StreamingSources, float ScaleFactor, const TFunctionRef<bool(const FRealtimeMeshSpatialComponentLocation&)>& IsReadyFunc);

private:
	void FinalizeCellLoad(const FRealtimeMeshSpatialComponentLocation& Cell, URealtimeMesh* InMesh, bool bIsReload, int32 ChangeId, int32 SerialKey);
	void FinalizeCellUpdate(const FRealtimeMeshSpatialComponentLocation& Cell, int32 ChangeId, bool bHasMesh, int32 SerialKey);
	void RecycleMesh(URealtimeMeshComponent* Component);

	void OnCellChanged(const FRealtimeMeshSpatialComponentLocation& Cell);
	
	virtual FString GetCellName(const FRealtimeMeshSpatialComponentLocation& Cell);
	EObjectFlags GetCellComponentFlags();
	
	friend class URealtimeMeshSpatialStreamingSubsystem;	
};
