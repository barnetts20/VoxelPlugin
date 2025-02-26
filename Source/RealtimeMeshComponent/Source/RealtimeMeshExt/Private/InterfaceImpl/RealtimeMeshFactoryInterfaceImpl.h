// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "Ext/RealtimeMeshFactoryInterface.h"
#include "Factory/RealtimeMeshFactory.h"

namespace RealtimeMesh
{
	class FRealtimeMeshFactoryImpl_v0 : public IRealtimeMeshFactory_v0
	{
		TSharedRef<FRealtimeMeshFactory> Factory;
	public:
		FRealtimeMeshFactoryImpl_v0(const TSharedRef<FRealtimeMeshFactory>& InFactory);

		TSharedRef<FRealtimeMeshFactory> GetFactory() const { return Factory; }
	};
	
	struct FRealtimeMeshFactoryInterfaceImpl_v0 : public IRealtimeMeshFactoryInterface_v0
	{
	private:
		mutable TMap<TSharedPtr<FRealtimeMeshFactory>, TWeakPtr<FRealtimeMeshFactoryImpl_v0>> FactoryMap;

		void CleanupDanglingReferences() const;
	public:
		virtual TSharedRef<IRealtimeMeshFactory_v0> CreateFactory(const TSharedRef<FRealtimeMeshFactoryProvider>& InProvider) const override;
	};
}
