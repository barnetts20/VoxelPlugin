// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "Core/RealtimeMeshInterface.h"
#include "Ext/RealtimeMeshProvider.h"

namespace RealtimeMesh
{
	class IRealtimeMeshFactory_v0
	{
	public:
		static FName GetModularFeatureName()
		{
			static FName FeatureName = TEXT("IRealtimeMeshFactory_v0");
			return FeatureName;
		}
		
	};

	class IRealtimeMeshFactoryInterface_v0 : public IModularFeature
	{
	public:
		virtual ~IRealtimeMeshFactoryInterface_v0() = default;

		static FName GetModularFeatureName()
		{
			static FName FeatureName = TEXT("IRealtimeMeshFactoryInterface_v0");
			return FeatureName;
		}

	
		virtual TSharedRef<IRealtimeMeshFactory_v0> CreateFactory(const TSharedRef<FRealtimeMeshFactoryProvider>& InProvider) const = 0;
	};


}