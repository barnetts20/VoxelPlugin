#pragma once
#include "CoreMinimal.h"

//TODO:USED IN OUR CURRENT SPARSE OCTREE, HOWEVER, IT REQUIRES AWKWARD COORDINATE TRANSFORMS AND OVERCOMPLICATES THE IMPLEMENTATION
//JUST USING REGULAR DOUBLE PRECISION COORDINATES IS SIMPLER AND EQUIVALENT IN PRECISION AT ALL SCALES THAT WOULD MATTER - REFACTOR SPARSE OCTREE
//TO JUST USE NORMAL DOUBLE PRECISION FVECTOR COORDINATES AND SCALES, THEN WE CAN GET RID OF THIS ENTIRELY
struct VOXELPLUGIN_API FInt64Coordinate
{
public:
	//We will "fudge" the int64 min/max range just a bit in the name of having consistent min and max coordinate bounds that are also divisible by 2
	static constexpr int64 MinCoord = INT64_MIN + 2; //-9,223,372,036,854,775,808 + 2 = -9,223,372,036,854,775,806
	static constexpr int64 MaxCoord = INT64_MAX - 1; //9,223,372,036,854,775,807 - 1  =  9,223,372,036,854,775,806

	int64 X;
	int64 Y;
	int64 Z;

    //Unit Conversions
    static FInt64Coordinate ToInt64Position(FVector InWorldPosition, double InPrecision)
    {
        return FInt64Coordinate(
            static_cast<int64>(InWorldPosition.X / InPrecision),
            static_cast<int64>(InWorldPosition.Y / InPrecision),
            static_cast<int64>(InWorldPosition.Z / InPrecision)
        );
    }
    static FVector ToWorldPosition(FInt64Coordinate InInternalPosition, double InPrecision)
    {
        return FVector(
            static_cast<double>(InInternalPosition.X) * InPrecision,
            static_cast<double>(InInternalPosition.Y) * InPrecision,
            static_cast<double>(InInternalPosition.Z) * InPrecision
        );
    }

    const FVector ToWorldPosition(double InPrecision) const
    {
        return FVector(
            static_cast<double>(X) * InPrecision,
            static_cast<double>(Y) * InPrecision,
            static_cast<double>(Z) * InPrecision
        );
    }

    //Operator overrides
	bool operator==(const FInt64Coordinate& Other) const
	{
		return (X == Other.X && Y == Other.Y && Z == Other.Z);
	}
	bool operator!=(const FInt64Coordinate& Other) const
	{
		return !(*this == Other);
	}
    FInt64Coordinate operator+(const FInt64Coordinate& Other) const
    {
        return FInt64Coordinate(X + Other.X, Y + Other.Y, Z + Other.Z);
    }
    FInt64Coordinate& operator+=(const FInt64Coordinate& Other)
    {
        X += Other.X;
        Y += Other.Y;
        Z += Other.Z;
        return *this;
    }
    FInt64Coordinate operator-(const FInt64Coordinate& Other) const
    {
        return FInt64Coordinate(X - Other.X, Y - Other.Y, Z - Other.Z);
    }
    FInt64Coordinate& operator-=(const FInt64Coordinate& Other)
    {
        X -= Other.X;
        Y -= Other.Y;
        Z -= Other.Z;
        return *this;
    }
    FInt64Coordinate operator*(int64 Scalar) const
    {
        return FInt64Coordinate(X * Scalar, Y * Scalar, Z * Scalar);
    }
    FInt64Coordinate& operator*=(int64 Scalar)
    {
        X *= Scalar;
        Y *= Scalar;
        Z *= Scalar;
        return *this;
    }
    FInt64Coordinate operator*(const FInt64Coordinate& Other) const
    {
        return FInt64Coordinate(X * Other.X, Y * Other.Y, Z * Other.Z);
    }
    FInt64Coordinate& operator*=(const FInt64Coordinate& Other)
    {
        X *= Other.X;
        Y *= Other.Y;
        Z *= Other.Z;
        return *this;
    }
    FInt64Coordinate operator/(int64 Scalar) const
    {
        if (Scalar == 0) return *this;  // Prevent division by zero
        return FInt64Coordinate(X / Scalar, Y / Scalar, Z / Scalar);
    }
    FInt64Coordinate& operator/=(int64 Scalar)
    {
        if (Scalar == 0) return *this;  // Prevent division by zero
        X /= Scalar;
        Y /= Scalar;
        Z /= Scalar;
        return *this;
    }
    FInt64Coordinate operator/(const FInt64Coordinate& Other) const
    {
        return FInt64Coordinate(
            Other.X != 0 ? X / Other.X : X,
            Other.Y != 0 ? Y / Other.Y : Y,
            Other.Z != 0 ? Z / Other.Z : Z
        );
    }
    FInt64Coordinate& operator/=(const FInt64Coordinate& Other)
    {
        if (Other.X != 0) X /= Other.X;
        if (Other.Y != 0) Y /= Other.Y;
        if (Other.Z != 0) Z /= Other.Z;
        return *this;
    }
    
    // Convert to double-based FVector for precision-based calculations
    FVector ToDoubleVector() const
    {
        return FVector(static_cast<double>(X), static_cast<double>(Y), static_cast<double>(Z));
    }
    // Convert to double-based FVector for precision-based calculations
    static FVector ToDoubleVector(FInt64Coordinate InternalCoordinate)
    {
        return FVector(static_cast<double>(InternalCoordinate.X), static_cast<double>(InternalCoordinate.Y), static_cast<double>(InternalCoordinate.Z));
    }
    // Convert to double-based FVector for precision-based calculations
    static FInt64Coordinate FromDoubleVector(FVector DoubleVector)
    {
        return FInt64Coordinate(static_cast<int64>(DoubleVector.X), static_cast<int64>(DoubleVector.Y), static_cast<int64>(DoubleVector.Z));
    }
    // Double-precision distance function
    static double DDist(const FInt64Coordinate& A, const FInt64Coordinate& B)
    {
        return FVector::Dist(A.ToDoubleVector(), B.ToDoubleVector());
    }

    static uint64 Dist(const FInt64Coordinate& A, const FInt64Coordinate& B)
    {
        int64 DX = A.X - B.X;
        int64 DY = A.Y - B.Y;
        int64 DZ = A.Z - B.Z;

        // Compute Euclidean distance, ensuring precision and avoiding overflow
        return static_cast<uint64>(FMath::Sqrt(
            static_cast<double>(DX) * DX +
            static_cast<double>(DY) * DY +
            static_cast<double>(DZ) * DZ
        ));
    }

	FInt64Coordinate() : 
        X(0), 
        Y(0), 
        Z(0) 
    {}
	FInt64Coordinate(int64 InX, int64 InY, int64 InZ) : 
        X(InX), 
        Y(InY), 
        Z(InZ) 
    {}
    FInt64Coordinate(FVector InWorldPosition, double InPrecision) : 
        X(static_cast<int64>(InWorldPosition.X / InPrecision)),
        Y(static_cast<int64>(InWorldPosition.Y / InPrecision)),
        Z(static_cast<int64>(InWorldPosition.Z / InPrecision)) 
    {}
    FInt64Coordinate(const FInt64Coordinate& InVector) : 
        X(InVector.X), 
        Y(InVector.Y), 
        Z(InVector.Z) 
    {}
};
