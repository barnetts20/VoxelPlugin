// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
struct VOXELPLUGIN_API FQEF
{
    double ATA_00, ATA_01, ATA_02, ATA_11, ATA_12, ATA_22;
    double ATb_X, ATb_Y, ATb_Z;
    double BTB;
    int32 PlaneCount;
    FVector MassPoint;
    FVector AccumulatedNormal;

    FQEF() { Reset(); }

    void Reset()
    {
        ATA_00 = ATA_01 = ATA_02 = 0.0;
        ATA_11 = ATA_12 = 0.0;
        ATA_22 = 0.0;
        ATb_X = ATb_Y = ATb_Z = 0.0;
        BTB = 0.0;
        PlaneCount = 0;
        MassPoint = FVector::ZeroVector;
        AccumulatedNormal = FVector::ZeroVector;
    }

    void AddPlane(const FVector& InPoint, const FVector& InNormal)
    {
        double nx = InNormal.X, ny = InNormal.Y, nz = InNormal.Z;
        double d = nx * InPoint.X + ny * InPoint.Y + nz * InPoint.Z;

        ATA_00 += nx * nx; ATA_01 += nx * ny; ATA_02 += nx * nz;
        ATA_11 += ny * ny; ATA_12 += ny * nz; ATA_22 += nz * nz;
        ATb_X += nx * d; ATb_Y += ny * d; ATb_Z += nz * d;
        BTB += d * d;

        MassPoint += InPoint;
        AccumulatedNormal += InNormal;
        PlaneCount++;
    }

    FVector GetAverageNormal() const
    {
        if (PlaneCount == 0) return FVector::ZeroVector;
        return AccumulatedNormal.GetSafeNormal();
    }

    FVector Solve(const FVector& InCellCenter, double InCellExtent, double* OutError = nullptr) const
    {
        if (PlaneCount == 0)
        {
            if (OutError) *OutError = 0.0;
            return InCellCenter;
        }

        FVector AvgMassPoint = MassPoint / (double)PlaneCount;

        double rhs_x = ATb_X - (ATA_00 * AvgMassPoint.X + ATA_01 * AvgMassPoint.Y + ATA_02 * AvgMassPoint.Z);
        double rhs_y = ATb_Y - (ATA_01 * AvgMassPoint.X + ATA_11 * AvgMassPoint.Y + ATA_12 * AvgMassPoint.Z);
        double rhs_z = ATb_Z - (ATA_02 * AvgMassPoint.X + ATA_12 * AvgMassPoint.Y + ATA_22 * AvgMassPoint.Z);

        FVector Delta = SolveSVD(rhs_x, rhs_y, rhs_z);
        FVector Result = AvgMassPoint + Delta;

        FVector MinBound = InCellCenter - FVector(InCellExtent);
        FVector MaxBound = InCellCenter + FVector(InCellExtent);
        Result.X = FMath::Clamp(Result.X, MinBound.X, MaxBound.X);
        Result.Y = FMath::Clamp(Result.Y, MinBound.Y, MaxBound.Y);
        Result.Z = FMath::Clamp(Result.Z, MinBound.Z, MaxBound.Z);

        if (OutError) *OutError = ComputeError(Result);
        return Result;
    }

private:
    FVector SolveSVD(double rhs_x, double rhs_y, double rhs_z) const
    {
        double a[3][3] = {
            {ATA_00, ATA_01, ATA_02},
            {ATA_01, ATA_11, ATA_12},
            {ATA_02, ATA_12, ATA_22}
        };
        double v[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };

        for (int iter = 0; iter < 20; iter++)
        {
            int p = 0, q = 1;
            double maxVal = FMath::Abs(a[0][1]);
            if (FMath::Abs(a[0][2]) > maxVal) { p = 0; q = 2; maxVal = FMath::Abs(a[0][2]); }
            if (FMath::Abs(a[1][2]) > maxVal) { p = 1; q = 2; maxVal = FMath::Abs(a[1][2]); }
            if (maxVal < 1e-12) break;

            double theta = 0.5 * FMath::Atan2(2.0 * a[p][q], a[p][p] - a[q][q]);
            double c = FMath::Cos(theta);
            double s = FMath::Sin(theta);

            double newA[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    newA[i][j] = a[i][j];

            newA[p][p] = c * c * a[p][p] + 2 * s * c * a[p][q] + s * s * a[q][q];
            newA[q][q] = s * s * a[p][p] - 2 * s * c * a[p][q] + c * c * a[q][q];
            newA[p][q] = newA[q][p] = 0.0;

            int r = 3 - p - q;
            newA[p][r] = c * a[p][r] + s * a[q][r]; newA[r][p] = newA[p][r];
            newA[q][r] = -s * a[p][r] + c * a[q][r]; newA[r][q] = newA[q][r];

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    a[i][j] = newA[i][j];

            for (int i = 0; i < 3; i++)
            {
                double vip = v[i][p], viq = v[i][q];
                v[i][p] = c * vip + s * viq;
                v[i][q] = -s * vip + c * viq;
            }
        }

        double eigenvalues[3] = { a[0][0], a[1][1], a[2][2] };
        double maxEigen = FMath::Max3(FMath::Abs(eigenvalues[0]), FMath::Abs(eigenvalues[1]), FMath::Abs(eigenvalues[2]));
        double threshold = maxEigen * 0.1;

        double vtRhs[3];
        for (int i = 0; i < 3; i++)
            vtRhs[i] = v[0][i] * rhs_x + v[1][i] * rhs_y + v[2][i] * rhs_z;

        double scaled[3];
        for (int i = 0; i < 3; i++)
            scaled[i] = (FMath::Abs(eigenvalues[i]) > threshold) ? vtRhs[i] / eigenvalues[i] : 0.0;

        return FVector(
            v[0][0] * scaled[0] + v[0][1] * scaled[1] + v[0][2] * scaled[2],
            v[1][0] * scaled[0] + v[1][1] * scaled[1] + v[1][2] * scaled[2],
            v[2][0] * scaled[0] + v[2][1] * scaled[1] + v[2][2] * scaled[2]);
    }

    double ComputeError(const FVector& Point) const
    {
        double x = Point.X, y = Point.Y, z = Point.Z;
        double xTATAx = x * (ATA_00 * x + ATA_01 * y + ATA_02 * z)
            + y * (ATA_01 * x + ATA_11 * y + ATA_12 * z)
            + z * (ATA_02 * x + ATA_12 * y + ATA_22 * z);
        double xTATb = x * ATb_X + y * ATb_Y + z * ATb_Z;
        return xTATAx - 2.0 * xTATb + BTB;
    }
};