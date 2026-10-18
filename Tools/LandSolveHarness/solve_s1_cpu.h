// solve_s1_cpu.h - the solve job's S1 routines as the GPU records them (VoxelLandSolve.cpp:
// the tiled fill's dispatch, the routing's tree fill, the accumulation by jumps), on the CPU.
// Included by the solve and look harnesses after their FSolve, ParallelFor and SwapJump.

#pragma once

/** One FillTiledCS dispatch (VoxelLandSolve.usf): per 16 x 16 tile of a face, VOXEL_SOLVE_FILL_LOCAL
 *  Jacobi iterations in the tile, its outside neighbours read once at the start. The tiles run
 *  from a snapshot (the GPU's tiles may see each other's writes; that only converges sooner).
 *  True when any level fell. */
static bool FillTiledPass(FSolve& S, float Eps, std::vector<float>& Snap)
{
    const int N = (int)S.SolveN, T = 16, Local = 24, TP = (N + T - 1) / T;
    const uint Cells = S.VoxelSolveNumCells();
    Snap.assign(S.SolveW, S.SolveW + Cells);
    std::atomic<int> Lowered{0};
    ParallelFor(6 * TP * TP, [&](int Tile) {
        const int Face = Tile / (TP * TP), Rest = Tile % (TP * TP), TJ = Rest / TP, TI = Rest % TP;
        float Wt[T * T], Wn[T * T], H[T * T], Outside[T * T][8], Descent[T * T][8];
        int LocalIx[T * T][8];
        bool bSlot[T * T];
        uint CellOf[T * T];
        const bool bRandom = S.SolveRouteNoise > 0.0f && Eps > 0.0f;
        for (int t = 0; t < T * T; t++)
        {
            const int I = TI * T + t % T, J = TJ * T + t / T;
            const bool bCell = I < N && J < N;
            const uint Cell = bCell ? S.VoxelSolveEncode((uint)Face, I, J) : 0u;
            CellOf[t] = Cell;
            bSlot[t] = bCell && S.SolveSlot[Cell] != VOXEL_SOLVE_NONE;
            Wt[t] = bCell ? Snap[Cell] : VOXEL_SOLVE_INF;
            H[t] = bSlot[t] ? S.SolveH[Cell] : 0.0f;
            const float Factor = bSlot[t] ? S.VoxelSolveFillFactor(Cell, Eps) : 0.0f;
            for (uint c = 0; c < 8; c++)
            {
                LocalIx[t][c] = -1;
                Outside[t][c] = VOXEL_SOLVE_INF;
                Descent[t][c] = bRandom ? S.VoxelSolveFillDescent(Factor, c) : Eps;
                if (!bSlot[t]) continue;
                const uint Nb = S.VoxelSolveNeighbour(Cell, c);
                if (Nb == VOXEL_SOLVE_NONE) continue;
                const auto P = S.VoxelSolveDecode(Nb);
                const int LI = P.I - TI * T, LJ = P.J - TJ * T;
                if ((int)P.Face == Face && LI >= 0 && LJ >= 0 && LI < T && LJ < T) LocalIx[t][c] = LJ * T + LI;
                else Outside[t][c] = Snap[Nb];
            }
        }
        float W0[T * T];
        std::copy(Wt, Wt + T * T, W0);
        for (int k = 0; k < Local; k++)
        {
            for (int t = 0; t < T * T; t++)
            {
                float Lowest = VOXEL_SOLVE_INF;
                for (uint c = 0; c < 8; c++)
                    Lowest = std::min(Lowest, (LocalIx[t][c] >= 0 ? Wt[LocalIx[t][c]] : Outside[t][c]) + Descent[t][c]);
                const float Candidate = std::max(H[t], Lowest);
                Wn[t] = bSlot[t] && Candidate < Wt[t] ? Candidate : Wt[t];
            }
            std::copy(Wn, Wn + T * T, Wt);
        }
        for (int t = 0; t < T * T; t++)
            if (bSlot[t] && Wt[t] < W0[t]) { S.SolveW[CellOf[t]] = Wt[t]; Lowered = 1; }
    });
    return Lowered != 0;
}

/** The tiled fill until a pass lowers nothing: the passes it took (the last one counted). */
static int FillTiled(FSolve& S, float Eps)
{
    std::vector<float> Snap;
    for (int p = 1; p < 100000; p++)
        if (!FillTiledPass(S, Eps, Snap)) return p;
    return -1;
}

static void SwapAcc(FSolve& S) { std::swap(S.JumpY, S.JumpYOut); std::swap(S.AccHi, S.AccHiOut); std::swap(S.JumpP, S.JumpPOut); }

/** VoxelSolveAccJump*: the exact accumulation in Rounds rounds (Mode 0 the D8 drainage, 1 the
 *  spread's channels). The adds are atomic on the GPU; here sequential, which is the same sum. */
static void AccumulateByJumps(FSolve& S, uint L, uint Mode, int Rounds)
{
    ParallelFor(L, [&](int s) { S.VoxelSolveAccJumpInitPass(s, L, Mode); });
    for (int r = 0; r < Rounds; r++)
    {
        ParallelFor(L, [&](int s) { S.VoxelSolveAccJumpCopyPass(s, L); });
        for (uint s = 0; s < L; s++) S.VoxelSolveAccJumpAddPass(s, L);
        SwapAcc(S);
    }
    ParallelFor(L, [&](int s) { S.VoxelSolveAccJumpFinishPass(s, L, Mode); });
}

/** VoxelSolveRouteTreeInitPass and the tree fill's jumps: the routing fill's start. */
static void RouteTreeFill(FSolve& S, uint L, float Eps, int Rounds)
{
    ParallelFor(L, [&](int s) { S.VoxelSolveRouteTreeInitPass(s, L, Eps); });
    // The GPU's jump pass decodes with SolveEps whichever fill (TreeFillJumpCS).
    for (int r = 0; r < Rounds; r++) { ParallelFor(L, [&](int s) { S.VoxelSolveTreeFillJumpPass(s, L, S.SolveEps); }); SwapJump(S); }
    ParallelFor(L, [&](int s) { S.VoxelSolveTreeFillFinishPass(s, L); });
}

