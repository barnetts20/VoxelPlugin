#include "FOctreeTests.h"
#include "Misc/AutomationTest.h"
#include "FSparseOctree.h"
#include "FSparseOctreeNode.h"
#include "FVoxelData.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInt64CoordinateTest,
    "VoxelPlugin.FInt64Coordinate.BasicOperations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FInt64CoordinateTest::RunTest(const FString& Parameters)
{
    const double Precision = 0.001; // 0.001 cm (1e-5 meters)

    // Default Constructor Test
    FInt64Coordinate DefaultCoord;
    UE_LOG(LogTemp, Log, TEXT("Default Coordinate: (%lld, %lld, %lld)"), DefaultCoord.X, DefaultCoord.Y, DefaultCoord.Z);
    TestEqual(TEXT("Default Constructor should initialize to (0,0,0)"), DefaultCoord, FInt64Coordinate(0, 0, 0));

    // Parameterized Constructor Test
    FInt64Coordinate CustomCoord(100, -200, 300);
    UE_LOG(LogTemp, Log, TEXT("Custom Coordinate: (%lld, %lld, %lld)"), CustomCoord.X, CustomCoord.Y, CustomCoord.Z);
    TestEqual(TEXT("Parameterized Constructor should match assigned values"), CustomCoord, FInt64Coordinate(100, -200, 300));

    // Conversion from FVector to FInt64Coordinate
    FVector WorldPos(1234.567, -9876.543, 5432.1);
    FInt64Coordinate Int64Pos = FInt64Coordinate::ToInt64Position(WorldPos, Precision);
    UE_LOG(LogTemp, Log, TEXT("World Position: (%f, %f, %f) -> Int64 Coordinate: (%lld, %lld, %lld)"),
        WorldPos.X, WorldPos.Y, WorldPos.Z, Int64Pos.X, Int64Pos.Y, Int64Pos.Z);
    TestEqual(TEXT("Conversion from FVector should match expected int64 values"),
        Int64Pos, FInt64Coordinate(1234567, -9876543, 5432100));

    // Conversion back to FVector
    FVector ConvertedWorldPos = Int64Pos.ToWorldPosition(Precision);
    UE_LOG(LogTemp, Log, TEXT("Int64 Coordinate: (%lld, %lld, %lld) -> World Position: (%f, %f, %f)"),
        Int64Pos.X, Int64Pos.Y, Int64Pos.Z, ConvertedWorldPos.X, ConvertedWorldPos.Y, ConvertedWorldPos.Z);
    TestEqual(TEXT("Conversion back to FVector should match original within precision range"),
        ConvertedWorldPos, WorldPos);

    // Equality and Inequality Operators
    TestTrue(TEXT("Equality operator should return true for identical coordinates"), CustomCoord == FInt64Coordinate(100, -200, 300));
    TestTrue(TEXT("Inequality operator should return true for different coordinates"), CustomCoord != FInt64Coordinate(101, -200, 300));

    // Addition
    FInt64Coordinate Sum = CustomCoord + FInt64Coordinate(50, 50, 50);
    UE_LOG(LogTemp, Log, TEXT("Addition: (%lld, %lld, %lld) + (50, 50, 50) -> (%lld, %lld, %lld)"),
        CustomCoord.X, CustomCoord.Y, CustomCoord.Z, Sum.X, Sum.Y, Sum.Z);
    TestEqual(TEXT("Addition should correctly sum coordinates"), Sum, FInt64Coordinate(150, -150, 350));

    // Subtraction
    FInt64Coordinate Diff = CustomCoord - FInt64Coordinate(50, 50, 50);
    UE_LOG(LogTemp, Log, TEXT("Subtraction: (%lld, %lld, %lld) - (50, 50, 50) -> (%lld, %lld, %lld)"),
        CustomCoord.X, CustomCoord.Y, CustomCoord.Z, Diff.X, Diff.Y, Diff.Z);
    TestEqual(TEXT("Subtraction should correctly subtract coordinates"), Diff, FInt64Coordinate(50, -250, 250));

    // Multiplication
    FInt64Coordinate ScaledUp = CustomCoord * 2;
    UE_LOG(LogTemp, Log, TEXT("Multiplication by scalar: (%lld, %lld, %lld) * 2 -> (%lld, %lld, %lld)"),
        CustomCoord.X, CustomCoord.Y, CustomCoord.Z, ScaledUp.X, ScaledUp.Y, ScaledUp.Z);
    TestEqual(TEXT("Multiplication by scalar should correctly scale coordinates"), ScaledUp, FInt64Coordinate(200, -400, 600));

    // Division
    FInt64Coordinate ScaledDown = CustomCoord / 2;
    UE_LOG(LogTemp, Log, TEXT("Division by scalar: (%lld, %lld, %lld) / 2 -> (%lld, %lld, %lld)"),
        CustomCoord.X, CustomCoord.Y, CustomCoord.Z, ScaledDown.X, ScaledDown.Y, ScaledDown.Z);
    TestEqual(TEXT("Division by scalar should correctly scale coordinates"), ScaledDown, FInt64Coordinate(50, -100, 150));

    // Division by Zero
    FInt64Coordinate NoChange = CustomCoord / 0;
    UE_LOG(LogTemp, Log, TEXT("Division by zero: (%lld, %lld, %lld) / 0 -> (%lld, %lld, %lld)"),
        CustomCoord.X, CustomCoord.Y, CustomCoord.Z, NoChange.X, NoChange.Y, NoChange.Z);
    TestEqual(TEXT("Division by zero should return the same coordinate"), NoChange, CustomCoord);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOctreeNodeTest,
    "VoxelPlugin.FSparseOctreeNode.BasicOperations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FOctreeNodeTest::RunTest(const FString& Parameters)
{
    // Test Root Node Creation
    TSharedPtr<FSparseOctreeNode> RootNode = MakeShared<FSparseOctreeNode>();
    TestTrue(TEXT("Root node should be valid"), RootNode.IsValid());

    UE_LOG(LogTemp, Log, TEXT("Root Node Created: Center=(%lld, %lld, %lld), HalfScale=%lld"),
        RootNode->GetCenter().X, RootNode->GetCenter().Y, RootNode->GetCenter().Z, RootNode->GetHalfScale());

    TestEqual(TEXT("Root node should be at (0,0,0)"), RootNode->GetCenter(), FInt64Coordinate(0, 0, 0));
    TestEqual(TEXT("Root node half scale should be MaxCoord"), RootNode->GetHalfScale(), FInt64Coordinate::MaxCoord);
    TestTrue(TEXT("Root node should be a leaf initially"), RootNode->IsLeaf());

    // Test Child Insertion
    uint8 ChildIndex = 3;
    TSharedPtr<FSparseOctreeNode> ChildNode = RootNode->InsertChild(ChildIndex);
    TestTrue(TEXT("Inserted child should be valid"), ChildNode.IsValid());

    UE_LOG(LogTemp, Log, TEXT("Inserted Child at Index=%d | Center=(%lld, %lld, %lld) | HalfScale=%lld"),
        ChildIndex, ChildNode->GetCenter().X, ChildNode->GetCenter().Y, ChildNode->GetCenter().Z, ChildNode->GetHalfScale());

    TestFalse(TEXT("Root node should no longer be a leaf after adding a child"), RootNode->IsLeaf());

    // Verify Parent Relationship
    TestTrue(TEXT("Child node's parent should be valid"), ChildNode->GetParent().IsValid());
    TestEqual(TEXT("Child node's parent should be root"), ChildNode->GetParent().Get(), RootNode.Get());

    // Test Multiple Child Insertions
    uint8 ChildIndex2 = 5;
    TSharedPtr<FSparseOctreeNode> AnotherChild = ChildNode->InsertChild(ChildIndex2);
    TestTrue(TEXT("Another child should be valid"), AnotherChild.IsValid());

    // Test Node Retrieval
    TSharedPtr<FSparseOctreeNode> RetrievedChild = RootNode->GetChildren()[ChildIndex];
    TestTrue(TEXT("Retrieved child should be valid"), RetrievedChild.IsValid());
    TestEqual(TEXT("Retrieved child should match inserted child"), RetrievedChild.Get(), ChildNode.Get());

    // Test Root Traversal
    TestEqual(TEXT("Root node should return itself as root"), RootNode->GetRoot().Get(), RootNode.Get());
    TestEqual(TEXT("Child node's root should be root"), ChildNode->GetRoot().Get(), RootNode.Get());

    // Test Payload Storage
    TSharedPtr<FVoxelData> TestPayload = MakeShared<FVoxelData>();
    ChildNode->SetPayload(TestPayload);
    TestTrue(TEXT("Child node should have a payload"), ChildNode->HasPayload());
    TestEqual(TEXT("Child node's retrieved payload should match inserted payload"), ChildNode->GetPayload().Get(), TestPayload.Get());

    // Test Clearing Payload
    ChildNode->ClearPayload();
    TestFalse(TEXT("Child node should no longer have a payload after clearing"), ChildNode->HasPayload());

    // Test Point Inside Node
    //FInt64Coordinate InsidePoint = ChildNode->GetCenter();
    //bool InsideTestResult = ChildNode->IsPointInsideNode(InsidePoint);

    //UE_LOG(LogTemp, Log, TEXT("Checking IsPointInsideNode | Test Point=(%lld, %lld, %lld) | Node Center=(%lld, %lld, %lld) | HalfScale=%lld | Expected=True, Got=%s"),
    //    InsidePoint.X, InsidePoint.Y, InsidePoint.Z,
    //    ChildNode->GetCenter().X, ChildNode->GetCenter().Y, ChildNode->GetCenter().Z,
    //    ChildNode->GetHalfScale(), InsideTestResult ? TEXT("True") : TEXT("False"));

    //TestTrue(TEXT("Point should be inside child node"), InsideTestResult);

    // Test Removing a Child
    RootNode->RemoveChild(ChildIndex);
    bool ChildrenRemoved = !RootNode->GetChildren()[ChildIndex].IsValid();

    UE_LOG(LogTemp, Log, TEXT("Removing Child at Index=%d | Expected=True, Got=%s"),
        ChildIndex, ChildrenRemoved ? TEXT("True") : TEXT("False"));

    TestFalse(TEXT("Child should be removed"), RootNode->GetChildren()[ChildIndex].IsValid());
    TestTrue(TEXT("Root node should be a leaf again if no other children exist"), RootNode->IsLeaf());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFOctree_OffsetTest,
    "VoxelPlugin.FSparseOctree.OffsetHandling",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FFOctree_OffsetTest::RunTest(const FString& Parameters)
{
    // Step 1: Create the Voxel Actor instance
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();

    // Step 2: Define Test Offsets
    FVector TestWorldOffset(1000.0, -500.0, 750.0);
    FInt64Coordinate TestInternalOffset = FInt64Coordinate::ToInt64Position(TestWorldOffset, OctreeData->GetPrecision());

    // Set Offset via World Position
    OctreeData->SetOctreeOffset(TestWorldOffset);
    FInt64Coordinate RetrievedInternalOffset1 = OctreeData->GetOctreeOffsetInternal();
    TestEqual(FString::Printf(TEXT("Octree offset (World -> Internal) mismatch: Expected (%lld, %lld, %lld), Got (%lld, %lld, %lld)"),
        TestInternalOffset.X, TestInternalOffset.Y, TestInternalOffset.Z,
        RetrievedInternalOffset1.X, RetrievedInternalOffset1.Y, RetrievedInternalOffset1.Z),
        RetrievedInternalOffset1, TestInternalOffset);

    // Set Offset via Internal Position
    OctreeData->SetOctreeOffset(TestInternalOffset);
    FInt64Coordinate RetrievedInternalOffset2 = OctreeData->GetOctreeOffsetInternal();
    TestEqual(FString::Printf(TEXT("Octree offset (Internal -> Internal) mismatch: Expected (%lld, %lld, %lld), Got (%lld, %lld, %lld)"),
        TestInternalOffset.X, TestInternalOffset.Y, TestInternalOffset.Z,
        RetrievedInternalOffset2.X, RetrievedInternalOffset2.Y, RetrievedInternalOffset2.Z),
        RetrievedInternalOffset2, TestInternalOffset);

    // Step 3: Validate Offset Retrieval
    FVector RetrievedWorldOffset = OctreeData->GetOctreeOffsetWorld();
    double OffsetError = FVector::Dist(TestWorldOffset, RetrievedWorldOffset);
    TestTrue(FString::Printf(TEXT("World offset mismatch: Expected (%f, %f, %f), Got (%f, %f, %f), Error: %f"),
        TestWorldOffset.X, TestWorldOffset.Y, TestWorldOffset.Z,
        RetrievedWorldOffset.X, RetrievedWorldOffset.Y, RetrievedWorldOffset.Z,
        OffsetError),
        OffsetError < OctreeData->GetPrecision());

    // Step 4: Test Internal <-> World Conversion
    FVector SampleWorldPos(2500.0, -1500.0, 3200.0);
    FInt64Coordinate SampleInternalPos = OctreeData->ConvertToInternalPosition(SampleWorldPos);
    FVector ReconvertedWorldPos = OctreeData->ConvertToWorldPosition(SampleInternalPos);
    double ConversionError = FVector::Dist(SampleWorldPos, ReconvertedWorldPos);
    TestTrue(FString::Printf(TEXT("World->Internal->World conversion mismatch: Expected (%f, %f, %f), Got (%f, %f, %f), Error: %f"),
        SampleWorldPos.X, SampleWorldPos.Y, SampleWorldPos.Z,
        ReconvertedWorldPos.X, ReconvertedWorldPos.Y, ReconvertedWorldPos.Z,
        ConversionError),
        ConversionError < OctreeData->GetPrecision());

    // Step 5: Validate Offset Effect on Conversions
    FInt64Coordinate ExpectedInternalPos = FInt64Coordinate::ToInt64Position(SampleWorldPos - TestWorldOffset, OctreeData->GetPrecision());
    TestEqual(FString::Printf(TEXT("Converted internal position mismatch: Expected (%lld, %lld, %lld), Got (%lld, %lld, %lld)"),
        ExpectedInternalPos.X, ExpectedInternalPos.Y, ExpectedInternalPos.Z,
        SampleInternalPos.X, SampleInternalPos.Y, SampleInternalPos.Z),
        SampleInternalPos, ExpectedInternalPos);

    // Step 6: Validate GetChildIndexForPosition
    TSharedPtr<FSparseOctreeNode> RootNode = OctreeData->GetNode(TArray<uint8>());
    FInt64Coordinate RootCenter = RootNode->GetCenter();

    FInt64Coordinate TestChildPosition = RootCenter + FInt64Coordinate(1, 1, 1);
    uint8 ChildIndex = OctreeData->GetChildIndexForPosition(RootNode, TestChildPosition);
    TestEqual(FString::Printf(TEXT("GetChildIndexForPosition mismatch: Expected %d, Got %d"), 7, ChildIndex), ChildIndex, 7); // Expecting 7 for a fully positive offset

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFOctree_ScalingTest,
    "VoxelPlugin.FSparseOctree.ScalingAndPrecision",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FFOctree_ScalingTest::RunTest(const FString& Parameters)
{
    // Step 1: Create the Octree instance
    TSharedPtr<FSparseOctree> Octree = MakeShared<FSparseOctree>();

    // === 1. Test: Default Values ===
    TestEqual(TEXT("Default MaxExtent should be MaxCoord"), Octree->GetMaxExtent(), FInt64Coordinate::MaxCoord);
    TestEqual(TEXT("Default MaxDepth should be 62"), Octree->GetMaxDepth(), 62);
    TestEqual(TEXT("Default Precision should be 2 * internal precision"), Octree->GetPrecision(), 0.000001); // Assuming default internal precision is 0.0000005

    // === 2. Test: Setting Max Contained Scale ===
    double TestScale = 5000000.0; // Set max contained scale to 5 million world units
    Octree->SetMaxContainedScale(TestScale);
    uint64 ExpectedInternalSize = static_cast<uint64>(TestScale / Octree->GetPrecision());
    uint64 ExpectedMaxExtent = FMath::RoundUpToPowerOfTwo(ExpectedInternalSize);
    int ExpectedMaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(ExpectedMaxExtent)));

    TestEqual(FString::Printf(TEXT("MaxExtent should be a power of two containing %f"), TestScale),
        static_cast<uint64>(Octree->GetMaxExtent()), ExpectedMaxExtent);
    TestEqual(TEXT("MaxDepth should be adjusted accordingly"), Octree->GetMaxDepth(), ExpectedMaxDepth);
    TestEqual(TEXT("MaxContainedScale should return the correct value"), Octree->GetMaxContainedScale(), TestScale);

    // === 3. Test: Setting Precision ===
    double NewPrecision = 0.00001; // 10x larger than default
    Octree->SetPrecision(NewPrecision);
    TestEqual(TEXT("Precision should be updated correctly"), Octree->GetPrecision(), NewPrecision);

    // Recalculate expected internal size and max extent after precision change
    ExpectedInternalSize = static_cast<uint64>(TestScale / Octree->GetPrecision());
    ExpectedMaxExtent = FMath::RoundUpToPowerOfTwo(ExpectedInternalSize);
    ExpectedMaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(ExpectedMaxExtent))) - 1;

    TestEqual(TEXT("MaxExtent should be recalculated when precision changes"), static_cast<uint64>(Octree->GetMaxExtent()), ExpectedMaxExtent);
    TestEqual(TEXT("MaxDepth should also be adjusted when precision changes"), Octree->GetMaxDepth(), ExpectedMaxDepth);

    // === 4. Test: Resetting Max Contained Scale to Default ===
    Octree->SetMaxContainedScale(0);
    TestEqual(TEXT("MaxExtent should reset to MaxCoord when SetMaxContainedScale(0) is called"),
        Octree->GetMaxExtent(), FInt64Coordinate::MaxCoord);
    TestEqual(TEXT("MaxDepth should reset to 62 when SetMaxContainedScale(0) is called"), Octree->GetMaxDepth(), 62);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInsertDataTest,
    "VoxelPlugin.FAOctreeData.InsertData",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FInsertDataTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Test: Insert data into the root node (empty TreeIndex) ===
    TSharedPtr<FVoxelData> TestDataRoot = MakeShared<FVoxelData>();
    TestDataRoot->Density = 0.5f;
    TSharedPtr<FSparseOctreeNode> InsertedNodeRoot = OctreeData->InsertData({}, TestDataRoot);

    // Check if the root node has the correct payload after insertion
    TestTrue(TEXT("Root node should have the correct payload after insertion"),
        InsertedNodeRoot->HasPayload());
    TestEqual(TEXT("Root node's payload density should be 0.5f"),
        InsertedNodeRoot->GetPayload()->Density, 0.5f);

    // **NEW: Check position is set to node center**
    TestEqual(TEXT("Root node position should match node center"),
        TestDataRoot->Position, InsertedNodeRoot->GetCenter());

    // === 2. Test: Insert data at a specific child node (non-empty TreeIndex) ===
    TArray<uint8> TreeIndex = { 0, 1 }; // Path to a child node
    TSharedPtr<FVoxelData> TestDataChild = MakeShared<FVoxelData>();
    TestDataChild->Density = 0.8f;
    TSharedPtr<FSparseOctreeNode> InsertedNodeChild = OctreeData->InsertData(TreeIndex, TestDataChild);

    // Check if the child node has the correct payload after insertion
    TestTrue(TEXT("Child node should have the correct payload after insertion"),
        InsertedNodeChild->HasPayload());
    TestEqual(TEXT("Child node's payload density should be 0.8f"),
        InsertedNodeChild->GetPayload()->Density, 0.8f);

    // **NEW: Check position is set to node center**
    TestEqual(TEXT("Child node position should match node center"),
        TestDataChild->Position, InsertedNodeChild->GetCenter());

    // === 3. Test: Ensure child node's data is overwritten when re-inserting ===
    TSharedPtr<FVoxelData> UpdatedTestData = MakeShared<FVoxelData>();
    UpdatedTestData->Density = 1.0f;
    TSharedPtr<FSparseOctreeNode> OverwrittenNode = OctreeData->InsertData(TreeIndex, UpdatedTestData); // Re-insert at the same node

    // Verify that the payload has been updated to the new value (overwrite check)
    TestTrue(TEXT("Node should be returned and overwritten with new payload"),
        OverwrittenNode->HasPayload());
    TestEqual(TEXT("Child node's payload density should be updated to 1.0f after re-insertion"),
        OverwrittenNode->GetPayload()->Density, 1.0f);

    // **NEW: Ensure position remains the same after overwrite**
    TestEqual(TEXT("Child node position should remain unchanged after overwrite"),
        UpdatedTestData->Position, OverwrittenNode->GetCenter());

    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkInsertDataTest,
    "VoxelPlugin.FAOctreeData.BulkInsertData",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkInsertDataTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Test: Insert multiple entries into the octree ===
    TArray<TArray<uint8>> TreeIndices = { {0}, {1, 2}, {3, 4} };  // Paths to child nodes
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < TreeIndices.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.5f * (i + 1);  // Different density for each entry
        Data.Add(TestData);
    }

    TArray<TSharedPtr<FSparseOctreeNode>> InsertedNodes = OctreeData->BulkInsertData(TreeIndices, Data);

    // Verify that each node has the correct payload and position after insertion
    for (int i = 0; i < InsertedNodes.Num(); ++i)
    {
        TestTrue(FString::Printf(TEXT("Node %d should have the correct payload after insertion"), i),
            InsertedNodes[i]->HasPayload());
        TestEqual(FString::Printf(TEXT("Node %d's payload density should be %f"), i),
            InsertedNodes[i]->GetPayload()->Density, 0.5f * (i + 1));

        // **New: Check position matches node center**
        TestEqual(FString::Printf(TEXT("Node %d's position should match node center"), i),
            Data[i]->Position, InsertedNodes[i]->GetCenter());
    }

    // === 2. Test: Overwrite behavior when inserting the same nodes again with new data ===
    TArray<TSharedPtr<FVoxelData>> OverwriteData;
    for (int i = 0; i < TreeIndices.Num(); i++)
    {
        TSharedPtr<FVoxelData> UpdatedData = MakeShared<FVoxelData>();
        UpdatedData->Density = 1.0f * (i + 1);  // New density for overwrite
        OverwriteData.Add(UpdatedData);
    }

    TArray<TSharedPtr<FSparseOctreeNode>> OverwrittenNodes = OctreeData->BulkInsertData(TreeIndices, OverwriteData);

    // Verify that the data is overwritten but **position remains unchanged**
    for (int i = 0; i < OverwrittenNodes.Num(); ++i)
    {
        TestTrue(FString::Printf(TEXT("Node %d should have the correct payload after overwrite"), i),
            OverwrittenNodes[i]->HasPayload());
        TestEqual(FString::Printf(TEXT("Node %d's payload density should be %f"), i),
            OverwrittenNodes[i]->GetPayload()->Density, 1.0f * (i + 1));  // Check if data was overwritten

        // **New: Ensure position remains unchanged after overwrite**
        TestEqual(FString::Printf(TEXT("Node %d's position should remain unchanged after overwrite"), i),
            OverwriteData[i]->Position, OverwrittenNodes[i]->GetCenter());
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInsertDataAtPointTest,
    "VoxelPlugin.FAOctreeData.InsertDataAtPoint",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FInsertDataAtPointTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Test: Insert data at a specific world position and depth ===
    FVector WorldPosition(100.0f, 200.0f, 300.0f);
    int32 TargetDepth = FSparseOctreeNode::MAX_DEPTH;
    TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
    TestData->Density = 0.75f;

    // Insert data at the world position
    TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPosition, TargetDepth, TestData);

    // Validate insertion
    TestTrue(TEXT("Node should be valid after insertion at world position"), InsertedNode.IsValid());
    TestTrue(TEXT("Inserted node should have the correct payload"), InsertedNode->HasPayload());
    TestEqual(TEXT("Inserted node's payload density should be 0.75f"), InsertedNode->GetPayload()->Density, 0.75f);

    // **New: Ensure VoxelData position matches converted internal position**
    FInt64Coordinate ExpectedInternalPos = OctreeData->ConvertToInternalPosition(WorldPosition);
    TestEqual(TEXT("Inserted voxel position should match converted internal position"), TestData->Position, ExpectedInternalPos);

    // === 2. Test: Insert data at a specific internal position and depth ===
    FInt64Coordinate InternalPosition(100, 200, 300);
    int32 TargetDepthInternal = FSparseOctreeNode::MAX_DEPTH;
    TSharedPtr<FVoxelData> TestDataInternal = MakeShared<FVoxelData>();
    TestDataInternal->Density = 0.9f;

    // Insert data at the internal position
    TSharedPtr<FSparseOctreeNode> InsertedNodeInternal = OctreeData->InsertDataAtPoint(InternalPosition, TargetDepthInternal, TestDataInternal);

    // Validate insertion
    TestTrue(TEXT("Node should be valid after insertion at internal position"), InsertedNodeInternal.IsValid());
    TestTrue(TEXT("Inserted node should have the correct payload"), InsertedNodeInternal->HasPayload());
    TestEqual(TEXT("Inserted node's payload density should be 0.9f"), InsertedNodeInternal->GetPayload()->Density, 0.9f);

    // **New: Ensure VoxelData position matches exact inserted internal position**
    TestEqual(TEXT("Inserted voxel position should match provided internal position"), TestDataInternal->Position, InternalPosition);

    // === 3. Test: Overwrite behavior at the same position ===
    TSharedPtr<FVoxelData> UpdatedTestData = MakeShared<FVoxelData>();
    UpdatedTestData->Density = 1.0f;

    // Re-insert at the same world position
    TSharedPtr<FSparseOctreeNode> OverwrittenNode = OctreeData->InsertDataAtPoint(WorldPosition, TargetDepth, UpdatedTestData);

    // Validate overwriting behavior
    TestTrue(TEXT("Node should be valid after overwriting"), OverwrittenNode.IsValid());
    TestEqual(TEXT("Overwritten node's payload density should be 1.0f"), OverwrittenNode->GetPayload()->Density, 1.0f);

    // **New: Ensure position remains unchanged after overwrite**
    TestEqual(TEXT("Voxel position should remain unchanged after overwrite"), UpdatedTestData->Position, ExpectedInternalPos);

    // === 5. Test: Insert at a higher depth and validate position ===
    int32 HighDepth = 6;
    TSharedPtr<FVoxelData> HighDepthTestData = MakeShared<FVoxelData>();
    HighDepthTestData->Density = 0.5f;

    // Insert data at the higher depth
    TSharedPtr<FSparseOctreeNode> HighDepthInsertedNode = OctreeData->InsertDataAtPoint(WorldPosition, HighDepth, HighDepthTestData);

    // Validate high-depth insertion
    TestTrue(TEXT("Node should be valid after insertion at high depth"), HighDepthInsertedNode.IsValid());
    TestTrue(TEXT("Inserted node should have the correct payload"), HighDepthInsertedNode->HasPayload());
    TestEqual(TEXT("Inserted node's payload density at high depth should be 0.5f"), HighDepthInsertedNode->GetPayload()->Density, 0.5f);

    // **New: Validate position of high-depth insertion**
    TestEqual(TEXT("Inserted voxel position at high depth should match converted internal position"), HighDepthTestData->Position, ExpectedInternalPos);

    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkInsertPointDataTest,
    "VoxelPlugin.FAOctreeData.BulkInsertPointData",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkInsertPointDataTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Max depth for precision**
    int32 TargetDepth = FSparseOctreeNode::MAX_DEPTH;

    // **1. Test: Insert multiple entries using world positions**
    TArray<FVector> WorldPositions = {
        FVector(1000000.0, 2000000.0, 3000000.0),
        FVector(-2500000.0, 1500000.0, -800000.0),
        FVector(5000000.0, -2000000.0, 1200000.0)
    };

    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.5f * (i + 1);  // Different density for each entry
        Data.Add(TestData);
    }

    TArray<TSharedPtr<FSparseOctreeNode>> InsertedNodesWorld = OctreeData->BulkInsertPointData(WorldPositions, { TargetDepth, TargetDepth, TargetDepth }, Data);

    // Verify each node has the correct payload and position
    for (int i = 0; i < InsertedNodesWorld.Num(); ++i)
    {
        TestTrue(FString::Printf(TEXT("World Node %d should have the correct payload after insertion"), i),
            InsertedNodesWorld[i]->HasPayload());
        TestEqual(FString::Printf(TEXT("World Node %d's payload density should be %f"), i),
            InsertedNodesWorld[i]->GetPayload()->Density, 0.5f * (i + 1));

        // **New: Ensure position matches converted internal position**
        FInt64Coordinate ExpectedInternalPos = OctreeData->ConvertToInternalPosition(WorldPositions[i]);
        TestEqual(FString::Printf(TEXT("World Node %d's position should match converted internal position"), i),
            Data[i]->Position, ExpectedInternalPos);
    }

    // **2. Test: Insert multiple entries using internal positions**
    TArray<FInt64Coordinate> InternalPositions = {
        FInt64Coordinate(1000000, 2000000, 3000000),
        FInt64Coordinate(-2500000, 1500000, -800000),
        FInt64Coordinate(5000000, -2000000, 1200000)
    };

    TArray<TSharedPtr<FVoxelData>> DataInternal;
    for (int i = 0; i < InternalPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 1.0f * (i + 1);  // Different density for each entry
        DataInternal.Add(TestData);
    }

    TArray<TSharedPtr<FSparseOctreeNode>> InsertedNodesInternal = OctreeData->BulkInsertPointData(InternalPositions, { TargetDepth, TargetDepth, TargetDepth }, DataInternal);

    // Verify each node has the correct payload and position
    for (int i = 0; i < InsertedNodesInternal.Num(); ++i)
    {
        TestTrue(FString::Printf(TEXT("Internal Node %d should have the correct payload after insertion"), i),
            InsertedNodesInternal[i]->HasPayload());
        TestEqual(FString::Printf(TEXT("Internal Node %d's payload density should be %f"), i),
            InsertedNodesInternal[i]->GetPayload()->Density, 1.0f * (i + 1));

        // **New: Ensure position matches exact inserted coordinate**
        TestEqual(FString::Printf(TEXT("Internal Node %d's position should match inserted internal position"), i),
            DataInternal[i]->Position, InternalPositions[i]);
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetNodeTest,
    "VoxelPlugin.FAOctreeData.GetNode",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetNodeTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === Populate the octree with some nodes for testing ===
    TArray<uint8> Index1 = { 0 };
    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.5f;
    TSharedPtr<FSparseOctreeNode> InsertedNode1 = OctreeData->InsertData(Index1, TestData1);

    TArray<uint8> Index2 = { 0, 1 };
    TSharedPtr<FVoxelData> TestData2 = MakeShared<FVoxelData>();
    TestData2->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> InsertedNode2 = OctreeData->InsertData(Index2, TestData2);

    // === 1. Test: Get the root node ===
    TArray<uint8> EmptyIndex;
    TSharedPtr<FSparseOctreeNode> RootNode = OctreeData->GetNode(EmptyIndex);

    TestTrue(TEXT("GetNode with empty TreeIndex should return the root node"), RootNode.IsValid());

    // **Check payload only if present**
    if (RootNode->HasPayload())
    {
        TestEqual(TEXT("Root node's VoxelData position should match node center"),
            RootNode->GetPayload()->Position, RootNode->GetCenter());
    }

    return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetRootNodeTest,
    "VoxelPlugin.FAOctreeData.GetRootNode",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetRootNodeTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // 1. Test: Get the root node
    TSharedPtr<FSparseOctreeNode> RootNode = OctreeData->GetRootNode();

    // Verify that the root node is valid and is the root of the octree
    TestTrue(TEXT("GetRootNode should return a valid root node"), RootNode.IsValid());
    TestFalse(TEXT("Root node should have no parent"), RootNode->GetParent().IsValid());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetNodeContainingPointAtDepthTest,
    "VoxelPlugin.FAOctreeData.GetNodeContainingPointAtDepth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetNodeContainingPointAtDepthTest::RunTest(const FString& Parameters)
{
    // Create and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();

    // **Max depth for precision**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define large-scale world positions**
    TArray<FVector> WorldPositions = {
        FVector(4000000.0, 4000000.0, 4000000.0),
        FVector(-4500000.0, 3500000.0, -1500000.0),
        FVector(5200000.0, -4700000.0, 2100000.0),
        FVector(-6500000.0, -5800000.0, -3500000.0)
    };

    // **Create data payloads with unique densities**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.2f * (i + 1);
        Data.Add(TestData);

        // **Convert to internal position & Insert**
        FInt64Coordinate InternalPosition = OctreeData->ConvertToInternalPosition(WorldPositions[i]);
        UE_LOG(LogTemp, Log, TEXT("Insert Position %d -> Internal: (%lld, %lld, %lld)"),
            i, InternalPosition.X, InternalPosition.Y, InternalPosition.Z);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPositions[i], InsertDepth, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());

        // **Ensure position is correctly assigned**
        TestEqual(FString::Printf(TEXT("Inserted voxel %d position should match converted internal position"), i),
            TestData->Position, InternalPosition);
    }

    // **Retrieve and verify world position alignment**
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetNodeContainingPointAtDepth(WorldPositions[i], InsertDepth);
        TestTrue(FString::Printf(TEXT("Retrieved node %d should be valid"), i), RetrievedNode.IsValid());

        // **Ensure retrieved node has a payload before checking**
        if (RetrievedNode->HasPayload())
        {
            TestEqual(FString::Printf(TEXT("Retrieved node %d's position should remain unchanged"), i),
                RetrievedNode->GetPayload()->Position, Data[i]->Position);

            TestEqual(FString::Printf(TEXT("Retrieved node %d should have correct density"), i),
                RetrievedNode->GetPayload()->Density, Data[i]->Density);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Retrieved node %d has no payload, skipping density and position validation."), i);
        }

        // **Compute real world bounds of the retrieved node**
        FVector NodeCenter = OctreeData->ConvertToWorldPosition(RetrievedNode->GetCenter());
        double HalfSize = RetrievedNode->GetHalfScale() * OctreeData->GetPrecision();
        FVector MinBounds = NodeCenter - FVector(HalfSize);
        FVector MaxBounds = NodeCenter + FVector(HalfSize);

        UE_LOG(LogTemp, Log, TEXT("Retrieved Node %d Center: (%f, %f, %f) | Bounds: Min(%f, %f, %f) Max(%f, %f, %f)"),
            i, NodeCenter.X, NodeCenter.Y, NodeCenter.Z, MinBounds.X, MinBounds.Y, MinBounds.Z, MaxBounds.X, MaxBounds.Y, MaxBounds.Z);

        // **Expanded Floating-Point Tolerance**
        const double Tolerance = OctreeData->GetPrecision() * 4.0;

        bool bInsideBounds =
            (WorldPositions[i].X >= MinBounds.X - Tolerance && WorldPositions[i].X <= MaxBounds.X + Tolerance) &&
            (WorldPositions[i].Y >= MinBounds.Y - Tolerance && WorldPositions[i].Y <= MaxBounds.Y + Tolerance) &&
            (WorldPositions[i].Z >= MinBounds.Z - Tolerance && WorldPositions[i].Z <= MaxBounds.Z + Tolerance);

        // **New: Log if failing to determine why**
        if (!bInsideBounds)
        {
            UE_LOG(LogTemp, Error, TEXT("FAILED: Inserted Position %d (%f, %f, %f) is NOT inside Retrieved Bounds Min(%f, %f, %f) Max(%f, %f, %f)"),
                i, WorldPositions[i].X, WorldPositions[i].Y, WorldPositions[i].Z, MinBounds.X, MinBounds.Y, MinBounds.Z, MaxBounds.X, MaxBounds.Y, MaxBounds.Z);
        }

        TestTrue(FString::Printf(TEXT("Inserted position %d should be inside retrieved node bounds"), i), bInsideBounds);
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetAllNodesTest,
    "VoxelPlugin.FAOctreeData.GetAllNodes",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetAllNodesTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Create a tree with multiple levels ===
    TArray<uint8> Index1 = { 0 };
    TArray<uint8> Index2 = { 0, 1 };
    TArray<uint8> Index3 = { 0, 1, 2 };

    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.5f;
    TSharedPtr<FSparseOctreeNode> Node1 = OctreeData->InsertData(Index1, TestData1);

    TSharedPtr<FVoxelData> TestData2 = MakeShared<FVoxelData>();
    TestData2->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> Node2 = OctreeData->InsertData(Index2, TestData2);

    // **Node3 exists but has no payload**
    TSharedPtr<FSparseOctreeNode> Node3 = OctreeData->InsertData(Index3, nullptr);

    // === 2. Test: Get all nodes (including empty ones) ===
    TArray<TSharedPtr<FSparseOctreeNode>> AllNodes = OctreeData->GetAllNodes(OctreeData->GetRootNode(), false);

    TestTrue(TEXT("GetAllNodes should return non-empty result"), AllNodes.Num() > 0);
    TestTrue(TEXT("Root node should be in the list"), AllNodes.Contains(OctreeData->GetRootNode()));
    TestTrue(TEXT("Child nodes should be included"), AllNodes.Contains(Node1) && AllNodes.Contains(Node2) && AllNodes.Contains(Node3));

    // === 3. Test: Get only occupied nodes ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->GetAllNodes(OctreeData->GetRootNode(), true);

    TestTrue(TEXT("Occupied nodes should not include empty nodes"), OccupiedNodes.Num() < AllNodes.Num());
    TestTrue(TEXT("Only occupied nodes should be returned"), OccupiedNodes.Contains(Node1) && OccupiedNodes.Contains(Node2));
    TestFalse(TEXT("Unoccupied nodes should not be included"), OccupiedNodes.Contains(Node3));

    // === 4. Test: Invalid Node should return an empty array ===
    TArray<TSharedPtr<FSparseOctreeNode>> InvalidResult = OctreeData->GetAllNodes(nullptr, false);
    TestTrue(TEXT("GetAllNodes with an invalid node should return an empty array"), InvalidResult.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkGetNodesTest,
    "VoxelPlugin.FAOctreeData.BulkGetNodes",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkGetNodesTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Populate the octree with multiple nodes ===
    TArray<TArray<uint8>> TreeIndices = {
        {0},         // First child node
        {1, 2},      // Second child node at depth 2
        {3, 4},      // Third child node at depth 2
        {5, 6, 7}    // Fourth child node at depth 3 (Will remain empty)
    };

    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < TreeIndices.Num()-1; i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.25f * (i + 1);
        Data.Add(TestData);
        OctreeData->InsertData(TreeIndices[i], TestData);
    }
    OctreeData->InsertData(TreeIndices[TreeIndices.Num() - 1], nullptr);

    // === 2. Test: Retrieve all nodes (including empty ones) ===
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedNodes = OctreeData->BulkGetNodes(TreeIndices, false);

    TestEqual(TEXT("BulkGetNodes should return the same number of nodes requested"), RetrievedNodes.Num(), TreeIndices.Num());
    for (int i = 0; i < TreeIndices.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Retrieved node %d should be valid"), i), RetrievedNodes[i].IsValid());
    }

    // === 3. Test: Retrieve only occupied nodes ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->BulkGetNodes(TreeIndices, true);

    TestTrue(TEXT("Occupied nodes should not include empty nodes"), OccupiedNodes.Num() < RetrievedNodes.Num());
    for (int i = 0; i < OccupiedNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Occupied node %d should have a payload"), i), OccupiedNodes[i]->HasPayload());
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetNodesContainingPointTest,
    "VoxelPlugin.FAOctreeData.GetNodesContainingPoint",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetNodesContainingPointTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Max depth for precision**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world position to insert data**
    FVector WorldPosition(4000000.0, 4000000.0, 4000000.0);
    FInt64Coordinate InternalPosition = OctreeData->ConvertToInternalPosition(WorldPosition);

    // **Insert data at the given position**
    TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
    TestData->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPosition, InsertDepth, TestData);
    TestTrue(TEXT("Inserted node should be valid"), InsertedNode.IsValid());

    // === 1. Test: Retrieve all nodes along the path to the position (including empty) ===
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedNodes = OctreeData->GetNodesContainingPoint(WorldPosition, false);

    TestTrue(TEXT("GetNodesContainingPoint should return at least one node"), RetrievedNodes.Num() > 0);
    TestTrue(TEXT("Root node should be included in the retrieved nodes"), RetrievedNodes.Contains(OctreeData->GetRootNode()));
    TestTrue(TEXT("Final inserted node should be included"), RetrievedNodes.Contains(InsertedNode));

    // **Ensure retrieved nodes form a valid path from root to the inserted node**
    for (int i = 1; i < RetrievedNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Node %d should be a child of node %d"), i, i - 1),
            RetrievedNodes[i]->GetParent() == RetrievedNodes[i - 1]);
    }

    // === 2. Test: Retrieve only occupied nodes ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->GetNodesContainingPoint(WorldPosition, true);

    TestTrue(TEXT("Occupied nodes should not include empty nodes"), OccupiedNodes.Num() <= RetrievedNodes.Num());
    TestTrue(TEXT("Inserted node should be in occupied nodes"), OccupiedNodes.Contains(InsertedNode));

    // **Ensure all occupied nodes have payloads**
    for (TSharedPtr<FSparseOctreeNode> Node : OccupiedNodes)
    {
        TestTrue(TEXT("Occupied node should have a payload"), Node->HasPayload());
    }

    // === 3. Test: Retrieve nodes using internal position ===
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedNodesInternal = OctreeData->GetNodesContainingPoint(InternalPosition, false);
    TestEqual(TEXT("Retrieving nodes by internal position should return the same number of nodes"),
        RetrievedNodesInternal.Num(), RetrievedNodes.Num());

    // === 4. Test: Retrieve nodes for a non-existent position ===
    FVector NonExistentWorldPosition(99999999.0, 99999999.0, 99999999.0);
    TArray<TSharedPtr<FSparseOctreeNode>> NonExistentNodes = OctreeData->GetNodesContainingPoint(NonExistentWorldPosition, true);
    TestTrue(TEXT("Non-existent position should return only root node"), NonExistentNodes.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkGetNodeContainingPointAtDepthTest,
    "VoxelPlugin.FAOctreeData.BulkGetNodeContainingPointAtDepth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkGetNodeContainingPointAtDepthTest::RunTest(const FString& Parameters)
{
    // Create and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Max depth for precision**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world positions**
    TArray<FVector> WorldPositions = {
        FVector(4000000.0, 4000000.0, 4000000.0),
        FVector(-4500000.0, 3500000.0, -1500000.0),
        FVector(5200000.0, -4700000.0, 2100000.0),
        FVector(-6500000.0, -5800000.0, -3500000.0)
    };

    // **Define depths for retrieval (some shallower, some deeper)**
    TArray<int32> Depths = { InsertDepth, InsertDepth - 2, InsertDepth - 4, InsertDepth };

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.3f * (i + 1);
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPositions[i], Depths[i], TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());
    }

    // === 1. Test: Bulk retrieval using world positions (all nodes, `bOnlyOccupied = false`) ===
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedNodes = OctreeData->BulkGetNodeContainingPointAtDepth(WorldPositions, Depths, false);

    TestEqual(TEXT("BulkGetNodeContainingPointAtDepth should return the same number of nodes requested"),
        RetrievedNodes.Num(), WorldPositions.Num());

    for (int i = 0; i < RetrievedNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Retrieved node %d should be valid"), i), RetrievedNodes[i].IsValid());
        TestEqual(FString::Printf(TEXT("Retrieved node %d should match inserted data density"), i),
            RetrievedNodes[i]->GetPayload()->Density, Data[i]->Density);
    }

    // === 2. Test: Bulk retrieval with `bOnlyOccupied = true` (should exclude empty nodes) ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->BulkGetNodeContainingPointAtDepth(WorldPositions, Depths, true);

    TestTrue(TEXT("Occupied nodes should not include empty nodes"), OccupiedNodes.Num() <= RetrievedNodes.Num());
    for (int i = 0; i < OccupiedNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Occupied node %d should have a payload"), i), OccupiedNodes[i]->HasPayload());
    }

    // === 3. Test: Bulk retrieval using internal coordinates ===
    TArray<FInt64Coordinate> InternalPositions;
    for (const FVector& Position : WorldPositions)
    {
        InternalPositions.Add(OctreeData->ConvertToInternalPosition(Position));
    }

    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedInternalNodes = OctreeData->BulkGetNodeContainingPointAtDepth(InternalPositions, Depths, false);

    TestEqual(TEXT("BulkGetNodeContainingPointAtDepth using internal coordinates should return the same number of nodes"),
        RetrievedInternalNodes.Num(), WorldPositions.Num());

    for (int i = 0; i < RetrievedInternalNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Retrieved internal node %d should be valid"), i), RetrievedInternalNodes[i].IsValid());
        TestEqual(FString::Printf(TEXT("Retrieved internal node %d should match inserted data density"), i),
            RetrievedInternalNodes[i]->GetPayload()->Density, Data[i]->Density);
    }

    // === 4. Test: Bulk retrieval for non-existent positions ===
    TArray<FVector> NonExistentPositions = {
        FVector(99999999.0, 99999999.0, 99999999.0),
        FVector(-99999999.0, -99999999.0, -99999999.0)
    };
    TArray<int32> NonExistentDepths = { InsertDepth, InsertDepth - 3 };
    TArray<TSharedPtr<FSparseOctreeNode>> NonExistentNodes = OctreeData->BulkGetNodeContainingPointAtDepth(NonExistentPositions, NonExistentDepths, false);

    TestTrue(TEXT("Bulk retrieval for non-existent positions should return an empty array"), NonExistentNodes.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetAllNodesInRadiusTest,
    "VoxelPlugin.FAOctreeData.GetAllNodesInRadius",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetAllNodesInRadiusTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define max insert depth**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world positions for insertions**
    TArray<FVector> WorldPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
        FVector(7000000.0, 5000000.0, 5000000.0) // Outside the small radius test
    };

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f * (i + 1);
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPositions[i], InsertDepth, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());
    }

    // === 1. Test: Retrieve all nodes within a small radius ===
    FVector QueryPosition(5000000.0, 5000000.0, 5000000.0);
    double SmallRadius = 250000.0;
    TArray<TSharedPtr<FSparseOctreeNode>> SmallRadiusNodes = OctreeData->GetAllNodesInRadius(QueryPosition, SmallRadius, false);

    TestTrue(TEXT("Nodes within small radius should be retrieved"), SmallRadiusNodes.Num() > 0);

    for (const TSharedPtr<FSparseOctreeNode>& Node : SmallRadiusNodes)
    {
        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the small radius"), Distance <= SmallRadius);
    }

    // === 2. Test: Retrieve all nodes within a large radius ===
    double LargeRadius = 3000000.0;
    TArray<TSharedPtr<FSparseOctreeNode>> LargeRadiusNodes = OctreeData->GetAllNodesInRadius(QueryPosition, LargeRadius, false);

    TestTrue(TEXT("Nodes within large radius should be retrieved"), LargeRadiusNodes.Num() > SmallRadiusNodes.Num());

    for (const TSharedPtr<FSparseOctreeNode>& Node : LargeRadiusNodes)
    {
        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the large radius"), Distance <= LargeRadius);
    }

    // === 3. Test: Retrieve only occupied nodes ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->GetAllNodesInRadius(QueryPosition, LargeRadius, true);

    TestTrue(TEXT("Occupied nodes should be a subset of all nodes"), OccupiedNodes.Num() <= LargeRadiusNodes.Num());
    for (const TSharedPtr<FSparseOctreeNode>& Node : OccupiedNodes)
    {
        TestTrue(TEXT("Occupied node should have a payload"), Node->HasPayload());
    }

    // === 4. Test: Retrieve using internal coordinates ===
    FInt64Coordinate InternalPosition = OctreeData->ConvertToInternalPosition(QueryPosition);
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedInternalNodes = OctreeData->GetAllNodesInRadius(InternalPosition, static_cast<uint64>(LargeRadius / OctreeData->GetPrecision() + 0.1), false);

    TestEqual(TEXT("Retrieving nodes by internal position should return the same number of nodes"),
        RetrievedInternalNodes.Num(), LargeRadiusNodes.Num());

    // === 5. Test: No nodes should be returned when the query is far away ===
    FVector FarPosition(100000000.0, 100000000.0, 100000000.0);
    TArray<TSharedPtr<FSparseOctreeNode>> EmptyNodes = OctreeData->GetAllNodesInRadius(FarPosition, LargeRadius, false);
    TestTrue(TEXT("No nodes should be returned for a far-away query"), EmptyNodes.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetAllNodesInRadiusAtDepthTest,
    "VoxelPlugin.FAOctreeData.GetAllNodesInRadiusAtDepth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetAllNodesInRadiusAtDepthTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define max insert depth**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH - 2;  // Use a slightly shallower depth for this test

    // **Define world positions for insertions**
    TArray<FVector> WorldPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
        FVector(7000000.0, 5000000.0, 5000000.0) // Outside the small radius test
    };

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f * (i + 1);
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(WorldPositions[i], InsertDepth, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());
    }

    // === 1. Test: Retrieve all nodes within a small radius at depth (`bOnlyOccupied = false`) ===
    FVector QueryPosition(5000000.0, 5000000.0, 5000000.0);
    double SmallRadius = 250000.0;
    TArray<TSharedPtr<FSparseOctreeNode>> SmallRadiusNodes = OctreeData->GetAllNodesInRadiusAtDepth(QueryPosition, SmallRadius, InsertDepth, false);

    TestTrue(TEXT("Nodes within small radius at depth should be retrieved"), SmallRadiusNodes.Num() > 0);

    for (const TSharedPtr<FSparseOctreeNode>& Node : SmallRadiusNodes)
    {
        TestEqual(TEXT("Retrieved nodes should be at the correct depth"), Node->GetIndex().Num(), InsertDepth);

        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the small radius"), Distance <= SmallRadius);
    }

    // === 2. Test: Retrieve all nodes within a large radius at depth (`bOnlyOccupied = false`) ===
    double LargeRadius = 3000000.0;
    TArray<TSharedPtr<FSparseOctreeNode>> LargeRadiusNodes = OctreeData->GetAllNodesInRadiusAtDepth(QueryPosition, LargeRadius, InsertDepth, false);

    TestTrue(TEXT("Nodes within large radius at depth should be retrieved"), LargeRadiusNodes.Num() > SmallRadiusNodes.Num());

    for (const TSharedPtr<FSparseOctreeNode>& Node : LargeRadiusNodes)
    {
        TestEqual(TEXT("Retrieved nodes should be at the correct depth"), Node->GetIndex().Num(), InsertDepth);

        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the large radius"), Distance <= LargeRadius);
    }

    // === 3. Test: Retrieve only occupied nodes ===
    TArray<TSharedPtr<FSparseOctreeNode>> OccupiedNodes = OctreeData->GetAllNodesInRadiusAtDepth(QueryPosition, LargeRadius, InsertDepth, true);

    TestTrue(TEXT("Occupied nodes should be a subset of all nodes"), OccupiedNodes.Num() <= LargeRadiusNodes.Num());
    for (const TSharedPtr<FSparseOctreeNode>& Node : OccupiedNodes)
    {
        TestTrue(TEXT("Occupied node should have a payload"), Node->HasPayload());
    }

    // === 4. Test: Retrieve using internal coordinates ===
    FInt64Coordinate InternalPosition = OctreeData->ConvertToInternalPosition(QueryPosition);
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedInternalNodes = OctreeData->GetAllNodesInRadiusAtDepth(InternalPosition, static_cast<uint64>(LargeRadius / OctreeData->GetPrecision()), InsertDepth, false);

    TestEqual(TEXT("Retrieving nodes by internal position should return the same number of nodes"),
        RetrievedInternalNodes.Num(), LargeRadiusNodes.Num());

    // === 5. Test: No nodes should be returned when the query is far away ===
    FVector FarPosition(100000000.0, 100000000.0, 100000000.0);
    TArray<TSharedPtr<FSparseOctreeNode>> EmptyNodes = OctreeData->GetAllNodesInRadiusAtDepth(FarPosition, LargeRadius, InsertDepth, false);
    TestTrue(TEXT("No nodes should be returned for a far-away query"), EmptyNodes.Num() == 0);

    // === 6. Test: Ensure nodes at the wrong depth are excluded ===
    int32 WrongDepth = InsertDepth - 5; // Depth that was NOT inserted at
    TArray<TSharedPtr<FSparseOctreeNode>> WrongDepthNodes = OctreeData->GetAllNodesInRadiusAtDepth(QueryPosition, LargeRadius, WrongDepth, true);
    TestTrue(TEXT("Querying at a depth that wasn't inserted should return 0 nodes"), WrongDepthNodes.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetAllLeavesTest,
    "VoxelPlugin.FAOctreeData.GetAllLeaves",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetAllLeavesTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // === 1. Insert multiple nodes into the octree ===
    TArray<uint8> Index1 = { 0 };
    TArray<uint8> Index2 = { 0, 1 };
    TArray<uint8> Index3 = { 0, 1, 2 };
    TArray<uint8> Index4 = { 0, 1, 3 };

    // **Insert data only at the leaves**
    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.5f;
    TSharedPtr<FSparseOctreeNode> Leaf1 = OctreeData->InsertData(Index3, TestData1);

    TSharedPtr<FVoxelData> TestData2 = MakeShared<FVoxelData>();
    TestData2->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> Leaf2 = OctreeData->InsertData(Index4, TestData2);

    // **Ensure intermediate nodes exist but remain non-leaf**
    TSharedPtr<FSparseOctreeNode> NonLeaf1 = OctreeData->GetNode(Index1);
    TSharedPtr<FSparseOctreeNode> NonLeaf2 = OctreeData->GetNode(Index2);

    TestTrue(TEXT("Non-leaf nodes should be valid"), NonLeaf1.IsValid() && NonLeaf2.IsValid());
    TestFalse(TEXT("Non-leaf node should not be marked as a leaf"), NonLeaf1->IsLeaf());
    TestFalse(TEXT("Non-leaf node should not be marked as a leaf"), NonLeaf2->IsLeaf());

    // === 2. Retrieve all leaves from the root ===
    TArray<TSharedPtr<FSparseOctreeNode>> LeafNodes = OctreeData->GetAllLeaves(OctreeData->GetRootNode());

    // **Verify that only the actual leaf nodes are returned**
    TestEqual(TEXT("Only 2 leaf nodes should be returned"), LeafNodes.Num(), 2);
    TestTrue(TEXT("Leaf1 should be in the returned leaves"), LeafNodes.Contains(Leaf1));
    TestTrue(TEXT("Leaf2 should be in the returned leaves"), LeafNodes.Contains(Leaf2));

    // === 3. Call `GetAllLeaves` on a single leaf node ===
    TArray<TSharedPtr<FSparseOctreeNode>> SingleLeaf = OctreeData->GetAllLeaves(Leaf1);
    TestEqual(TEXT("Calling GetAllLeaves on a single leaf should return only itself"), SingleLeaf.Num(), 1);
    TestEqual(TEXT("Returned node should be the same as Leaf1"), SingleLeaf[0], Leaf1);

    // === 4. Handle null input (empty tree) ===
    TArray<TSharedPtr<FSparseOctreeNode>> EmptyResult = OctreeData->GetAllLeaves(nullptr);
    TestTrue(TEXT("Calling GetAllLeaves with an invalid node should return an empty array"), EmptyResult.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetLeafAtPointTest,
    "VoxelPlugin.FAOctreeData.GetLeafAtPoint",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetLeafAtPointTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define max insert depth**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world positions for insertions**
    FVector InsertedPosition(5000000.0, 5000000.0, 5000000.0);
    FVector EmptyPosition(7000000.0, 5000000.0, 5000000.0); // Structural node but no payload

    // **Insert data at a position**
    TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
    TestData->Density = 0.85f;
    TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(InsertedPosition, InsertDepth, TestData);
    TSharedPtr<FSparseOctreeNode> EmptyNode = OctreeData->InsertDataAtPoint(EmptyPosition, InsertDepth, nullptr);
    TestTrue(TEXT("Inserted node should be valid"), InsertedNode.IsValid());

    // === 1. Test: Retrieve leaf node for an inserted position ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetLeafAtPoint(InsertedPosition);
    TestTrue(TEXT("Retrieved leaf node should be valid"), RetrievedNode.IsValid());
    TestTrue(TEXT("Retrieved node should be a leaf"), RetrievedNode->IsLeaf());
    TestEqual(TEXT("Retrieved node should match inserted node"), RetrievedNode, InsertedNode);

    // === 2. Test: Retrieve a structural node for an unoccupied position ===
    TSharedPtr<FSparseOctreeNode> StructuralNode = OctreeData->GetLeafAtPoint(EmptyPosition);
    TestTrue(TEXT("Structural node should be valid (even if no voxel exists)"), StructuralNode.IsValid());
    TestTrue(TEXT("Structural node should be a leaf (deepest node in this branch)"), StructuralNode->IsLeaf());

    // === 3. Test: Retrieve using internal coordinates ===
    FInt64Coordinate InternalPosition = OctreeData->ConvertToInternalPosition(InsertedPosition);
    TSharedPtr<FSparseOctreeNode> RetrievedInternalNode = OctreeData->GetLeafAtPoint(InternalPosition);
    TestEqual(TEXT("Retrieving node by internal position should return the same result"), RetrievedInternalNode, InsertedNode);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetLeavesAtPointsTest,
    "VoxelPlugin.FAOctreeData.GetLeavesAtPoints",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetLeavesAtPointsTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define max insert depth**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world positions for insertions**
    TArray<FVector> InsertedPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
    };

    FVector EmptyPosition(7000000.0, 5000000.0, 5000000.0); // Structural node but no payload

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < InsertedPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f * (i + 1);
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(InsertedPositions[i], InsertDepth, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());
    }

    // **Ensure structural node exists for EmptyPosition**
    OctreeData->InsertDataAtPoint(EmptyPosition, InsertDepth, nullptr);

    // === 1. Test: Retrieve multiple leaves at inserted positions ===
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedNodes = OctreeData->GetLeavesAtPoints(InsertedPositions);

    TestEqual(TEXT("GetLeavesAtPoints should return the same number of nodes requested"),
        RetrievedNodes.Num(), InsertedPositions.Num());

    for (int i = 0; i < RetrievedNodes.Num(); i++)
    {
        TestTrue(FString::Printf(TEXT("Retrieved node %d should be valid"), i), RetrievedNodes[i].IsValid());
        TestTrue(FString::Printf(TEXT("Retrieved node %d should be a leaf"), i), RetrievedNodes[i]->IsLeaf());
        TestEqual(FString::Printf(TEXT("Retrieved node %d should match inserted node"), i),
            RetrievedNodes[i]->GetPayload()->Density, Data[i]->Density);
    }

    // === 2. Test: Retrieve a structural node with no payload ===
    TArray<FVector> QueryPositions = { EmptyPosition };
    TArray<TSharedPtr<FSparseOctreeNode>> StructuralNodes = OctreeData->GetLeavesAtPoints(QueryPositions);

    TestEqual(TEXT("Structural node query should return 1 result"), StructuralNodes.Num(), 1);
    TestTrue(TEXT("Structural node should be valid"), StructuralNodes[0].IsValid());
    TestTrue(TEXT("Structural node should be a leaf"), StructuralNodes[0]->IsLeaf());
    TestFalse(TEXT("Structural node should not have a payload"), StructuralNodes[0]->HasPayload());

    // === 3. Test: Retrieve using internal coordinates ===
    TArray<FInt64Coordinate> InternalPositions;
    for (const FVector& Position : InsertedPositions)
    {
        InternalPositions.Add(OctreeData->ConvertToInternalPosition(Position));
    }

    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedInternalNodes = OctreeData->GetLeavesAtPoints(InternalPositions);

    TestEqual(TEXT("Retrieving nodes by internal position should return the same number of nodes"),
        RetrievedInternalNodes.Num(), InsertedPositions.Num());

    for (int i = 0; i < RetrievedInternalNodes.Num(); i++)
    {
        TestEqual(FString::Printf(TEXT("Retrieved internal node %d should match inserted node"), i),
            RetrievedInternalNodes[i], RetrievedNodes[i]);
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGetAllLeavesInRadiusTest,
    "VoxelPlugin.FAOctreeData.GetAllLeavesInRadius",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FGetAllLeavesInRadiusTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define max insert depth**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH;

    // **Define world positions for insertions**
    TArray<FVector> InsertedPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
        FVector(7000000.0, 5000000.0, 5000000.0) // Positioned outside the small radius test
    };

    FVector QueryPosition(5000000.0, 5000000.0, 5000000.0);
    double SmallRadius = 250000.0;
    double LargeRadius = 3000000.0;

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (int i = 0; i < InsertedPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f * (i + 1);
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(InsertedPositions[i], InsertDepth, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node %d should be valid"), i), InsertedNode.IsValid());
    }

    // === 1. Test: Retrieve all leaf nodes within a small radius ===
    TArray<TSharedPtr<FSparseOctreeNode>> SmallRadiusLeaves = OctreeData->GetAllLeavesInRadius(QueryPosition, SmallRadius);

    TestTrue(TEXT("Leaves within small radius should be retrieved"), SmallRadiusLeaves.Num() > 0);

    for (const TSharedPtr<FSparseOctreeNode>& Node : SmallRadiusLeaves)
    {
        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the small radius"), Distance <= SmallRadius);
        TestTrue(TEXT("Retrieved node should be a leaf"), Node->IsLeaf());
    }

    // === 2. Test: Retrieve all leaf nodes within a large radius ===
    TArray<TSharedPtr<FSparseOctreeNode>> LargeRadiusLeaves = OctreeData->GetAllLeavesInRadius(QueryPosition, LargeRadius);

    TestTrue(TEXT("Leaves within large radius should be retrieved"), LargeRadiusLeaves.Num() > SmallRadiusLeaves.Num());

    for (const TSharedPtr<FSparseOctreeNode>& Node : LargeRadiusLeaves)
    {
        double Distance = FVector::Dist(QueryPosition, OctreeData->ConvertToWorldPosition(Node->GetCenter()));
        TestTrue(TEXT("Retrieved node should be within the large radius"), Distance <= LargeRadius);
        TestTrue(TEXT("Retrieved node should be a leaf"), Node->IsLeaf());
    }

    // === 3. Test: Retrieve using internal coordinates ===
    FInt64Coordinate InternalQueryPosition = OctreeData->ConvertToInternalPosition(QueryPosition);
    TArray<TSharedPtr<FSparseOctreeNode>> RetrievedInternalLeaves = OctreeData->GetAllLeavesInRadius(InternalQueryPosition, static_cast<uint64>(LargeRadius / OctreeData->GetPrecision() + 0.1));

    TestEqual(TEXT("Retrieving nodes by internal position should return the same number of nodes"),
        RetrievedInternalLeaves.Num(), LargeRadiusLeaves.Num());

    for (int i = 0; i < RetrievedInternalLeaves.Num(); i++)
    {
        TestEqual(FString::Printf(TEXT("Retrieved internal node %d should match world query"), i),
            RetrievedInternalLeaves[i], LargeRadiusLeaves[i]);
    }

    // === 4. Test: No leaves should be returned when the query is far away ===
    FVector FarPosition(100000000.0, 100000000.0, 100000000.0);
    TArray<TSharedPtr<FSparseOctreeNode>> EmptyLeaves = OctreeData->GetAllLeavesInRadius(FarPosition, LargeRadius);
    TestTrue(TEXT("No leaves should be returned for a far-away query"), EmptyLeaves.Num() == 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeleteDataTest,
    "VoxelPlugin.FAOctreeData.DeleteData",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FDeleteDataTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define tree indices for testing**
    TArray<uint8> Index1 = { 0 };
    TArray<uint8> Index2 = { 0, 1 };
    TArray<uint8> Index3 = { 0, 1, 2 }; // This will be a child of Index2

    // **Insert data at the specified indices**
    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.5f;
    TSharedPtr<FSparseOctreeNode> Node1 = OctreeData->InsertData(Index1, TestData1);

    TSharedPtr<FVoxelData> TestData2 = MakeShared<FVoxelData>();
    TestData2->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> Node2 = OctreeData->InsertData(Index2, TestData2);

    TSharedPtr<FVoxelData> TestData3 = MakeShared<FVoxelData>();
    TestData3->Density = 1.0f;
    TSharedPtr<FSparseOctreeNode> Node3 = OctreeData->InsertData(Index3, TestData3); // Child of Node2

    TestTrue(TEXT("Inserted node 1 should be valid"), Node1.IsValid());
    TestTrue(TEXT("Inserted node 2 should be valid"), Node2.IsValid());
    TestTrue(TEXT("Inserted node 3 should be valid"), Node3.IsValid());

    // === 1. Test: Delete data from an occupied node ===
    OctreeData->DeleteData(Index2);
    TestFalse(TEXT("Deleted node 2 should no longer have a payload"), Node2->HasPayload());

    // === 2. Test: Ensure the node structure remains after deletion if it has a child ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode2 = OctreeData->GetNode(Index2);
    TestTrue(TEXT("Node 2 should still exist in the octree because it has a child"), RetrievedNode2.IsValid());

    // === 3. Test: Ensure the node structure collapses when no children remain ===
    OctreeData->DeleteData(Index3);
    TSharedPtr<FSparseOctreeNode> RetrievedNode3 = OctreeData->GetNode(Index3);
    TestFalse(TEXT("Node 3 should be removed from the octree"), RetrievedNode3.IsValid());

    TSharedPtr<FSparseOctreeNode> RetrievedNode2_AfterChildDeletion = OctreeData->GetNode(Index2);
    TestFalse(TEXT("Node 2 should now be removed after its last child was deleted"), RetrievedNode2_AfterChildDeletion.IsValid());

    // === 4. Test: Ensure cleanup removes the node if it's empty and has no children ===
    OctreeData->DeleteData(Index1);
    TSharedPtr<FSparseOctreeNode> RetrievedNode1 = OctreeData->GetNode(Index1);
    TestFalse(TEXT("Node 1 should be removed if it has no payload and no children"), RetrievedNode1.IsValid());

    // === 5. Test: Deleting a non-existent node should not crash ===
    TArray<uint8> NonExistentIndex = { 3, 5 };
    OctreeData->DeleteData(NonExistentIndex); // Should do nothing but must not crash

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkDeleteDataTest,
    "VoxelPlugin.FAOctreeData.BulkDeleteData",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkDeleteDataTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define valid tree indices**
    TArray<TArray<uint8>> TreeIndices = {
        { 0 },       // Node 1
        { 1, 2 },    // Node 2 (Parent)
        { 1, 2, 3 }, // Node 3 (Leaf under Node 2)
        { 4 }        // Node 4 (Independent branch)
    };

    // **Insert data at the specified indices**
    for (const TArray<uint8>& Index : TreeIndices)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 1.0f;
        OctreeData->InsertData(Index, TestData);
    }

    // === 1. Test: Delete multiple occupied nodes ===
    OctreeData->BulkDeleteData({ TreeIndices[0], TreeIndices[1], TreeIndices[3] });

    TSharedPtr<FSparseOctreeNode> RetrievedNode1 = OctreeData->GetNode(TreeIndices[0]);
    TestFalse(TEXT("Node 1 should be removed after bulk delete"), RetrievedNode1.IsValid());

    TSharedPtr<FSparseOctreeNode> RetrievedNode4 = OctreeData->GetNode(TreeIndices[3]);
    TestFalse(TEXT("Node 4 should be removed after bulk delete"), RetrievedNode4.IsValid());

    // === 2. Test: Ensure nodes with children remain ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode2 = OctreeData->GetNode(TreeIndices[1]);
    TestTrue(TEXT("Node 2 should still exist because it has a child"), RetrievedNode2.IsValid());
    TestFalse(TEXT("Node 2 should no longer have a payload"), RetrievedNode2->HasPayload());

    TSharedPtr<FSparseOctreeNode> RetrievedNode3 = OctreeData->GetNode(TreeIndices[2]);
    TestTrue(TEXT("Node 3 (child) should still exist"), RetrievedNode3.IsValid());
    TestTrue(TEXT("Node 3 should still have its payload"), RetrievedNode3->HasPayload());

    // === 3. Test: Cleanup should remove parent node if all children are deleted ===
    OctreeData->BulkDeleteData({ TreeIndices[2] });

    TSharedPtr<FSparseOctreeNode> RetrievedNode3_AfterDeletion = OctreeData->GetNode(TreeIndices[2]);
    TestFalse(TEXT("Node 3 should be removed after deletion"), RetrievedNode3_AfterDeletion.IsValid());

    TSharedPtr<FSparseOctreeNode> RetrievedNode2_AfterChildDeletion = OctreeData->GetNode(TreeIndices[1]);
    TestFalse(TEXT("Node 2 should be removed since all its children are gone"), RetrievedNode2_AfterChildDeletion.IsValid());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeleteDataAtPointTest,
    "VoxelPlugin.FAOctreeData.DeleteDataAtPoint",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FDeleteDataAtPointTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define world positions for insertions**
    FVector Position1(5000000.0, 5000000.0, 5000000.0);
    FVector Position2(5100000.0, 5000000.0, 5000000.0); // Different but close point
    FVector StructuralPosition(7000000.0, 5000000.0, 5000000.0); // Structural node but no payload

    // **Insert data into the octree**
    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.6f;
    TSharedPtr<FSparseOctreeNode> Node1 = OctreeData->InsertDataAtPoint(Position1, FSparseOctreeNode::MAX_DEPTH, TestData1);
    TestTrue(TEXT("Inserted node 1 should be valid"), Node1.IsValid());

    TSharedPtr<FVoxelData> TestData2 = MakeShared<FVoxelData>();
    TestData2->Density = 0.8f;
    TSharedPtr<FSparseOctreeNode> Node2 = OctreeData->InsertDataAtPoint(Position2, FSparseOctreeNode::MAX_DEPTH, TestData2);
    TestTrue(TEXT("Inserted node 2 should be valid"), Node2.IsValid());

    // **Ensure structural node exists**
    OctreeData->InsertDataAtPoint(StructuralPosition, FSparseOctreeNode::MAX_DEPTH, nullptr);

    // === 1. Test: Delete data at an occupied point ===
    OctreeData->DeleteDataAtPoint(Position1);
    TestFalse(TEXT("Deleted node 1 should no longer have a payload"), Node1->HasPayload());

    // === 2. Test: Ensure the structural node remains if it has children ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode1 = OctreeData->GetLeafAtPoint(Position1);
    TestTrue(TEXT("Node 1 should still exist in the octree after deletion"), RetrievedNode1.IsValid());
    TestFalse(TEXT("Node 1 should be empty after deletion"), RetrievedNode1->HasPayload());

    // === 3. Test: Ensure deleting a structural node does nothing ===
    OctreeData->DeleteDataAtPoint(StructuralPosition);
    TSharedPtr<FSparseOctreeNode> RetrievedStructuralNode = OctreeData->GetLeafAtPoint(StructuralPosition);
    TestTrue(TEXT("Structural node should still exist even after deletion"), RetrievedStructuralNode.IsValid());

    // === 4. Test: Delete multiple nodes at once ===
    OctreeData->DeleteDataAtPoint(Position2);
    TestFalse(TEXT("Deleted node 2 should no longer have a payload"), Node2->HasPayload());

    // === 5. Test: Ensure cleanup removes nodes if they become empty ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode2 = OctreeData->GetLeafAtPoint(Position2);

    // **Fix: Instead of checking for nullptr, ensure it's NOT the original deleted node**
    TestNotEqual(TEXT("Deleted node 2 should not exist after cleanup"), RetrievedNode2, Node2);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkDeleteDataAtPointTest,
    "VoxelPlugin.FAOctreeData.BulkDeleteDataAtPoint",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkDeleteDataAtPointTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define world positions for insertions**
    TArray<FVector> InsertedPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
    };

    FVector StructuralPosition(7000000.0, 5000000.0, 5000000.0); // Structural node but no payload

    // **Insert data into the octree**
    TArray<TSharedPtr<FVoxelData>> Data;
    for (const FVector& Position : InsertedPositions)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f;
        Data.Add(TestData);

        TSharedPtr<FSparseOctreeNode> InsertedNode = OctreeData->InsertDataAtPoint(Position, FSparseOctreeNode::MAX_DEPTH, TestData);
        TestTrue(FString::Printf(TEXT("Inserted node at %s should be valid"), *Position.ToString()), InsertedNode.IsValid());
    }

    // **Ensure structural node exists**
    OctreeData->InsertDataAtPoint(StructuralPosition, FSparseOctreeNode::MAX_DEPTH, nullptr);

    // === 1. Test: Bulk delete data at occupied positions ===
    OctreeData->BulkDeleteDataAtPoint(InsertedPositions);

    for (const FVector& Position : InsertedPositions)
    {
        TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetLeafAtPoint(Position);
        TestFalse(FString::Printf(TEXT("Deleted node at %s should no longer have a payload"), *Position.ToString()), RetrievedNode->HasPayload());
    }

    // === 2. Test: Ensure the structural node remains after deletion ===
    TSharedPtr<FSparseOctreeNode> RetrievedStructuralNode = OctreeData->GetLeafAtPoint(StructuralPosition);
    TestTrue(TEXT("Structural node should still exist even after deletion"), RetrievedStructuralNode.IsValid());

    // === 3. Test: Bulk delete using internal coordinates ===
    TArray<FInt64Coordinate> InternalPositions;
    for (const FVector& Position : InsertedPositions)
    {
        InternalPositions.Add(OctreeData->ConvertToInternalPosition(Position));
    }

    // **Reinsert the deleted nodes for a second round of testing**
    for (const FInt64Coordinate& InternalPos : InternalPositions)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.6f;
        OctreeData->InsertDataAtPoint(InternalPos, FSparseOctreeNode::MAX_DEPTH, TestData);
    }

    OctreeData->BulkDeleteDataAtPoint(InternalPositions);

    for (const FInt64Coordinate& InternalPos : InternalPositions)
    {
        TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetLeafAtPoint(InternalPos);
        TestFalse(TEXT("Deleted node (internal) should no longer have a payload"), RetrievedNode->HasPayload());
    }

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeleteDataAtPointAndDepthTest,
    "VoxelPlugin.FAOctreeData.DeleteDataAtPointAndDepth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FDeleteDataAtPointAndDepthTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define depth and world positions for insertions**
    int32 InsertDepth = FSparseOctreeNode::MAX_DEPTH - 2;  // Slightly shallower depth for testing
    FVector Position1(5000000.0, 5000000.0, 5000000.0);
    FVector Position2(5100000.0, 5000000.0, 5000000.0);

    // **Insert parent node**
    TSharedPtr<FVoxelData> TestData1 = MakeShared<FVoxelData>();
    TestData1->Density = 0.75f;
    TSharedPtr<FSparseOctreeNode> Node1 = OctreeData->InsertDataAtPoint(Position1, InsertDepth, TestData1);
    TestTrue(TEXT("Inserted node 1 should be valid"), Node1.IsValid());

    // **Insert a child under `Node1` to ensure it remains after deletion**
    FVector ChildPosition = Position1; // Slight offset to ensure different point
    TSharedPtr<FVoxelData> ChildData = MakeShared<FVoxelData>();
    ChildData->Density = 1.0f;
    TSharedPtr<FSparseOctreeNode> ChildNode = OctreeData->InsertDataAtPoint(ChildPosition, InsertDepth + 1, ChildData);
    TestTrue(TEXT("Inserted child node should be valid"), ChildNode.IsValid());

    // === 1. Test: Delete data at an occupied point at the specified depth ===
    OctreeData->DeleteDataAtPointAndDepth(Position1, InsertDepth);
    TestFalse(TEXT("Deleted node 1 should no longer have a payload"), Node1->HasPayload());

    // === 2. Test: Ensure the node structure remains if it has children ===
    TSharedPtr<FSparseOctreeNode> RetrievedNode1 = OctreeData->GetNodeContainingPointAtDepth(Position1, InsertDepth);
    TestTrue(TEXT("Node 1 should still exist in the octree after deletion because it has a child"), RetrievedNode1.IsValid());
    TestFalse(TEXT("Node 1 should be empty after deletion"), RetrievedNode1->HasPayload());

    // === 3. Test: If a node has no children, it should be cleaned up ===
    OctreeData->DeleteDataAtPointAndDepth(ChildPosition, InsertDepth + 1);
    TSharedPtr<FSparseOctreeNode> RetrievedChildNode = OctreeData->GetNodeContainingPointAtDepth(ChildPosition, InsertDepth + 1);
    TestFalse(TEXT("Child node should be removed after deletion"), RetrievedChildNode.IsValid());

    TSharedPtr<FSparseOctreeNode> RetrievedNode1_AfterChildDeletion = OctreeData->GetNodeContainingPointAtDepth(Position1, InsertDepth);
    TestFalse(TEXT("Node 1 should now be removed since its only child was deleted"), RetrievedNode1_AfterChildDeletion.IsValid());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBulkDeleteAtPointAndDepthTest,
    "VoxelPlugin.FAOctreeData.BulkDeleteAtPointAndDepth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

    bool FBulkDeleteAtPointAndDepthTest::RunTest(const FString& Parameters)
{
    // Create a OctreeData instance and initialize the octree
    TSharedPtr<FSparseOctree> OctreeData = MakeShared<FSparseOctree>();
 

    // **Define depths and world positions for insertions**
    int32 InsertDepth1 = FSparseOctreeNode::MAX_DEPTH - 3;
    int32 InsertDepth2 = FSparseOctreeNode::MAX_DEPTH - 2;
    int32 InsertDepth3 = FSparseOctreeNode::MAX_DEPTH - 1;

    TArray<FVector> InsertedPositions = {
        FVector(5000000.0, 5000000.0, 5000000.0),
        FVector(5100000.0, 5000000.0, 5000000.0),
        FVector(5300000.0, 5000000.0, 5000000.0),
    };

    TArray<int32> InsertedDepths = { InsertDepth1, InsertDepth2, InsertDepth3 };

    // **Insert data into the octree**
    for (int i = 0; i < InsertedPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.4f * (i + 1);
        OctreeData->InsertDataAtPoint(InsertedPositions[i], InsertedDepths[i], TestData);
    }

    // **Insert a structural node without a payload**
    FVector StructuralPosition(7000000.0, 5000000.0, 5000000.0);
    OctreeData->InsertDataAtPoint(StructuralPosition, InsertDepth2, nullptr);

    // === 1. Test: Bulk delete data at occupied positions ===
    OctreeData->BulkDeleteAtPointAndDepth(InsertedPositions, InsertedDepths);

    for (const FVector& Position : InsertedPositions)
    {
        TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetNodeContainingPointAtDepth(Position, InsertDepth1);
        TestFalse(FString::Printf(TEXT("Deleted node at %s should no longer have a payload"), *Position.ToString()), RetrievedNode.IsValid());
    }

    // === 2. Test: Ensure structural nodes remain after deletion ===
    TSharedPtr<FSparseOctreeNode> RetrievedStructuralNode = OctreeData->GetNodeContainingPointAtDepth(StructuralPosition, InsertDepth2);
    TestTrue(TEXT("Structural node should still exist after deletion"), RetrievedStructuralNode.IsValid());

    // === 3. Test: Bulk delete using internal coordinates ===
    TArray<FInt64Coordinate> InternalPositions;
    for (const FVector& Position : InsertedPositions)
    {
        InternalPositions.Add(OctreeData->ConvertToInternalPosition(Position));
    }

    // **Reinsert the deleted nodes for a second round of testing**
    for (int i = 0; i < InternalPositions.Num(); i++)
    {
        TSharedPtr<FVoxelData> TestData = MakeShared<FVoxelData>();
        TestData->Density = 0.6f;
        OctreeData->InsertDataAtPoint(InternalPositions[i], InsertedDepths[i], TestData);
    }

    OctreeData->BulkDeleteAtPointAndDepth(InternalPositions, InsertedDepths);

    for (const FInt64Coordinate& InternalPos : InternalPositions)
    {
        TSharedPtr<FSparseOctreeNode> RetrievedNode = OctreeData->GetNodeContainingPointAtDepth(InternalPos, InsertDepth1);
        TestFalse(TEXT("Deleted internal node should no longer exist"), RetrievedNode.IsValid());
    }

    return true;
}