using System;

namespace mapf;

/// <summary>
/// represents items that are stored in the binary heap
/// </summary>
public interface IBinaryHeapItem : IComparable<IBinaryHeapItem> 
{
    /// <summary>
    /// The index of the item in the binary heap
    /// </summary>
    int GetIndexInHeap();
    void SetIndexInHeap(int index);
}
