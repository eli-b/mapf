using System;
using System.Collections.Generic;
using System.Linq;

namespace mapf;

public class DisjointSetItem
{
    public DisjointSetItem parent;
    public int rank;

    public DisjointSetItem()
    {
        parent = this;
        rank = 0;
    }

    public bool IsSingle()
    {
        if (this.parent != this)
            return false;
        if (this.rank != 0) // Rank is only correct in representative items!
            return false;
        return true;
    }
}

/// <summary>
/// Algorithm from http://en.wikipedia.org/wiki/Disjoint-set_data_structure.
/// Operations cost O(Ackerman^-1(n)) time - practically constant. Cost is asymptotically optimal :)
/// </summary>
public class DisjointSets<T>
{
    protected IDictionary<T, DisjointSetItem> entriesToItems;
    public int maxRank { get; protected set; }

    public DisjointSets()
    {
        entriesToItems = new Dictionary<T, DisjointSetItem>();
        maxRank = 0;
    }

    /// <summary>
    /// Initially all entries are single item sets.
    /// </summary>
    /// <param name="entries"></param>
    public DisjointSets(ISet<T> entries)
    {
        entriesToItems = new Dictionary<T, DisjointSetItem>();
        maxRank = 0;
        foreach (var entry in entries)
        {
            entriesToItems[entry] = new DisjointSetItem();
        }
    }

    public DisjointSets(DisjointSets<T> other)
    {
        entriesToItems = new Dictionary<T, DisjointSetItem>();
        maxRank = other.maxRank;
        var queue = new Queue<T>(other.entriesToItems.Keys);
        var otherRepsToNewReps = new Dictionary<DisjointSetItem, DisjointSetItem>();
        while (queue.Count != 0)
        {
            var entry = queue.Dequeue();
            var item = other.entriesToItems[entry];
            other.Find(item); // Makes item.parent point directly do the set's representative
            if (item.parent == item) // This is the set's representative
            {
                var newItem = new DisjointSetItem();
                entriesToItems[entry] = newItem;
                // .parent already points to itself as needed
                newItem.rank = item.rank; // Since all items in a set do not point directly to the set's rep,
                                            // the set's rank would have been 1 regardless of its size, had it
                                            // been created by calls to Union. We prefer to remember the original rank.
                otherRepsToNewReps[item] = newItem;
            }
            else
            {
                if (otherRepsToNewReps.ContainsKey(item.parent)) // We've copied this rep already
                {
                    var newItem = new DisjointSetItem();
                    entriesToItems[entry] = newItem;
                    newItem.parent = otherRepsToNewReps[item.parent];
                    // .rank not important in non-rep items
                }
                else
                    queue.Enqueue(entry); // Wait for the rep to be copied first.
            }
        }
    }

    /// <summary>
    /// Does nothing if entry is already present.
    /// Returns whether the entry was really added now.
    /// </summary>
    public bool AddSet(T entry)
    {
        if (entriesToItems.ContainsKey(entry) == false)
        {
            entriesToItems[entry] = new DisjointSetItem();
            return true;
        }
        return false;
    }

    /// <summary>
    /// Assumes entry is already in the DisjointSets data structure
    /// </summary>
    /// <param name="entry"></param>
    /// <returns></returns>
    public bool IsSingle(T entry)
    {
        var item = entriesToItems[entry];
        return item.IsSingle();
    }

    public IEnumerable<ISet<T>> GetSets()
    {
        Dictionary<DisjointSetItem, ISet<T>> repsToSets = new Dictionary<DisjointSetItem, ISet<T>>();
        foreach (var entryAndSetItem in entriesToItems)
        {
            var entry = entryAndSetItem.Key;
            var item = entryAndSetItem.Value;
            var rep = Find(item);
            if (repsToSets.ContainsKey(rep) == false)
                repsToSets.Add(rep, new HashSet<T>());
            repsToSets[rep].Add(entry);
        }
        return repsToSets.Values;
    }

    public int GetNumOfSets()
    {
        int count = 0;
        foreach (var entryAndSetItem in entriesToItems)
        {
            var entry = entryAndSetItem.Key;
            var item = entryAndSetItem.Value;
            if (item.parent == item)
                count += 1;
        }
        return count;
    }

    public override string ToString()
    {
        string ans = "{";
        foreach (var set in GetSets())
        {
            ans += "{";
            foreach (var item in set)
            {
                ans += item.ToString() + ", ";
            }
            ans += "}, ";
        }
        ans += "}";
        return ans;
    }

    /// <summary>
    /// Modifies this data structure to also contains all unions from other.
    /// </summary>
    /// <param name="other"></param>
    /// <returns>Whether this object was changed</returns>
    public bool CopyUnions(DisjointSets<T> other)
    {
        // Create a temporary reverse mapping of other's items to its entries - O(n)
        var otherItemsToEntries = new Dictionary<DisjointSetItem, T>();
        foreach (var kvp in other.entriesToItems)
        {
            otherItemsToEntries[kvp.Value] = kvp.Key;
        }

        // Unite each of other's entries with its representative only - O(n)
        bool ret = false;
        foreach (var entryAndSetItem in other.entriesToItems)
        {
            var entry = entryAndSetItem.Key;
            var item = entryAndSetItem.Value;
            if (item.IsSingle() == true)
                continue;
            other.Find(item);
            var repEntry = otherItemsToEntries[item.parent];
            bool wasOnlyUnitedNow = this.Union(entry, repEntry);
            ret = ret || wasOnlyUnitedNow;
        }
        return ret;
    }

    /// <summary>
    /// Finds the representative of the item's set
    /// </summary>
    /// <param name="x"></param>
    /// <returns></returns>
    protected DisjointSetItem Find(DisjointSetItem x)
    {
        if (x.parent != x)
            x.parent = Find(x.parent);
        return x.parent;
    }

    /// <summary>
    /// Doesn't assume x, y already in the data structure.
    /// Returns whether they were really only united now.
    /// </summary>
    /// <param name="entryA"></param>
    /// <param name="entryB"></param>
    /// <returns></returns>
    public bool Union(T entryA, T entryB)
    {
        AddSet(entryA);
        AddSet(entryB);
        var x = entriesToItems[entryA];
        var y = entriesToItems[entryB];
        var xRoot = Find(x);
        var yRoot = Find(y);
        if (xRoot == yRoot)
            return false;

        // x and y are not already in same set. Merge them.
        if (xRoot.rank < yRoot.rank)
            xRoot.parent = yRoot;
        else if (xRoot.rank > yRoot.rank)
            yRoot.parent = xRoot;
        else
        {
            // Arbitrarily choose x to be the root of the union
            yRoot.parent = xRoot;
            xRoot.rank = xRoot.rank + 1;
            if (xRoot.rank > maxRank)
                maxRank = xRoot.rank;
        }
        return true;
    }

    /// <summary>
    /// Assumes entries are already in the DisjointSets data structure
    /// </summary>
    /// <param name="entryA"></param>
    /// <param name="entryB"></param>
    /// <returns></returns>
    public bool AreUnited(T entryA, T entryB)
    {
        var x = entriesToItems[entryA];
        var y = entriesToItems[entryB];
        var xRoot = Find(x);
        var yRoot = Find(y);
        return xRoot == yRoot;
    }

    public bool Contains(T entry)
    {
        return entriesToItems.ContainsKey(entry);
    }
}
