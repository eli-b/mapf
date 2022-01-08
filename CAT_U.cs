using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace mapf;

/// <summary>
/// This class represents a union of Dictionaries, which each map <typeparamref name="K"/>
/// to a List of <typeparamref name="V"/>.
/// It is similar to Python's collections.ChainMap, but 1) read-only, 2) a combined list of V
/// elements is returned instead of just the first match.
/// </summary>
[DebuggerDisplay("count = {Count}")]
[Serializable]
public class CAT_U : ConflictAvoidanceTable
{
    List<ConflictAvoidanceTable> Data;

    public CAT_U()
    {
        this.Data = new List<ConflictAvoidanceTable>();
    }

    /// <summary>
    /// Gets a combined list of the values mapped to the specified key in the combined dictionaries.
    /// </summary>
    /// <param name="key">The key to locate.</param>
    /// <returns>A combined list of the values mapped to the specified key in the combined dictionaries.</returns>
    /// <exception cref="System.ArgumentNullException">key is null.</exception>
    /// <exception cref="System.Collections.Generic.KeyNotFoundException">key is not found.</exception>
    public new IReadOnlyList<int> this[TimedMove key]
    {
        get
        {
            if (Data.Count > 1)
            {
                var ret = new List<int>();
                foreach (ConflictAvoidanceTable cat in Data)
                {
                    if (cat.ContainsKey(key))
                        ret.AddRange(cat[key]);
                }
                if (ret.Count == 0)
                    throw new KeyNotFoundException();
                return ret;
            }
            else
            {
                return Data.First()[key];
            }
        }
    }

    /// <summary>
    /// Returns whether the specified key exists in any of the combined dictionaries
    /// </summary>
    /// <param name="key">The key to look for</param>
    /// <returns></returns>
    public new bool ContainsKey(TimedMove key)
    {
        foreach (ConflictAvoidanceTable item in Data)
        {
            if (item.ContainsKey(key))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Remove all dictionaries from the union. Does not clear them.
    /// </summary>
    public new void Clear()
    {
        Data.Clear();
    }

    /// <summary>
    /// Add a dictionary to the union
    /// </summary>
    /// <param name="other">the dictionary to add</param>
    public void Join(ConflictAvoidanceTable other)
    {
        Data.Add(other);
    }

    /// <summary>
    /// Remove a dictionary from the union
    /// </summary>
    /// <param name="other">the dictionary to add</param>
    public void Separate(ConflictAvoidanceTable other)
    {
        Data.Remove(other);
    }

    /// <summary>
    /// Gets the number of keys in the Dictionary. 
    /// </summary>
    public int Count
    {
        get
        {
            return this.Data.Sum(cat => cat.NumPlans);
        }
    }
}
