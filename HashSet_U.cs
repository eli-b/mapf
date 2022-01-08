using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace mapf;

// <summary>
// This class represents a union of HashSets.
// </summary>
[DebuggerDisplay("count = {Count}")]
[Serializable]
public class HashSet_U<T> : ISet<T>
{
    List<ISet<T>> Data;
    public HashSet_U()
    {
        this.Data = new List<ISet<T>>();
    }

    bool ISet<T>.Add(T value)
    {
        throw new Exception("Illegal Operation");
    }

    void ICollection<T>.Add(T value)
    {
        throw new Exception("Illegal Operation");
    }

    public void CopyTo(T[] array, int arrayIndex)
    {
        throw new Exception("Illegal Operation"); // Lazy...
    }

    public bool Remove(T item)
    {
        throw new Exception("Illegal Operation");
    }

    public void ExceptWith(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public void IntersectWith(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool IsProperSubsetOf(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool IsProperSupersetOf(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool IsSubsetOf(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool IsSupersetOf(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool Overlaps(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public bool SetEquals(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public void SymmetricExceptWith(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public void UnionWith(IEnumerable<T> other)
    {
        throw new Exception("Illegal Operation");
    }

    public IEnumerator<T> GetEnumerator()
    {
        foreach (HashSet<T> set in Data)
        {
            foreach (T item in set)
            {
                yield return item;
            }
        }
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        foreach (HashSet<T> set in Data)
        {
            foreach (T item in set)
            {
                yield return item;
            }
        }
    }

    public bool Contains(T key)
    {
        foreach (HashSet<T> item in Data)
        {
            if (item.Contains(key))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Hardly used.
    /// </summary>
    public void Clear()
    {
        foreach (HashSet<T> item in Data)
        {
            item.Clear();
        }
        // FIXME: We claim below it's read-only. 
        Data.Clear();
    }

    public void Join(ISet<T> other)
    {
        Data.Add(other);
    }

    public void Separate(ISet<T> other)
    {
        Data.Remove(other);
    }

    /// <summary>
    /// Not used.
    /// </summary>
    public void Print()
    {
        foreach (HashSet<T> set in Data)
        {
            foreach (T item in set)
            {
                Console.WriteLine(item);
            }
        }
    }

    /// <summary>
    /// Gets the number of values in the HashSet. 
    /// </summary>
    public int Count
    {
        get { 
            return this.Data.Sum(set => set.Count);
        }
    }

    public bool IsReadOnly { get { return true; } } // Using Add() it's read-only
}
