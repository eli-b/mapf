using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    // <summary>
    // This class represents a union of HashSets.
    // </summary>
    [Serializable]
    public class HashSet_U<T> : ICollection<T>
    {
        List<HashSet<T>> Data;
        public HashSet_U()
        {
            this.Data = new List<HashSet<T>>();
        }

        public void Add(T value)
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
            Data.Clear();
        }

        public void Join(HashSet<T> other)
        {
            Data.Add(other);
        }

        public void Seperate(HashSet<T> other)
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
                return this.Data.Sum<HashSet<T>>(set => set.Count);
            }
        }

        public bool IsReadOnly { get { return true; } } // Using Add() it's read-only
    }
}
