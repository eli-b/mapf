using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    // <summary>
    // This class represents a union of HashSets.
    // TODO: Make sure it's really better than using HashSet.UnionWith().
    // </summary>
    public class HashSet_U<T>
    {
        List<HashSet<T>> Data;
        public HashSet_U()
        {
            this.Data = new List<HashSet<T>>();
        }

        public void Add(object value)
        {
            throw new Exception("Illegal Operation");
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

        public void print()
        {
            foreach (HashSet<T> set in Data)
            {
                foreach (T item in set)
                {
                    Console.WriteLine(item);
                }
            }
        }
    }
}
