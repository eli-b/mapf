using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    public class HashSet_U<T>
    {
        List<HashSet<T>> Data;
        public HashSet_U()
        {
            this.Data = new List<HashSet<T>>();
        }

        public void Add(object value)
        {
            throw new Exception("Ileagal Operation");
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
        public void  print()
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
