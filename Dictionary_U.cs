using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    // <summary>
    // This class represents a union of Dictionaries.
    // </summary>
    [Serializable]
    public class Dictionary_U<K, V> : IReadOnlyDictionary<K, List<V>>
    {
        List<Dictionary<K, List<V>>> Data;
        public Dictionary_U()
        {
            this.Data = new List<Dictionary<K, List<V>>>();
        }

        //public void Add(K value)
        //{
        //    throw new Exception("Illegal Operation");
        //}

        //public void CopyTo(K[] array, int arrayIndex)
        //{
        //    throw new Exception("Illegal Operation"); // Lazy...
        //}

        //public bool Remove(K item)
        //{
        //    throw new Exception("Illegal Operation");
        //}

        IEnumerator IEnumerable.GetEnumerator()
        {
            foreach (Dictionary<K, List<V>> dict in Data)
            {
                foreach (KeyValuePair<K, List<V>> item in dict)
                {
                    yield return item;
                }
            }
        }

        public IEnumerator<KeyValuePair<K, List<V>>> GetEnumerator()
        {
            foreach (Dictionary<K, List<V>> dict in Data)
            {
                foreach (KeyValuePair<K, List<V>> item in dict)
                {
                    yield return item;
                }
            }
        }

        // Summary:
        //     Gets an enumerable collection that contains the keys in the read-only dictionary.
        //
        // Returns:
        //     An enumerable collection that contains the keys in the read-only dictionary.
        public IEnumerable<K> Keys
        {
            get
            {
                foreach (Dictionary<K, List<V>> dict in Data)
                {
                    foreach (KeyValuePair<K, List<V>> item in dict)
                    {
                        yield return item.Key;
                    }
                }
            }
        }

        //
        // Summary:
        //     Gets an enumerable collection that contains the values in the read-only dictionary.
        //
        // Returns:
        //     An enumerable collection that contains the values in the read-only dictionary.
        public IEnumerable<List<V>> Values
        {
            get
            {
                foreach (Dictionary<K, List<V>> dict in Data)
                {
                    foreach (KeyValuePair<K, List<V>> item in dict)
                    {
                        yield return item.Value;
                    }
                }
            }
        }

        // Summary:
        //     Gets the element that has the specified key in the read-only dictionary.
        //
        // Parameters:
        //   key:
        //     The key to locate.
        //
        // Returns:
        //     The element that has the specified key in the read-only dictionary.
        //
        // Exceptions:
        //   System.ArgumentNullException:
        //     key is null.
        //
        //   System.Collections.Generic.KeyNotFoundException:
        //     The property is retrieved and key is not found.
        public List<V> this[K key]
        {
            get
            {
                var ret = new List<V>();

                foreach (Dictionary<K, List<V>> dict in Data)
                {
                    if (dict.ContainsKey(key))
                        ret.AddRange(dict[key]);
                }

                if (ret.Count == 0)
                    throw new KeyNotFoundException();
                return ret;
            }
        }

        public bool TryGetValue(K key, out List<V> value)
        {
            if (this.ContainsKey(key))
            {
                value = this[key];
                return true;
            }
            else
            {
                value = null;
                return false;
            }
        }

        public bool ContainsKey(K key)
        {
            foreach (Dictionary<K, List<V>> item in Data)
            {
                if (item.ContainsKey(key))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Hardly used.
        /// </summary>
        public void Clear()
        {
            foreach (Dictionary<K, List<V>> item in Data)
            {
                item.Clear();
            }
            Data.Clear();
        }

        public void Join(Dictionary<K, List<V>> other)
        {
            Data.Add(other);
        }

        public void Seperate(Dictionary<K, List<V>> other)
        {
            Data.Remove(other);
        }

        /// <summary>
        /// Not used.
        /// </summary>
        public void Print()
        {
            foreach (Dictionary<K, List<V>> dict in Data)
            {
                foreach (KeyValuePair<K, List<V>> item in dict)
                {
                    Console.WriteLine(item);
                }
            }
        }

        /// <summary>
        /// Gets the number of values in the Dictionary. 
        /// </summary>
        public int Count
        {
            get
            {
                return this.Data.Sum<Dictionary<K, List<V>>>(dict => dict.Count);
            }
        }

        //public bool IsReadOnly { get { return true; } } // Using Add() it's read-only
    }
}
