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
        List<IReadOnlyDictionary<K, List<V>>> Data;
        public Dictionary_U()
        {
            this.Data = new List<IReadOnlyDictionary<K, List<V>>>();
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
            foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
            {
                foreach (KeyValuePair<K, List<V>> item in dict)
                {
                    yield return item;
                }
            }
        }

        /// Summary:
        ///     Gets an enumerable collection that contains the keys in the read-only dictionary.
        ///     Warning: May contain duplicates if multiple inner dicts have the same key.
        ///
        /// Returns:
        ///     An enumerable collection that contains the keys in the read-only dictionary.
        ///     
        public IEnumerable<K> Keys
        {
            get
            {
                foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
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
                foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
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

                foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
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
            foreach (IReadOnlyDictionary<K, List<V>> item in Data)
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

        public void Join(IReadOnlyDictionary<K, List<V>> other)
        {
            Data.Add(other);
        }

        public void Separate(IReadOnlyDictionary<K, List<V>> other)
        {
            Data.Remove(other);
        }

        /// <summary>
        /// Not used.
        /// </summary>
        public void Print()
        {
            foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
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
                return this.Data.Sum<IReadOnlyDictionary<K, List<V>>>(dict => dict.Count);
            }
        }
    }
}
