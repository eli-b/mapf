using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace mapf
{
    /// <summary>
    /// This class represents a union of Dictionaries, which each map <typeparamref name="K"/>
    /// to a List of <typeparamref name="V"/>.
    /// It is similar to Python's collections.ChainMap, but 1) read-only, 2) a combined list of V
    /// elements is returned instead of just the first match.
    /// </summary>
    [DebuggerDisplay("count =  {Count}")]
    [Serializable]
    public class Dictionary_U<K, V> : IReadOnlyDictionary<K, List<V>>
    {
        List<IReadOnlyDictionary<K, List<V>>> Data;

        public Dictionary_U()
        {
            this.Data = new List<IReadOnlyDictionary<K, List<V>>>();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            foreach (IReadOnlyDictionary<K, List<V>> dict in Data)
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

        /// <summary>
        /// Gets an enumerable collection that contains the keys in the read-only dictionary.
        /// Warning: May contain duplicates if multiple inner dicts have the same key.
        /// </summary>
        /// <returns>An enumerable collection that contains the keys in the read-only dictionary.</returns>
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

        /// <summary>
        /// Gets an enumerable collection that contains the values in the read-only dictionary.
        /// </summary>
        /// <returns>
        /// An enumerable collection that contains the values in the read-only dictionary.
        /// </returns>
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

        /// <summary>
        /// Gets a combined list of the values mapped to the specified key in the combined dictionaries.
        /// </summary>
        /// <param name="key">The key to locate.</param>
        /// <returns>A combined list of the values mapped to the specified key in combined dictionaries.</returns>
        /// <exception cref="System.ArgumentNullException">key is null.</exception>
        /// <exception cref="System.Collections.Generic.KeyNotFoundException">key is not found.</exception>
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

        /// <summary>
        /// Gets a combined list of values mapped to the specified key in the combined dictionaries, or null if none
        /// are found.
        /// </summary>
        /// <param name="key"></param>
        /// <param name="value"></param>
        /// <returns></returns>
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

        /// <summary>
        /// Returns whether the specified key exists in any of the combined dictionaries
        /// </summary>
        /// <param name="key">The key to look for</param>
        /// <returns></returns>
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
        /// Remove all dictionaries from the union
        /// </summary>
        public void Clear()
        {
            Data.Clear();
        }

        /// <summary>
        /// Add a dictionary to the union
        /// </summary>
        /// <param name="other">the dictionary to add</param>
        public void Join(IReadOnlyDictionary<K, List<V>> other)
        {
            Data.Add(other);
        }

        /// <summary>
        /// Remove a dictionary from the union
        /// </summary>
        /// <param name="other">the dictionary to add</param>
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
        /// Gets the number of keys in the Dictionary. 
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
