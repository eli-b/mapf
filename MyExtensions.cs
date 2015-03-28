using System;
using System.Collections.Generic;
using System.Linq;

namespace ExtensionMethods
{
    public static class MyExtensions
    {
        public static int IndexOfMax<T>(this IEnumerable<T> sequence)
            where T : IComparable<T>
        {
            T maxValue = default(T); // Immediately overwritten anyway
            int ans = 0;
            int index = 0;
            foreach (T value in sequence)
            {
                int compareResult = value.CompareTo(maxValue);
                if (compareResult > 0 || index == 0)
                {
                    ans = index;
                    maxValue = value;
                }
                index++;
            }
            return ans;
        }

        public static int IndexOfMax<T, R>(this IEnumerable<T> sequence, Func<T, R> mapping)
            where R : IComparable<R>
        {
            R maxValue = default(R); // Immediately overwritten anyway
            int ans = 0;
            int index = 0;
            foreach (T value in sequence)
            {
                R res = mapping(value);
                int compareResult = res.CompareTo(maxValue);
                if (compareResult > 0 || index == 0)
                {
                    ans = index;
                    maxValue = res;
                }
                index++;
            }
            return ans;
        }

        /// <summary>
        /// Like Python's max(x, key=func)
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <typeparam name="R"></typeparam>
        /// <param name="sequence"></param>
        /// <param name="mapping"></param>
        /// <returns></returns>
        public static T MaxByKeyFunc<T, R>(this IEnumerable<T> sequence, Func<T, R> mapping)
            where R : IComparable<R>
        {
            R maxValue = default(R); // Immediately overwritten anyway
            T ans = default(T);
            int index = 0;
            foreach (T value in sequence)
            {
                R res = mapping(value);
                int compareResult = res.CompareTo(maxValue);
                if (compareResult > 0 || index == 0)
                {
                    ans = value;
                    maxValue = res;
                }
                index++;
            }
            return ans;
        }
        
        /// <summary>
        /// Based on MaxIndex By Jon Skeet: http://stackoverflow.com/questions/462699/how-do-i-get-the-index-of-the-highest-value-in-an-array-using-linq
        /// </summary>
        /// <param name="sequence"></param>
        /// <returns></returns>
        public static IList<int> IndicesOfMax<T>(this IEnumerable<T> sequence)
            where T : IComparable<T>
        {
            var ans = new List<int>(4);
            T maxValue = default(T); // Immediately overwritten anyway

            int index = 0;
            foreach (T value in sequence)
            {
                int compareResult = value.CompareTo(maxValue);
                if (compareResult > 0 || ans.Count == 0)
                {
                    ans.Clear();
                    ans.Add(index);
                    maxValue = value;
                }
                if (compareResult == 0)
                    ans.Add(index);
                index++;
            }
            return ans;
        }

        public static IList<int> IndicesOfMax<T, R>(this IEnumerable<T> sequence, Func<T, R> mapping)
            where R : IComparable<R>
        {
            var ans = new List<int>(4);
            R maxRes = default(R); // Immediately overwritten anyway

            int index = 0;
            foreach (T value in sequence)
            {
                R res = mapping(value);
                int compareResult = res.CompareTo(maxRes);
                if (compareResult > 0 || ans.Count == 0)
                {
                    ans.Clear();
                    ans.Add(index);
                    maxRes = res;
                }
                if (compareResult == 0)
                    ans.Add(index);
                index++;
            }
            return ans;
        }
        
        public static IList<T> AllMax<T>(this IEnumerable<T> sequence)
            where T : IComparable<T>
        {
            var ans = new List<T>();
            T maxVal = default(T); // Immediately overwritten anyway

            foreach (T item in sequence)
            {
                int compareResult = item.CompareTo(maxVal);
                if (compareResult > 0 || ans.Count == 0)
                {
                    ans.Clear();
                    ans.Add(item);
                    maxVal = item;
                }
                if (compareResult == 0)
                    ans.Add(item);
            }
            return ans;
        }

        public static IList<T> AllMax<T, R>(this IEnumerable<T> sequence, Func<T, R> mapping)
            where R : IComparable<R>
        {
            var ans = new List<T>();
            R maxResult = default(R); // Immediately overwritten anyway

            foreach (T item in sequence)
            {
                R mappingResult = mapping(item);
                int compareResult = mappingResult.CompareTo(maxResult);
                if (compareResult > 0 || ans.Count == 0)
                {
                    ans.Clear();
                    ans.Add(item);
                    maxResult = mappingResult;
                }
                if (compareResult == 0)
                    ans.Add(item);
            }
            return ans;
        }

        public static IList<K> KeysOfMaxValue<K, V>(this IEnumerable<KeyValuePair<K, V>> mapping)
            where V : IComparable<V>
        {
            var ans = new List<K>();
            V maxValue = default(V); // Immediately overwritten anyway

            foreach (KeyValuePair<K, V> kvp in mapping)
            {
                int compareResult = kvp.Value.CompareTo(maxValue);
                if (compareResult > 0 || ans.Count == 0)
                {
                    ans.Clear();
                    ans.Add(kvp.Key);
                    maxValue = kvp.Value;
                }
                if (compareResult == 0)
                    ans.Add(kvp.Key);
            }
            return ans;
        }
    }
}
