using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class DynamicArray<T>
    {
        T[] array;
        int count;

        public DynamicArray()
        {
            array = new T[1];
            count = 0;
        }

        public T this[int index]
        {
            get
            {
                if (index < array.Length)
                {
                    return array[index];
                }
                return default(T);
            }
            set
            {
                if (index >= array.Length)
                {
                    T[] bigger = new T[index + 5];
                    for (int i = 0; i < array.Length; i++)
                    {
                        bigger[i] = array[i];
                    }
                    array = bigger;
                }
                if (array[index] == null)
                    count++;
                array[index] = value;
            }
        }

        public void clear()
        {
            array = null;
        }

        public int getLength() { return array.Length; }

        public int getCount() { return count; }
    }
}
