using System;
using System.Collections;


namespace CPF_experiment
{
    // <summary>
    // This class implements HashSet with object retrieval.
    // This class uses a Hashtable (HashMap) "<object, ArrayList<object>>".
    // If more than one object with the same hash is stored, the first to appear is used as a key.
    // If object retrieval based on an object that is Equal to it is useful, it means
    // your objects are implicitly different, which may indicate problematic design.
    // </summary>
    [Serializable]
    public class HashTable_C : Hashtable
    {
        public void Add(object value)
        {
            ArrayList chain;
            if (base.ContainsKey(value))
                ((ArrayList)base[value]).Add(value);
            else
            {
                chain = new ArrayList();
                chain.Add(value);
                base.Add(value, chain);
            }
        }

        // <summary>
        // Can be thought of as "ContainsSimilar", as we're clearly not justing using an object as a key to search for itself
        // <summary>
        public override bool Contains(object key)
        {
            return ContainsKey(key);
        }

        // <summary>
        // Can be thought of as "ContainsSimilar", as we're clearly not justing using an object as a key to search for itself
        // <summary>
        public override bool ContainsKey(object key)
        {
            if (base.ContainsKey(key) == false)
                return false;
            foreach (object compare in ((ArrayList)base[key]))
            {
                if (key.Equals(compare))
                    return true;
            }
            return false;
        }

        // The get method of this property assumes the HashTable_C was alraedy checked to contain the key.
        // Otherwise the commented trick needs to be used.
        public override object this[object key]
        {
            get
            {
                foreach (object compare in ((ArrayList)base[key]) /*?? new ArrayList(0)*/ )
                {
                    if (key.Equals(compare))
                        return compare;
                }
                return null;
            }
            set
            {
                ((ArrayList)base[key]).Add(value);
            }
        }

        public override void Remove(object key)
        {
            ArrayList arr = ((ArrayList)base[key]);
            if (arr == null)
                return;
            base.Remove(key);
            foreach (object item in arr)
            {
                if(item.Equals(key) == false)
                    this.Add(item);
            }
            arr.Clear();
        }
    }
}
