using System.Collections;

namespace CPF_experiment
{
    /// <summary>
    /// simmilar to the given hashtable with the possability of chaining objects
    /// </summary>
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
        public override bool Contains(object key)
        {
            if (base[key] == null)
                return false;
            foreach (object compare in ((ArrayList)base[key]))
            {
                if (key.Equals(compare))
                    return true;
            }
            return false;
        }
        public override bool ContainsKey(object key)
        {
            if (base[key] == null)
                return false;
            foreach (object compare in ((ArrayList)base[key]))
            {
                if (key.Equals(compare))
                    return true;
            }
            return false;
        }
        public override object this[object key]
        {
            get
            {
                foreach (object compare in ((ArrayList)base[key]))
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
            ArrayList alocate = new ArrayList();
            foreach (object item in arr)
            {
                if(item.Equals(key) == false)
                    alocate.Add(item);
            }
            arr.Clear();
            base.Remove(key);
            foreach (object item in alocate)
            {
                this.Add(item);
            }
        }
    }
}
