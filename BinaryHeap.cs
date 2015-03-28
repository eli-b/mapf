using System;
using System.Collections.Generic;
namespace CPF_experiment
{
    /// <summary>
    /// A binary heap, useful for sorting data and priority queues.
    /// </summary>
    public class BinaryHeap
    {
        // Constants
        private const int DEFAULT_SIZE = 4;
        public const int REMOVED_FROM_HEAP = -1;

        // Fields
        private IBinaryHeapItem[] _data;
        private int _count = 0;
        private int _capacity = DEFAULT_SIZE;
        private bool _sorted;

        // Constructors
        /// <summary>
        /// Creates a new binary heap.
        /// </summary>
        public BinaryHeap()
        {
            _data = new IBinaryHeapItem[DEFAULT_SIZE];
            // _capacity is already set to DEFAULT_SIZE
            // _count already set to 0
        }

        /// <summary>
        /// Creates a new binary heap with the specified initial capacity.
        /// </summary>
        public BinaryHeap(int capacity)
        {
            _data = new IBinaryHeapItem[capacity];
            _capacity = capacity;
            // _count already set to 0
        }

        /// <summary>
        /// Creates a new binary heap from the given array.
        /// </summary>
        public BinaryHeap(IBinaryHeapItem[] data, int count)
        {
            _count = count;
            _capacity = count;
            _data = new IBinaryHeapItem[_capacity];
            Array.Copy(data, _data, count);
        }

        /// <summary>
        /// Creates a new binary heap from the given collection.
        /// </summary>
        public BinaryHeap(IReadOnlyCollection<IBinaryHeapItem> from)
            : this(from.Count)
        {
            foreach (var item in from)
                this.Add(item);
        }

        // TODO: Support iterators!

        // Properties
        /// <summary>
        /// Gets the number of values in the heap. 
        /// </summary>
        public int Count
        {
            get { return _count; }
        }

        /// <summary>
        /// Gets or sets the capacity of the heap.
        /// Can only set it to values larger than the current capacity.
        /// </summary>
        public int Capacity
        {
            get { return _capacity; }
            set
            {
                if (value > _capacity)
                {
                    _capacity = value;
                    IBinaryHeapItem[] temp = new IBinaryHeapItem[_capacity];
                    Array.Copy(_data, temp, _count);
                    _data = temp;
                }
            }
        }

        // Methods
        /// <summary>
        /// Gets the first value in the heap without removing it.
        /// </summary>
        /// <returns>The lowest value of type TValue.</returns>
        public IBinaryHeapItem Peek()
        {
            return _data[0];
        }

        /// <summary>
        /// Removes all items from the heap.
        /// </summary>
        public void Clear()
        {
            this._count = 0;
            _data = new IBinaryHeapItem[_capacity]; // Faster than clearing the array, maybe
        }
        
        /// <summary>
        /// Adds a key and value to the heap.
        /// </summary>
        /// <param name="item">The item to add to the heap.</param>
        public void Add(IBinaryHeapItem item)
        {
            if (item == null)
                return;
            
            if (_count == _capacity)
                Capacity *= 2; // Automatically grows the array!

            item.SetIndexInHeap(_count);
            _data[_count] = item;
            _count++;
            UpHeap();
        }

        /// <summary>
        /// Removes and returns the first item in the heap.
        /// </summary>
        /// <returns>The next item in the heap.</returns>
        public IBinaryHeapItem Remove()
        {
            if (this._count == 0)
                throw new InvalidOperationException("Cannot remove item, heap is empty.");

            IBinaryHeapItem v = _data[0];
            v.SetIndexInHeap(REMOVED_FROM_HEAP);
            _count--;
            if (this._count != 0)
            {
                _data[0] = _data[_count];
                DownHeap();
            }
            _data[_count] = default(IBinaryHeapItem); // Clear the last node
            return v;
        }

        private void UpHeap()
        //helper function that performs up-heap bubbling - bubbles the last item up to its correct place
        // This is up if you imagine the heap as a down-growing tree:
        //             0
        //       1           2
        //   3       4   5       6
        {
            _sorted = false;
            int p = _count - 1;
            IBinaryHeapItem item = _data[p];
            int par = Parent(p);
            while (par > -1 && item.CompareTo(_data[par]) < 0)
            {
                _data[p] = _data[par]; // Swap parent down
                _data[p].SetIndexInHeap(p);
                p = par;
                par = Parent(p);
            }
            _data[p] = item; // Finally, place item at the base of the bubble-up chain
            _data[p].SetIndexInHeap(p);
        }
        
        private void DownHeap()
        //helper function that performs down-heap bubbling - bubbles the root down to its correct place
        // This is down if you imagine the heap as a down-growing tree:
        //             0
        //       1           2
        //   3       4   5       6
        {
            _sorted = false;
            int n;
            int p = 0;
            IBinaryHeapItem item = _data[p];
            while (true)
            {
                int ch1 = Child1(p);
                if (ch1 >= _count)
                    break;
                
                int ch2 = Child2(p);
                if (ch2 >= _count)
                    n = ch1;
                else
                    n = _data[ch1].CompareTo(_data[ch2]) < 0 ? ch1 : ch2;

                if (item.CompareTo(_data[n]) > 0)
                {
                    _data[p] = _data[n]; // Swap child up
                    _data[p].SetIndexInHeap(p);
                    p = n;
                }
                else
                {
                    break;
                }
            }
            _data[p] = item; // Finally, place item at the base of the bubble-down chain
            _data[p].SetIndexInHeap(p);
        }
        
        private void EnsureSort()
        {
            if (_sorted) return;
            Array.Sort(_data, 0, _count);
            _sorted = true;
        }
        
        /// <summary>
        /// helper function that calculates the parent of a node
        /// </summary>
        /// <param name="index"></param>
        /// <returns>-1 if there's no parent (index==0)</returns>
        private static int Parent(int index)
        {
            return (index - 1) >> 1;
        }
        
        /// <summary>
        /// helper function that calculates the first child of a node
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        private static int Child1(int index)
        {
            return (index << 1) + 1;
        }
        
        /// <summary>
        /// helper function that calculates the second child of a node
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        private static int Child2(int index)
        {
            return (index << 1) + 2;
        }

        /// <summary>
        /// Creates a new instance of an identical binary heap.
        /// </summary>
        /// <returns>A BinaryHeap.</returns>
        public BinaryHeap Copy()
        {
            return new BinaryHeap(_data, _count);
        }

        /// <summary>
        /// Gets an enumerator for the binary heap.
        /// </summary>
        /// <returns>An IEnumerator of type T.</returns>
        //public GetEnumerator()
        //{
        //    EnsureSort();
        //    for (int i = 0; i < _count; i++)
        //    {
        //        yield return _data[i];
        //    }
        //}
        //System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        //{
        //    return GetEnumerator();
        //}

        /// <summary>
        /// Checks to see if the binary heap contains the specified item.
        /// Uses CompareTo, not the item's binary heap index.
        /// First call runs in O(nlogn) time. Next calls are O(logn).
        /// </summary>
        /// <param name="item">The item to search the binary heap for.</param>
        /// <returns>A boolean, true if binary heap contains item.</returns>
        public bool Contains(IBinaryHeapItem item)
        {
            EnsureSort();
            return Array.BinarySearch<IBinaryHeapItem>(_data, 0, _count, item) >= 0;
        }
        
        /// <summary>
        /// Copies the binary heap to an array at the specified index.
        /// </summary>
        /// <param name="array">One dimensional array that is the destination of the copied elements.</param>
        /// <param name="arrayIndex">The zero-based index at which copying begins.</param>
        public void CopyTo(IBinaryHeapItem[] array, int arrayIndex)
        {
            EnsureSort();
            Array.Copy(_data, array, _count);
        }
        
        /// <summary>
        /// Returns whether or not the binary heap is readonly.
        /// </summary>
        public bool IsReadOnly
        {
            get { return false; }
        }
        
        /// <summary>
        /// Removes an item from the binary heap. 
        /// Assumes item is or was in the heap. Doesn't use Equality checks.
        /// This will not remove duplicates.
        /// TODO: Change into Remove(int binaryHeapIndex)!
        /// </summary>
        /// <param name="item">The item to be removed.</param>
        /// <returns>Boolean true if the item was removed.</returns>
        public bool Remove(IBinaryHeapItem item)
        {
            if (item == null)
                return false;
            int child_index = item.GetIndexInHeap();
            
            if (child_index == REMOVED_FROM_HEAP)
                return false;

            _data[child_index].SetIndexInHeap(REMOVED_FROM_HEAP);
            //if (child_index == 0) // This seems unnecessary
            //{
            //    Remove();
            //    return true;
            //}

            IBinaryHeapItem to_remove = _data[child_index];
            // Bubble to_remove up the heap
            // If UpHeap received an index parameter instead of always starting from the last element,
            // we could maybe remove some code duplication.
            int father_index = Parent(child_index);
            while (child_index != 0)
            {
                _data[child_index] = _data[father_index]; // Swap parent down
                _data[child_index].SetIndexInHeap(child_index);
                child_index = father_index;
                father_index = Parent(child_index);
            }
            // We got to 0
            _data[0] = to_remove;
            Remove(); // Ignoring the returned value.
            return true;
        }
    }
}
