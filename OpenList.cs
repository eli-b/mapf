using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// Supporting O(1) insertion and removal of items that compare equal to the top of the heap.
    /// </summary>
    [DebuggerDisplay("Count = {Count}")]
    public class OpenList : IAccumulatingStatisticsCsvWriter
    {
        //protected IBinaryHeapItem lastRemovedItem;
        protected Queue<IBinaryHeapItem> queue;
        //protected LinkedList<IBinaryHeapItem> queue;
        protected BinaryHeap heap;
        
        protected ISolver user;
        protected int quickInsertionCount;
        protected int accQuickInsertionCount;

        protected int quickInsertionsCancelled;
        protected int accQuickInsertionsCancelled;

        public OpenList(ISolver user)
        {
            this.heap = new BinaryHeap();
            this.queue = new Queue<IBinaryHeapItem>();

            this.user = user;
            this.ClearPrivateStatistics();
            this.ClearPrivateAccumulatedStatistics();
        }

        public int Count
        {
            get { return this.heap.Count + this.queue.Count; }
        }

        public IBinaryHeapItem Peek()
        {
            if (this.queue.Count != 0)
                return this.queue.Peek();
            return this.heap.Peek();
        }

        public void Clear()
        {
            this.queue.Clear();
            this.heap.Clear();
        }

        public void Add(IBinaryHeapItem item)
        {
            if (this.queue.Count == 0)
            {
                if (this.heap.Count == 0)
                    this.heap.Add(item); // It's very cheap.
                else
                {
                    int compareRes = item.CompareTo(this.heap.Peek());
                    if (compareRes != -1) // Even if equal, respect the stable order, don't "cut the line".
                        this.heap.Add(item);
                    else
                    {
                        this.queue.Enqueue(item);
                        this.quickInsertionCount++;
                    }
                }
            }
            else
            {
                int compareRes = item.CompareTo(this.queue.Peek());
                if (compareRes == 1) // item is larger than the queue
                    this.heap.Add(item);
                else // 
                {
                    if (compareRes == -1) // Item is smaller than the queue
                    {
                        while (this.queue.Count != 0)
                        {
                            IBinaryHeapItem fromQueue = this.queue.Dequeue();
                            this.heap.Add(fromQueue);
                            this.quickInsertionCount--;
                            this.quickInsertionsCancelled++;
                        }
                    }
                    this.queue.Enqueue(item);
                    this.quickInsertionCount++;
                }
            }

            //// The last removed item is the parent of all items added until another item is removed,
            //// or the same node that was last removed, partially expanded or deferred with increased cost.
            //// Otherwise the inserted item is one that was already in the open list, and its cost was
            //// was increased by one of the children of the last removed item. In this case, since the item
            //// wasn't the min of the open list and the last removed item was, now, with its increased cost,
            //// it certainly won't be smaller than the last removed item.
            //// If a partially expanded or otherwise deferred node is re-inserted with an updated cost,
            //// that must be done after all its children generated so far are inserted. Otherwise the
            //// cost comparison with the last removed item, which would still be the same node, would have
            //// incorrect results.
        }

        public virtual IBinaryHeapItem Remove()
        {
            IBinaryHeapItem item;
            if (this.queue.Count != 0)
            {
                item = this.queue.Dequeue();
                item.SetIndexInHeap(BinaryHeap.REMOVED_FROM_HEAP); // The heap assumes all items not in it have this index,
                                                                   // so we need to set it for when we search for this node
                                                                   // in the open list later.
            }
            else
                item = this.heap.Remove();
            return item;
        }

        /// <summary>
        /// Uses Equality check only for removing from the queue.
        /// Might cost O(n) if all items are in the queue and not the heap.
        /// </summary>
        /// <param name="item"></param>
        /// <returns></returns>
        public bool Remove(IBinaryHeapItem item)
        {
            if (item.GetIndexInHeap() == BinaryHeap.REMOVED_FROM_HEAP) // Or from queue.
                return false;

            bool removedFromQueue = false;
            // Remove from the queue if it's there, keeping the order in the queue.
            for (int i = 0; i < this.queue.Count; ++i )
            {
                IBinaryHeapItem temp = this.queue.Dequeue();
                if (temp.Equals(item))
                {
                    removedFromQueue = true;
                    temp.SetIndexInHeap(BinaryHeap.REMOVED_FROM_HEAP);
                }
                else
                    this.queue.Enqueue(temp);
            }
            if (removedFromQueue == true)
                return true;

            return this.heap.Remove(item);
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.user.ToString() + " Quick Insertions");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.user.ToString() + " Quick Insertions Cancelled");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine(this.user.ToString() + " Quick insertions: {0}", this.quickInsertionCount);
            Console.WriteLine(this.user.ToString() + " Quick insertions cancelled: {0}", this.quickInsertionsCancelled);

            output.Write(this.quickInsertionCount + Run.RESULTS_DELIMITER);
            output.Write(this.quickInsertionsCancelled + Run.RESULTS_DELIMITER);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 2;
            }
        }

        protected void ClearPrivateStatistics()
        {
            this.quickInsertionCount = 0;
            this.quickInsertionsCancelled = 0;
        }

        protected void ClearPrivateAccumulatedStatistics()
        {
            this.accQuickInsertionCount = 0;
            this.accQuickInsertionsCancelled = 0;
        }

        public virtual void ClearStatistics()
        {
            this.ClearPrivateStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.ClearPrivateAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accQuickInsertionCount += this.quickInsertionCount;
            this.accQuickInsertionsCancelled += this.quickInsertionsCancelled;
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine(this.user.ToString() + " Accumulated Quick insertions: {0}", this.accQuickInsertionCount);
            Console.WriteLine(this.user.ToString() + " Accumulated Quick insertions cancelled: {0}", this.accQuickInsertionsCancelled);

            output.Write(this.accQuickInsertionCount + Run.RESULTS_DELIMITER);
            output.Write(this.accQuickInsertionsCancelled + Run.RESULTS_DELIMITER);
        }

        public override string ToString()
        {
            return "OpenList";
        }
    }
}
