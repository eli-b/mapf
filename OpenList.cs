using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// Supporting O(1) insertion and removal of items that compare equal to the top of the heap.
    /// </summary>
    public class OpenList : IAccumulatingStatisticsCsvWriter
    {
        //protected IBinaryHeapItem lastRemovedItem;
        protected Queue<IBinaryHeapItem> queue;
        //protected LinkedList<IBinaryHeapItem> queue;
        protected BinaryHeap heap;
        
        protected ISolver user;
        protected int quickInsertionCount;
        protected int accQuickInsertionCount;

        public OpenList(ISolver user)
        {
            this.heap = new BinaryHeap();
            this.queue = new Queue<IBinaryHeapItem>();
            //this.queue = new LinkedList<IBinaryHeapItem>();

            this.user = user;
            //this.lastRemovedItem = null;
            this.quickInsertionCount = 0;
        }

        public int Count
        {
            get { return this.heap.Count + this.queue.Count; }
        }

        public IBinaryHeapItem Peek()
        {
            if (this.queue.Count != 0)
                return this.queue.Peek();
                //return this.queue.Last.Value;
            return this.heap.Peek();
        }

        public void Clear()
        {
            this.queue.Clear();
            this.heap.Clear();
            //this.lastRemovedItem = null;
        }

        public void Add(IBinaryHeapItem item)
        {
            if (this.queue.Count == 0)
            {
                if (this.heap.Count == 0)
                    this.heap.Add(item); // And it's very cheap.
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
            //if (this.lastRemovedItem != null)
            //{
            //    if (item != this.lastRemovedItem) // This is a child node (or a node equal to a child node, with updated cost)
            //    {
            //        IBinaryHeapItem parent = this.lastRemovedItem;
            //        int compareRes = item.CompareTo(parent);
            //        if (compareRes == 1)
            //            this.heap.Add(item);
            //        else
            //        {
            //            if (compareRes == 0) // This item would have gone straight to the top (min) of the heap:
            //            // Removed items are assumed to be in non-decreasing order, and this one
            //            // costs the same as the last item removed, so it has to be the next item
            //            // to be removed.
            //                this.queue.Enqueue(item);
            //                //this.queue.AddFirst(item);
            //            else // compareRes == -1. Only a goal is allowed to be smaller than its parent.
            //                 // Others will be intercepted as an error in the best-first-search.
            //                ...
            //                //this.queue.AddLast(item); // Add to front of queue
            //            //throw new Exception("Inserting child node " + item + " to the open list, which costs less than its parent " + this.lastRemovedItem + ".");
            //            ++this.quickInsertionCount;
            //        }
            //    }
            //    else // This is the same node, now partially expanded, or otherwise deferred.
            //         // This happens a lot when using DyanmicLazyOpenList
            //    {
            //        // Can't compare this node to the last removed item (itself), since its cost may have changed since.
            //        // But the elements in the queue, if they exist, remember its previous cost:
            //        if (this.queue.Count != 0) // So this item also came from the queue. The queue's head was equal to this item before.
            //        {
            //            IBinaryHeapItem queueRepresentative = this.queue.Peek();
            //            //IBinaryHeapItem queueRepresentative = this.queue.Last.Value; //this.queue.Peek();
            //            int compareRes = item.CompareTo(queueRepresentative);
            //            if (compareRes == 1) // The item's cost was increased
            //                this.heap.Add(item);
            //            else
            //            {
            //                if (compareRes == -1) // The item's cost was decreased! In DynamicLazyOpenList, this happens only when the expensive heuristic finds the goal.
            //                    //this.queue.AddLast(item); // Add to front of queue
            //                    ...
            //                else // The item's cost didn't change. Give another the other equal items a chance by adding the item to the end of the queue.
            //                     //I don't see when this would happen.
            //                    //this.queue.AddFirst(item);
            //                    this.queue.Enqueue(item);
            //                ++this.quickInsertionCount;
            //            }
            //            // FIXME: Code dup with child insertion
            //        }
            //        else
            //        {
            //            if (this.heap.Count != 0 && item.CompareTo(this.heap.Peek()) == -1) // Item is still strictly better than the next
            //            {
            //                this.queue.Enqueue(item);
            //                //this.queue.AddFirst(item);
            //                ++this.quickInsertionCount;
            //            }
            //            else // Item could be equal to the next, but we want to give the next a chance too (the heap is stable - doesn't swap equal items).
            //                this.heap.Add(item);
            //        }
            //    }
            //}
            //else
            //    this.heap.Add(item); // It's tempting to add it to the queue instead, but this way supports multiple insertions of
            //                         // different costs before the first removal. That's not supposed to happen, but who knows.
        }

        public virtual IBinaryHeapItem Remove()
        {
            IBinaryHeapItem item;
            if (this.queue.Count != 0)
            {
                item = this.queue.Dequeue();
                //item = this.queue.Last.Value;
                //this.queue.RemoveLast();
                item.SetIndexInHeap(BinaryHeap.REMOVED_FROM_HEAP); // The heap assumes all items not in it have this index,
                                                                   // so we need to set it for when we search for this node
                                                                   // in the open list later.
            }
            else
                item = this.heap.Remove();
            //this.lastRemovedItem = item;
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
                //IBinaryHeapItem temp = this.queue.Last.Value;
                //this.queue.RemoveLast();
                if (temp.Equals(item))
                {
                    removedFromQueue = true;
                    temp.SetIndexInHeap(BinaryHeap.REMOVED_FROM_HEAP);
                }
                else
                    this.queue.Enqueue(temp);
                    //this.queue.AddFirst(temp);
            }
            if (removedFromQueue == true)
                return true;

            return this.heap.Remove(item);
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.user.ToString() + " Quick Insertions");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine(this.user.ToString() + " Quick insertions: {0}", this.quickInsertionCount);

            output.Write(this.quickInsertionCount + Run.RESULTS_DELIMITER);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 1;
            }
        }

        public virtual void ClearStatistics()
        {
            this.quickInsertionCount = 0;
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accQuickInsertionCount = 0;
        }

        public virtual void AccumulateStatistics()
        {
            this.accQuickInsertionCount += this.quickInsertionCount;
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine(this.user.ToString() + " Quick insertions: {0}", this.quickInsertionCount);

            output.Write(this.accQuickInsertionCount + Run.RESULTS_DELIMITER);
        }

        public override string ToString()
        {
            return "OpenList";
        }
    }
}
