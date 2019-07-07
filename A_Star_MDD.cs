using System;
using System.Collections.Generic;
using System.Linq;

namespace mapf
{
    /// <summary>
    /// Finds the solution with the least number of conflicts, given a set of MDDs
    /// </summary>
    class A_Star_MDD
    {
        MDD[] problem;
        Dictionary<MDDStep, MDDStep> closedList;
        Run runner;
        BinaryHeap<MDDStep> openList;
        public int expanded;
        public int generated;
        public int conflictCount;
        Dictionary<TimedMove, List<int>> ID_CAT;
        Dictionary<TimedMove, List<int>> CBS_CAT;

        public A_Star_MDD(MDD[] problem, Run runner, Dictionary<TimedMove, List<int>> ID_CAT, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            this.expanded = 0;
            this.generated = 0;
            MDDStep root;
            this.problem = problem;
            this.runner = runner;
            this.ID_CAT = ID_CAT;
            this.CBS_CAT = CBS_CAT;
            this.closedList = new Dictionary<MDDStep, MDDStep>();
            this.openList = new BinaryHeap<MDDStep>();
            MDDNode[] sRoot = new MDDNode[problem.Length];
            for (int i = 0; i < problem.Length; i++)
            {
                sRoot[i] = problem[i].levels[0].First.Value;
                sRoot[i].legal = true;
            }
            root = new MDDStep(sRoot, null);
            openList.Add(root);
            closedList.Add(root, root); // There will never be a hit. This is only done for consistancy
            conflictCount = 0;
        }
       
        public SinglePlan[] Solve()
        {
            MDDStep currentNode;
            ExpandedNode toExpand = new ExpandedNode();

            while (openList.Count > 0)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    return null;
                }
                currentNode = openList.Remove();
                // Check if node is the goal
                if (this.GoalTest(currentNode))
                {
                    conflictCount = currentNode.conflictCount;
                    return GetAnswer(currentNode);
                }

                // Expand
                expanded++;
                toExpand.Setup(currentNode);
                Expand(toExpand);
            }
            return null;
        }

        public void Expand(ExpandedNode currentNode)
        {
            while (true)
            {
                MDDStep child = currentNode.GetNextChild();
                if (child == null)
                    break;

                if (IsLegalMove(child))
                {
                    child.conflictCount = currentNode.parent.conflictCount;
                    child.SetConflicts(ID_CAT, CBS_CAT);

                    if (this.closedList.ContainsKey(child) == true)
                    {
                        MDDStep inClosedList = this.closedList[child];

                        if (inClosedList.conflictCount > child.conflictCount)
                        {
                            closedList.Remove(inClosedList);
                            openList.Remove(inClosedList);
                        }
                    }
                    if (this.closedList.ContainsKey(child) == false)
                    {
                        this.openList.Add(child);
                        this.closedList.Add(child, child);
                        generated++;
                    }
                }
            }
        }

        public int GetGenerated() { return this.generated; }
        
        public int GetExpanded() { return this.expanded; }
        
        private bool GoalTest(MDDStep toCheck)
        {
            if (toCheck.GetDepth() == problem[0].levels.Length - 1)
                return true;
            return false;
        }

        private SinglePlan[] GetAnswer(MDDStep finish)
        {
            // TODO: Move the construction of the SinglePlans to a static method in SinglePlan
            var routes = new LinkedList<Move>[problem.Length];
            for (int i = 0; i < routes.Length; i++)
                routes[i] = new LinkedList<Move>();

            MDDStep current = finish;
            while (current != null)
            {
                for (int i = 0; i < problem.Length; i++)
                {
                    routes[i].AddFirst(new Move(current.allSteps[i].move));
                }
                current = current.prevStep;
            }

            var ans = new SinglePlan[problem.Length];
            for (int i = 0; i < ans.Length; i++)
                ans[i] = new SinglePlan(routes[i], i);
            return ans;
        }
        
        private bool CheckIfLegal(MDDNode to1, MDDNode to2)
        {
            return to1.move.IsColliding(to2.move) == false;
        }
        
        private bool IsLegalMove(MDDStep to)
        {
            if (to == null)
                return false;
            if (to.prevStep == null)
                return true;
            for (int i = 0; i < problem.Length; i++)
            {
                for (int j = i+1; j < to.allSteps.Length; j++)
                {
                    if (CheckIfLegal(to.allSteps[i], to.allSteps[j]) == false)
                        return false;
                }
            }
            return true;
        }
    }

    class MDDStep : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        public MDDNode[] allSteps;
        public MDDStep prevStep;
        public int conflictCount;
        int binaryHeapIndex;

        public MDDStep(MDDNode[] allSteps, MDDStep prevStep)
        { 
            this.allSteps = allSteps;
            this.prevStep = prevStep;
        }

        public MDDStep(MDDStep cpy)
        {
            this.allSteps = new MDDNode[cpy.allSteps.Length];
            for (int i = 0; i < allSteps.Length; i++)
            {
                this.allSteps[i] = cpy.allSteps[i];
            }
            this.prevStep = cpy.prevStep;
        }

        /// <summary>
        /// Only compares the steps.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            MDDStep comp = (MDDStep)obj;
            return this.allSteps.SequenceEqual<MDDNode>(comp.allSteps);
        }

        /// <summary>
        /// Only uses the steps
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int code = 0;
                for (int i = 0; i < allSteps.Length; i++)
                {
                    code += allSteps[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
                }
                return code;
            }
        }

        public int GetDepth() { return allSteps[0].move.time; }

        /// <summary>
        /// Updates the conflicts member according to given CATs. Both tables may be null.
        /// </summary>
        /// <param name="ID_CAT"></param>
        /// <param name="CBS_CAT"></param>
        public void SetConflicts(Dictionary<TimedMove, List<int>> ID_CAT, Dictionary<TimedMove, List<int>> CBS_CAT)
        {
            TimedMove queryMove = new TimedMove();
            if (this.prevStep == null)
                return;
            for (int i = 0; i < allSteps.Length; i++)
            {
                // TODO: Kill this code dup. The ConflictAvoidanceTable class takes care of it.
                queryMove.setup(allSteps[i].move.x, allSteps[i].move.y, Move.Direction.NO_DIRECTION, allSteps[i].move.time);
                if (ID_CAT != null && ID_CAT.ContainsKey(queryMove))
                    conflictCount++;
                if (CBS_CAT != null && CBS_CAT.ContainsKey(queryMove))
                    conflictCount++;
                queryMove.direction = allSteps[i].move.direction;
                queryMove.setOppositeMove();
                if (ID_CAT != null && ID_CAT.ContainsKey(queryMove))
                    conflictCount++;
                if (CBS_CAT != null && CBS_CAT.ContainsKey(queryMove))
                    conflictCount++;
            }
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }
        
        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        public int CompareTo(IBinaryHeapItem other)
        {
            MDDStep that = (MDDStep)other;
            if (this.conflictCount  < that.conflictCount)
                return -1;
            if (this.conflictCount > that.conflictCount)
                return 1;

            if (this.GetDepth() > that.GetDepth())
                return -1;
            if (this.GetDepth() < that.GetDepth())
                return 1;

            return 0;
        }
    }

    class ExpandedNode
    {
        public MDDStep parent;
        int[] chosenChild;

        public ExpandedNode() { }

        public ExpandedNode(MDDStep parent)
        {
            this.parent = parent;
            this.chosenChild = new int[parent.allSteps.Length];
            foreach (MDDNode node in parent.allSteps)
            {
                if (node.children.Count == 0)
                {
                    chosenChild[0] = -1;
                    break;
                }
            }
        }

        public void Setup(MDDStep parent)
        {
            this.parent = parent;
            this.chosenChild = new int[parent.allSteps.Length];
            foreach (MDDNode node in parent.allSteps)
            {
                if (node.children.Count == 0)
                {
                    this.chosenChild[0] = -1;
                    break;
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>The next child, or null if there aren't any more</returns>
        public MDDStep GetNextChild()
        {
            if (this.chosenChild[0] == -1)
                return null;
            var ans = new MDDNode[parent.allSteps.Length];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = parent.allSteps[i].children.ElementAt(this.chosenChild[i]);
            }
            SetNextChild(this.chosenChild.Length - 1);
            return new MDDStep(ans, parent);
        }

        private void SetNextChild(int agentNum)
        {
            if (agentNum == -1)
                this.chosenChild[0] = -1;
            else if (this.chosenChild[agentNum] < parent.allSteps[agentNum].children.Count - 1)
                this.chosenChild[agentNum]++;
            else
            {
                this.chosenChild[agentNum] = 0;
                SetNextChild(agentNum - 1);
            }
        }
    }
}
