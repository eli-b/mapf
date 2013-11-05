using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experement
{
    class AStarMDD
    {
        MDD[] problem;
        HashTable_C closedList;
        Run runner;
        BinaryHeap openList;
        public int expanded;
        public int generated;
        public int conflictAvoidenceVaiulations;
        HashSet<TimedMove> ID_CAT;
        HashSet_U<TimedMove> CBS_CAT;


        public AStarMDD(MDD[] problem, Run runner, HashSet<TimedMove> conflicts, HashSet_U<TimedMove> CBS_CAT)
        {
            this.expanded = 0;
            this.generated = 0;
            MDDStep root;
            this.problem = problem;
            this.runner = runner;
            this.ID_CAT = conflicts;
            this.CBS_CAT = CBS_CAT;
            this.closedList = new HashTable_C();
            this.openList = new BinaryHeap();
            MDDNode[] sRoot = new MDDNode[problem.Length];
            for (int i = 0; i < problem.Length; i++)
            {
                sRoot[i] = problem[i].levels[0].First.Value;
                sRoot[i].leagel = true;
            }
            root = new MDDStep(sRoot, null);
            openList.Add(root);
            conflictAvoidenceVaiulations = 0;
        }
       
        public LinkedList<Move>[] solve()
        {

            MDDStep currentNode;
            ExpandedNode toExpand = new ExpandedNode();

            while (openList.Count>0)
            {
                 if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    return null;
                }
                 currentNode = (MDDStep)openList.Remove();
                // Check if node is the goal
                if (goalTest(currentNode))
                {
                    conflictAvoidenceVaiulations=currentNode.conflicts;
                    return getAnswear(currentNode);
                }

                // Expand
                expanded++;
                toExpand.setup(currentNode);
                expand(toExpand);
            }
            return null;
        }

        public void expand(ExpandedNode currentNode)
        {
            MDDStep chield = currentNode.getNextChield();
            while (chield != null)
            {
                if (isLeagelMove(chield))
                {
                        chield.conflicts = currentNode.parent.conflicts;
                        chield.setConflicts(ID_CAT, CBS_CAT);

                    if (this.closedList.Contains(chield) == true)
                    {
                        MDDStep inClosedList = (MDDStep)this.closedList[chield];

                        if (inClosedList.conflicts > chield.conflicts)
                        {
                            closedList.Remove(inClosedList); //than remove state
                            openList.Remove(inClosedList);
                        }
                    }
                    if (this.closedList.Contains(chield) == false)
                    {
                        this.openList.Add(chield);
                        this.closedList.Add(chield);
                        generated++;
                    }
                }
                chield = currentNode.getNextChield();
            }
        }
        public void clearIleagel()
        {
            foreach (MDD mdd in problem)
                foreach (LinkedList<MDDNode> level in mdd.levels)
                    foreach (MDDNode node in level)
                        if (node.leagel == false)
                            node.delete();
        }
        public void resetIleagel()
        {
            foreach (MDD mdd in problem)
                for (int i = 1; i < mdd.levels.Length; i++)
                    foreach (MDDNode node in mdd.levels[i])
                            node.leagel = false;
        }
        public int getGenerated() { return closedList.Count; }
        public int getExpanded() { return this.expanded; }
        private bool goalTest(MDDStep toCheck)
        {
            if (toCheck.getDepth() == problem[0].levels.Length - 1)
                return true;
            return false;
        }
        private LinkedList<Move>[] getAnswear(MDDStep finish)
        {
            if (finish==null)
                return new LinkedList<Move>[1];
            LinkedList<Move>[] ans = new LinkedList<Move>[problem.Length];
            int direction;
            for (int i = 0; i < ans.Length; i++)
                ans[i] = new LinkedList<Move>();
            MDDStep current = finish;
            while (current.prevStep != null)
            {
                for (int i = 0; i < problem.Length; i++)
                {
                    direction = Move.getDirection(current.allSteps[i].getX(), current.allSteps[i].getY(), current.prevStep.allSteps[i].getX(), current.prevStep.allSteps[i].getY());
                    ans[i].AddFirst(new Move(current.allSteps[i].getX(), current.allSteps[i].getY(), direction));
                }
                current = current.prevStep;
            }
            for (int i = 0; i < problem.Length; i++)
            {
                ans[i].AddFirst(new Move(current.allSteps[i].getX(), current.allSteps[i].getY(), 0));
            }
            return ans;
        }
        private bool checkIfLeagel(MDDNode from1, MDDNode to1, MDDNode from2, MDDNode to2)
        {
            if (to1.getX() == to2.getX() && to1.getY() == to2.getY())
                return false;
            if (to1.getX() == from2.getX() && from1.getX() == to2.getX() && to1.getY() == from2.getY() && from1.getY() == to2.getY())
                return false;
            return true;
        }
        private bool isLeagelMove(MDDStep to)
        {
            if (to == null)
                return false;
            if(to.prevStep==null)
                return true;
            for (int i = 0; i < problem.Length; i++)
            {
                for (int j = i+1; j < to.allSteps.Length; j++)
                {
                    if (checkIfLeagel(to.prevStep.allSteps[i], to.allSteps[i], to.prevStep.allSteps[j], to.allSteps[j]) == false)
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
        public int depth;
        public int conflicts;
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
        public override bool Equals(object obj)
        {
            MDDStep comp = (MDDStep)obj;
            for (int i = 0; i < allSteps.Length; i++)
            {
                if (this.allSteps[i].Equals(comp.allSteps[i]) == false)
                    return false;
            }
            return true;
        }
        public override int GetHashCode()
        {
            int code = 0;
            for (int i = 0; i < allSteps.Length; i++)
            {
                code += allSteps[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % 22];
            }
            return code;
        }
        public int getDepth() { return allSteps[0].level; }
        public void setConflicts(HashSet<TimedMove> ID_CAT, HashSet_U<TimedMove> CBS_CAT)
        {
            TimedMove m2 = new TimedMove();
            if (this.prevStep == null)
                return;
            for (int i = 0; i < allSteps.Length; i++)
            {
                    m2.setup(allSteps[i].getX(), allSteps[i].getY(), -1, getDepth());
                    if (ID_CAT != null && ID_CAT.Contains(m2))
                        conflicts++;
                    if (CBS_CAT != null && CBS_CAT.Contains(m2))
                        conflicts++;
                    m2.direction = Move.getDirection(allSteps[i].getX(), allSteps[i].getY(), prevStep.allSteps[i].getX(), prevStep.allSteps[i].getY());
                    m2.setOppositeMove();
                    if (ID_CAT != null && ID_CAT.Contains(m2))
                        conflicts++;
                    if (CBS_CAT != null && CBS_CAT.Contains(m2))
                        conflicts++;
            }
        }


        /// <summary>
        /// BH_Iteam implimatation
        /// </summary>
        /// <returns></returns>
        public int getIndexInHeap() { return binaryHeapIndex; }
        /// <summary>
        /// BH_Iteam implimatation
        /// </summary>
        /// <returns></returns>
        public void setIndexInHeap(int index) { binaryHeapIndex = index; }

        public int CompareTo(IBinaryHeapItem other)
        {
            MDDStep that = (MDDStep)other;
            if (this.conflicts  < that.conflicts)
                return -1;
            if (this.conflicts > that.conflicts)
                return 1;

            if (this.getDepth() > that.getDepth())
                return -1;
            if (this.getDepth() < that.getDepth())
                return 1;

            return 0;
        }
    }
    class ExpandedNode
    {
        public MDDStep parent;
        int[] chosenChield;
        public ExpandedNode() { }
        public ExpandedNode(MDDStep parent)
        {
            this.parent = parent;
            chosenChield = new int[parent.allSteps.Length];
            foreach (MDDNode node in parent.allSteps)
            {
                if (node.children.Count == 0)
                {
                    chosenChield[0] = -1;
                    break;
                }
            }
        }
        public void setup(MDDStep parent)
        {
            this.parent = parent;
            chosenChield = new int[parent.allSteps.Length];
            foreach (MDDNode node in parent.allSteps)
            {
                if (node.children.Count == 0)
                {
                    chosenChield[0] = -1;
                    break;
                }
            }
        }
        public MDDStep getNextChield()
        {
            if (chosenChield[0] == -1)
                return null;
            MDDNode[] ans=new MDDNode[parent.allSteps.Length];
            for (int i = 0; i < ans.Length; i++)
			{
			    ans[i]=parent.allSteps[i].children.ElementAt(chosenChield[i]);
			}
            setNextChield(chosenChield.Length-1);
            return new MDDStep(ans, parent);
        }
        private void setNextChield(int agentNum)
        {
            if(agentNum==-1)
                chosenChield[0] = -1;
            else if (chosenChield[agentNum] < parent.allSteps[agentNum].children.Count - 1)
                chosenChield[agentNum]++;
            else
            {
                chosenChield[agentNum] = 0;
                setNextChield(agentNum - 1);
            }
        }
    }
}
