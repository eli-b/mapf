using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    class MddMatchAndPrune
    {
        MDD[] allMDDs;
        Queue<MddMatchAndPruneState> openList;
        Dictionary<MddMatchAndPruneState, MddMatchAndPruneState> closedList;
        int solutionDepth; //the depth is the cost + 1 because the root also counts as a level
        MddMatchAndPruneState goal; // This will contain the goal node if such was found
        bool legal; //indicates if all single MDDs are legal
        public bool[] conflicted; //indicates if the matching process found any illegal nodes/edges and pruned any of the MDDs
        Run runner;

        /// <summary>
        /// constructor
        /// </summary>
        public MddMatchAndPrune(Run runner)
        {
            this.openList = new Queue<MddMatchAndPruneState>();
            this.closedList = new Dictionary<MddMatchAndPruneState, MddMatchAndPruneState>();
            conflicted = new bool[4];
            this.runner = runner;
        }

        public void initialize(MDD[] allMDDs)
        {
            this.allMDDs = allMDDs;
            this.openList.Clear();
            this.closedList.Clear();
            MDDNode[] rootPositions = new MDDNode[allMDDs.Length];
            for (int i = 0; i < allMDDs.Length; i++)
            {
                if (allMDDs[i].levels == null)
                {
                    legal = false;
                    return;
                }
                rootPositions[i] = allMDDs[i].levels[0].First.Value;
                this.conflicted[i] = false;
            }
            MddMatchAndPruneState root = new MddMatchAndPruneState(rootPositions);
            this.solutionDepth = root.allPositions[0].mdd.levels.Length;
            openList.Enqueue(root);
            legal = true;
        }

        /// <summary>
        /// Build the generalized MDD
        /// </summary>
        /// <returns></returns>
        private bool buildGeneralMDD()
        {
            MddMatchAndPruneState current = openList.Dequeue();
            successorIterator allChildren = new successorIterator(allMDDs.Length);
            int currentLevel = current.stateLevel;

            while (current.stateLevel + 1 != this.solutionDepth) // while not goal
            {
                Expand(current,allChildren);
                if (openList.Count == 0)
                    return false;
                current = openList.Dequeue();
                if (current.stateLevel != currentLevel)
                {
                    closedList.Clear();
                    currentLevel++;
                }
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// expands a given node
        /// </summary>
        /// <param name="toExpand"></param>
        private void Expand(MddMatchAndPruneState toExpand, successorIterator allChildren)
        {
            allChildren.initialize(toExpand);
            MddMatchAndPruneState successor;

            while (allChildren.hasNext)
            {
                successor = allChildren.getNext();
                if (closedList.ContainsKey(successor))
                {
                    ((MddMatchAndPruneState)closedList[successor]).addParent(toExpand);
                }
                else
                {
                    CostTreeNodeSolver.matchCounter++;
                    successor.addParent(toExpand);
                    closedList.Add(successor, successor);
                    openList.Enqueue(successor);
                    if (successor.stateLevel + 1 == this.solutionDepth)
                        goal = successor;
                }
            }
        }

        private void reverseExpand(MddMatchAndPruneState toExpand)
        {
            foreach (MddMatchAndPruneState parent in toExpand.parents) 
            {
                for (int i = 0; i < toExpand.allPositions.Length; i++)
                {
                    CostTreeSearchSolver.edgesMatrix[i, parent.allPositions[i].getVertexIndex(), (int) Move.Direction.Wait] = CostTreeSearchSolver.edgesMatrixCounter + 1;
                }
                if (closedList.ContainsKey(parent) == false)
                {
                    openList.Enqueue(parent);
                    closedList.Add(parent, parent);
                }
            }
        }


        /// <summary>
        /// prunes a given level according to the edgesMatrix
        /// </summary>
        /// <param name="level"></param>
        private void pruneLevel(int level)
        {
            MDDNode[] parentsToDelte = new MDDNode[5];
            int parentI;

            for (int i = 0; i < allMDDs.Length; i++)
            {
                foreach (MDDNode node in allMDDs[i].levels[level])
                {
                    if (node.isDeleted)
                        continue;
                    parentI = 0;

                    for (int d = 0; d < 5; d++)
                        parentsToDelte[d] = null;

                    foreach (MDDNode parent in node.parents)
                    {
                        //if not legal
                        if ((int)CostTreeSearchSolver.edgesMatrix[i, parent.getVertexIndex(), (int) node.move.direction /*or parent.move.direction, I'm not sure*/] != CostTreeSearchSolver.edgesMatrixCounter + 1)
                        {
                            parentsToDelte[parentI] = parent;
                            this.conflicted[i] = true;
                        }
                        parentI++;
                    }
                    foreach (MDDNode delteParent in parentsToDelte)
                        if (delteParent != null)
                           node.removeParent(delteParent);
                }    
            }
        }

        /// <summary>
        /// prunes the given MDDs according to each other
        /// </summary>
         public bool pruneMDDs()
         {
             if (legal == false || buildGeneralMDD() == false)
                 return false;
   
             //Run.resultsWriterdd.Write(CostTreeNodeSolver.matchCounter + ",");
             //Run.resultsWriterdd.WriteLine();
             //Run.resultsWriterdd.Flush();
             //Run.resultsWriterdd.Close();
   
             if (openList.Count != 0)
                 Console.ReadLine();//should be empty
   
             MddMatchAndPruneState current = goal;
             int currentLevel = goal.stateLevel;
             closedList.Clear();
   
             while (current.stateLevel > 0) //while not root
             {
                 if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                     return false;
   
                 if (current.stateLevel < currentLevel)
                 {
                     pruneLevel(currentLevel);
                     currentLevel--;
                     CostTreeSearchSolver.edgesMatrixCounter++;
                     closedList.Clear();
                 }
                 reverseExpand(current);
                 current = openList.Dequeue();
             }
             pruneLevel(currentLevel); //prune level 1
             return true;
         }
   
    }

    class MddMatchAndPruneState
    {
        public MDDNode[] allPositions;
        public int stateLevel; //starts at 0
        public LinkedList<MddMatchAndPruneState> parents;
        LinkedList<MddMatchAndPruneState> childrens; 
   
        public MddMatchAndPruneState(MDDNode[] allPositions)
        {
            this.allPositions = allPositions;
            this.stateLevel = allPositions[0].move.time;
            this.parents = new LinkedList<MddMatchAndPruneState>();
            this.childrens = new LinkedList<MddMatchAndPruneState>();
        }

        public MddMatchAndPruneState(LinkedListNode<MDDNode>[] allSuccessors)
        {
            allPositions=new MDDNode[allSuccessors.Length];
            for (int i = 0; i < allPositions.Length; i++)
            {
                allPositions[i] = allSuccessors[i].Value;
            }
            this.stateLevel = allPositions[0].move.time;
            this.parents = new LinkedList<MddMatchAndPruneState>();
            this.childrens = new LinkedList<MddMatchAndPruneState>();
        }
       
        public void margeParents(LinkedList<MddMatchAndPruneState> otherParents)
        {
            parents.Union(otherParents);
        }
   
        public void addParent(MddMatchAndPruneState parent)
        {
            this.parents.AddLast(parent);
            parent.childrens.AddLast(this);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                for (int i = 0; i < allPositions.Length; i = i + 2)
                {
                    ans += allPositions[i].move.GetHashCode();
                }
                return ans;
            }
        }
    
        public override bool Equals(object obj)
        {
            MddMatchAndPruneState comp = (MddMatchAndPruneState)obj;
            return this.allPositions.SequenceEqual<MDDNode>(comp.allPositions);
        }
    }

    class successorIterator
    {
        LinkedListNode<MDDNode>[] nodesFromChildrenList;
        MddMatchAndPruneState prevStep;
        public bool hasNext;

        public successorIterator(int size)
        {
            this.nodesFromChildrenList=new LinkedListNode<MDDNode>[size]; 
        }

        public void initialize(MddMatchAndPruneState prevStep)
        {
            this.hasNext = true;
            this.prevStep = prevStep;
            for (int i = 0; i < nodesFromChildrenList.Length; i++)
            {
                nodesFromChildrenList[i] = prevStep.allPositions[i].children.First; //if crashes check there if there can be a node with no children
                if (nodesFromChildrenList[i] == null)
                    this.hasNext = false;
            }
            while (hasNext == true && isLegal(0) == false)
            {
                getNext();
            }
        }

        /// <summary>
        /// Return the next generated child.
        /// </summary>
        /// <returns></returns>
        public MddMatchAndPruneState getNext()
        {
            //first return than iterate
            MddMatchAndPruneState ans = new MddMatchAndPruneState(nodesFromChildrenList);
            if (nextChild(0) == false)
            {
                hasNext = false;
            }
            return ans;
        }

        /// <summary>
        /// Sets the next child in line for a given agent, if it's the last child returns the first in order (closed loop). If reset returns false.
        /// </summary>
        /// <param name="agent"></param>
        private bool nextSuccessor(int agent)
        {
            if (nodesFromChildrenList[agent].Next == null)
            {
                nodesFromChildrenList[agent] = nodesFromChildrenList[agent].List.First;
                return false;
            }

            nodesFromChildrenList[agent] = nodesFromChildrenList[agent].Next;
            return true;
        }

        /// <summary>
        /// Recursive function, try to proceed the first agent if it is at the end of its children resets him and proceeds the second agent and so on
        /// </summary>
        /// <param name="agent"></param>
        private bool nextChild(int agent)
        {
            if (agent == nodesFromChildrenList.Length)
                return false;
            if (nextSuccessor(agent) == false)
                return nextChild(agent+1);
            while (isLegal(agent) == false)
            {
                if (nextSuccessor(agent) == false)
                    return nextChild(agent+1);
            }

            if (agent > 0 && isLegal(0) == false)
                return nextChild(0);
            return true;
        }

        /// <summary>
        /// if we move the i'st agent we check whether it is legal with all agents that moved before, i.e. from that point forward
        /// </summary>
        /// <param name="checkFrom"></param>
        /// <returns></returns>
         private bool isLegal(int checkFrom)
        {
            // check if all moves are legal from the i agents forward (collisons+head on collisions)
             for (int i = checkFrom; i < nodesFromChildrenList.Length - 1; i++)
            {
                for (int j = i+1; j < nodesFromChildrenList.Length; j++)
                {
                    if (nodesFromChildrenList[i].Value.Equals(nodesFromChildrenList[j].Value))
                        return false;
                    if (nodesFromChildrenList[i].Value.getVertexIndex() == prevStep.allPositions[j].getVertexIndex() && nodesFromChildrenList[j].Value.getVertexIndex() == prevStep.allPositions[i].getVertexIndex())
                        return false;
                }
            }
            return true;
        }


    }
}
