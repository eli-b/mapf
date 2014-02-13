using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class MddMatchAndPrune
    {
        MDD[] allMDDs;
         Queue<MddMatchAndPruneState> openList;
         HashTable_C closedList;
         int solutionDepth; //the depth is the cost + 1 because the root also counts as a level
         MddMatchAndPruneState goal; // This will contain the goal node if such was found
         bool leagel; //indicates if all single MDDs are leagel
         public bool[] conflicted;//indicated if the matching proccess foud any ileagel nodes/edges and pruned any of the MDDs
         Run runner;

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="root"></param>
         public MddMatchAndPrune(Run runner)
         {
             this.openList = new Queue<MddMatchAndPruneState>();
             this.closedList=new HashTable_C();
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
                     leagel = false;
                     return;
                 }
                 rootPositions[i] = allMDDs[i].levels[0].First.Value;
                 this.conflicted[i] = false;
             }
             MddMatchAndPruneState root = new MddMatchAndPruneState(rootPositions);
             this.solutionDepth = root.allPositions[0].father.levels.Length;
             openList.Enqueue(root);
             leagel = true;
         }

        /// <summary>
        /// build the generalized MDD
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
                 if (closedList.Contains(successor))
                 {
                     ((MddMatchAndPruneState)closedList[successor]).addParent(toExpand);
                 }
                 else
                 {
                     CostTreeNodeSolver.matchCounter++;
                     successor.addParent(toExpand);
                     closedList.Add(successor);
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
                     CostTreeSearchSolver.edgesMatrix[i, parent.allPositions[i].getVertxIndex(), parent.allPositions[i].getDirection(toExpand.allPositions[i])] = CostTreeSearchSolver.edgesMatrixCounter + 1;
                 }
                 if (closedList.Contains(parent) == false)
                 {
                     openList.Enqueue(parent);
                     closedList.Add(parent);
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
                         //if not leagel
                         if ((int)CostTreeSearchSolver.edgesMatrix[i, parent.getVertxIndex(), parent.getDirection(node)] != CostTreeSearchSolver.edgesMatrixCounter + 1)
                         {
                             parentsToDelte[parentI] = parent;
                             this.conflicted[i] = true;
                         }
                         parentI++;
                     }
                     foreach (MDDNode delteParent in parentsToDelte)
                         if(delteParent!=null)
                            node.removeParent(delteParent);
                 }    
             }
         }

        /// <summary>
        /// prunes the given MDDs according to each other
        /// </summary>
         public bool pruneMDDs()
         {
             if (leagel==false || buildGeneralMDD() == false)
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
            this.stateLevel = allPositions[0].level;
            this.parents = new LinkedList<MddMatchAndPruneState>();
            this.childrens = new LinkedList<MddMatchAndPruneState>();
        }

        public MddMatchAndPruneState(LinkedListNode<MDDNode>[] allSuccessors)
        {
            allPositions=new MDDNode[allSuccessors.Length];
            for (int i = 0; i < allPositions.Length; i++)
			{
			    allPositions[i]=allSuccessors[i].Value;
			}
            this.stateLevel = allPositions[0].level;
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
            int ans = 0;
            for (int i = 0; i < allPositions.Length; i=i+2)
            {
                ans += allPositions[i].getX() * Constants.PRIMES_FOR_HASHING[i%22];
                ans += allPositions[i].getY() * Constants.PRIMES_FOR_HASHING[(i+1)%22];
            }
            return ans;
        }

        public override bool Equals(object obj)
        {
            MddMatchAndPruneState comp = (MddMatchAndPruneState)obj;
            for (int i = 0; i < this.allPositions.Length; i++)
                if (this.allPositions[i].Equals(comp.allPositions[i]) == false)
                    return false;
            return true;
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
            while (hasNext == true && isLeagel(0) == false)
            {
                getNext();
            }
        }

        /// <summary>
        /// return the next generated chield
        /// </summary>
        /// <returns></returns>
        public MddMatchAndPruneState getNext()
        {
            //first return than iterate
            MddMatchAndPruneState ans=new MddMatchAndPruneState(nodesFromChildrenList);
            if(nextChield(0)==false)
            {
                hasNext=false;
            }
            return ans;
        }

        /// <summary>
        /// sets the next chield in line for a given agent, if its the last chield returns the first in order (closed loop). if reseted returns false
        /// </summary>
        /// <param name="agent"></param>
        private bool nextSuccessor(int agent)
        {
            if (nodesFromChildrenList[agent].Next == null)
            {
                nodesFromChildrenList[agent]=nodesFromChildrenList[agent].List.First;
                return false;
            }

            nodesFromChildrenList[agent]=nodesFromChildrenList[agent].Next;
            return true;
        }

        /// <summary>
        /// recorsive function, try to proceed the first agent if it is at the end of its children resets him and proceeds the second agent and so on
        /// </summary>
        /// <param name="agent"></param>
        private bool nextChield(int agent)
        {
            if(agent==nodesFromChildrenList.Length)
                return false;
            if(nextSuccessor(agent)==false)
                return nextChield(agent+1);
            while (isLeagel(agent)==false)
            {
	            if(nextSuccessor(agent)==false)
                    return nextChield(agent+1);
            }

            if (agent > 0 && isLeagel(0) == false)
                return nextChield(0);
	        return true;
        }

        /// <summary>
        /// if we move the i'st agent we check whether it is leagel with all agents that moved before i.e. from that point forword
        /// </summary>
        /// <param name="checkFrom"></param>
        /// <returns></returns>
         private bool isLeagel(int checkFrom)
        {
            // check if all moves are leagel from the i agent forword (collisons+had on collisons)
             for (int i = checkFrom; i < nodesFromChildrenList.Length - 1; i++)
			{
			    for (int j = i+1; j < nodesFromChildrenList.Length; j++)
			    {
			        if(nodesFromChildrenList[i].Value.Equals(nodesFromChildrenList[j].Value))
                        return false;
                    if (nodesFromChildrenList[i].Value.getVertxIndex()==prevStep.allPositions[j].getVertxIndex() && nodesFromChildrenList[j].Value.getVertxIndex()==prevStep.allPositions[i].getVertxIndex())
                        return false;
			    }
			}
            return true;
        }


    }
}
