using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;

namespace CPF_experiment
{
    class MDD
    {
        public LinkedList<MDDNode>[] levels;
        private int agentNum;
        private int mddNum;
        public ProblemInstance problem;

        public MDD(int mddNum, int agentNum, int start_Pos_X, int start_pos_y, int cost,int maxCostOnLevel, int numOfAgents, ProblemInstance instance)
        {
            //if (agentNum == 2 && maxCostOnLevel == 4)
            //    Console.Write("ff");
            this.problem = instance;
            this.mddNum = mddNum;
            this.agentNum=agentNum;
            levels = new LinkedList<MDDNode>[maxCostOnLevel + 1];
            Hashtable closedList = new Hashtable();
            LinkedList<MDDNode> children;
            LinkedList<MDDNode> toDelete = null;

            for (int i = 0; i <= maxCostOnLevel; i++)
            {
                levels[i] = new LinkedList<MDDNode>();
            }
            MDDNode toAdd = new MDDNode(start_Pos_X, start_pos_y,0 , numOfAgents, this);
            LinkedListNode<MDDNode> llNode = new LinkedListNode<MDDNode>(toAdd);
            toAdd.setMyNode(llNode);
            llNode.Value.startOrGoal = true;
            levels[0].AddFirst(llNode);
            for (int i = 0; i < maxCostOnLevel ; i++)
            {
                int heuristicBound = cost - i - 1;
                if (heuristicBound < 0)
                {
                    heuristicBound = 0;
                }
                LinkedListNode<MDDNode> currentMddNode = levels[i].First;
                while (currentMddNode!=null)
                {
                    LinkedListNode<MDDNode> child;
                    children = this.GetAllChildren(currentMddNode.Value, heuristicBound,numOfAgents,i);
                    child = children.First;
                    if (child == null)
                    {
                        if (toDelete == null)
                            toDelete = new LinkedList<MDDNode>();
                        toDelete.AddFirst(currentMddNode.Value);
                    }
                    while (child != null)
                    {
                        toAdd = child.Value;
                        if (closedList.Contains(toAdd))
                        {
                            toAdd = (MDDNode)closedList[toAdd];
                        }
                        else
                        {
                            closedList.Add(toAdd,toAdd);
                            llNode = new LinkedListNode<MDDNode>(toAdd);
                            toAdd.setMyNode(llNode);
                            levels[i + 1].AddLast(llNode);
                        }
                        currentMddNode.Value.addChild(toAdd);
                        toAdd.addParent(currentMddNode.Value);

                        child = child.Next;
                    }


                    currentMddNode=currentMddNode.Next;
                }
                closedList.Clear();
            }
            if (levels[maxCostOnLevel].Count != 0)
                levels[maxCostOnLevel].First.Value.startOrGoal = true;
            if (toDelete!=null)
            {
                foreach (MDDNode remove in toDelete)
                {
                    remove.delete();
                }
            }
            if (levels[maxCostOnLevel].Count == 0 || levels[0].First.Value.isDeleted == true) //if no possibale rout mark levels as null
                levels = null;
        }

        /// <summary>
        /// Returns all the children of a given MDD node that have a heuristic estimate that is not larger than the given heuristic bound.
        /// </summary>
        /// <param name="father"></param>
        /// <param name="heuristicBound"></param>
        /// <param name="numOfAgents">The number of agents in the MDD node</param>
        /// <returns>A list of relevant MDD nodes</returns>
        private LinkedList<MDDNode> GetAllChildren(MDDNode father, int heuristicBound,int numOfAgents,int i)
        {
            int newX;
            int newY;
            int direction;
            MDDNode child;
            TimedMove move = new TimedMove();

            LinkedList<MDDNode> children = new LinkedList<MDDNode>(); 
            for (int op = 0; op < 5; op++)
            {
                newX = father.getX() + WorldState.operators[op, 0];
                newY = father.getY() + WorldState.operators[op, 1];
                move.setup(newX, newY, op, i + 1);
                if ((this.problem.IsValidForMove(move))&&(this.problem.GetSingleAgentShortestPath(this.agentNum,newX,newY)<=heuristicBound))
                {
                    direction = WorldState.operators[op, 2];
                    child = new MDDNode(newX, newY, i+1, numOfAgents, this);
                    children.AddFirst(child);
                }
            }
            return children;
        }

        /// <summary>
        /// matche and prun MDD according to another MDD
        /// </summary>
        /// <param name="other"></param>
        /// <param name="setUntil"></param>
        /// <returns>0-if the entire tree was pruned 1-if nothing was pruned 2-if something was pruned</returns>
        public int sync3GDDs(MDD other, int setUntil)
        {
            int ans = 1;
            if (this.levels == null || other.levels == null)
                return 0;
            bool validParent;
            LinkedListNode<MDDNode> parent;
            MDDNode parentToDelete;
            LinkedListNode<MDDNode> toSetCoexisting;
            LinkedListNode<MDDNode> tempToSetCoexisting;
            LinkedList<MDDNode> coexitingForNodeLL = new LinkedList<MDDNode>();
            HashSet<MDDNode> coexitingForNodeHS = new HashSet<MDDNode>();

            coexitingForNodeLL.AddFirst(other.levels[0].First.Value);
            levels[0].First.Value.setCoexist(coexitingForNodeLL, other.mddNum);

            for (int i = 1; i < levels.Length; i++)
            {
                toSetCoexisting = levels[i].First;
                while (toSetCoexisting != null && toSetCoexisting.List!=null)
                {
                    if (toSetCoexisting.Value.isDeleted)
                    {
                        tempToSetCoexisting = toSetCoexisting;
                        toSetCoexisting = toSetCoexisting.Next;
                        levels[i].Remove(tempToSetCoexisting);
                        continue;
                    }
                    coexitingForNodeLL = new LinkedList<MDDNode>();
                    coexitingForNodeHS.Clear();
                    

                    parent=toSetCoexisting.Value.parents.First;
                    while (parent!=null)
                    {
                        validParent = false;
                        foreach (MDDNode coexist in parent.Value.coexistLinkedList[other.mddNum])
                        {
                            foreach (MDDNode chield in coexist.children)
                            {
                                if (!toSetCoexisting.Value.Equals(chield))
                                {
                                    if (!parent.Value.EqualsSwitch(chield) || !toSetCoexisting.Value.EqualsSwitch(coexist))
                                    {
                                        if (toSetCoexisting.Value.isCoexistingWithOtherMDDs(chield, other.mddNum))
                                        {
                                            validParent = true;

                                            if (!coexitingForNodeHS.Contains(chield))
                                            {
                                                CostTreeNodeSolver.matchCounter++;
                                                coexitingForNodeHS.Add(chield);
                                                coexitingForNodeLL.AddFirst(chield);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        parentToDelete = parent.Value;
                        parent = parent.Next;
                        if (!validParent)
                        {
                            toSetCoexisting.Value.removeParent(parentToDelete);
                            ans = 2;
                        }
                    }
                    toSetCoexisting.Value.setCoexist(coexitingForNodeLL, other.mddNum);
                    toSetCoexisting.Value.setUntil = setUntil;
                    tempToSetCoexisting = toSetCoexisting;
                    toSetCoexisting = toSetCoexisting.Next;
                    if (tempToSetCoexisting.Value.getCoexistCount(other.mddNum) == 0)
                    {
                        tempToSetCoexisting.Value.delete();
                        ans = 2;
                    }
                }
                if (levels[0].First.Value.children.Count == 0)
                {
                    return 0;
                }
            }
            return ans;
        }

        public int sync2GDDs(MDD other)
        {
            int ans = 1;
            if (this.levels == null || other.levels == null)
                return 0;
            bool validParent;
            LinkedListNode<MDDNode> parent;
            MDDNode parentToDelete;
            LinkedListNode<MDDNode> toSetCoexisting;
            LinkedListNode<MDDNode> tempToSetCoexisting;
            LinkedList<MDDNode> coexitingForNodeLL = new LinkedList<MDDNode>();
            HashSet<MDDNode> coexitingForNodeHS = new HashSet<MDDNode>();

            coexitingForNodeLL.AddFirst(other.levels[0].First.Value);
            levels[0].First.Value.setCoexist(coexitingForNodeLL, other.mddNum);

            for (int i = 1; i < levels.Length; i++)
            {
                toSetCoexisting = levels[i].First;
                while (toSetCoexisting != null && toSetCoexisting.List != null)
                {
                    if (toSetCoexisting.Value.isDeleted)
                    {
                        tempToSetCoexisting = toSetCoexisting;
                        toSetCoexisting = toSetCoexisting.Next;
                        levels[i].Remove(tempToSetCoexisting);
                        continue;
                    }
                    coexitingForNodeLL = new LinkedList<MDDNode>();
                    coexitingForNodeHS.Clear();


                    parent = toSetCoexisting.Value.parents.First;
                    while (parent != null)
                    {
                        validParent = false;
                        foreach (MDDNode coexist in parent.Value.coexistLinkedList[other.mddNum])
                        {
                            foreach (MDDNode chield in coexist.children)
                            {
                                if (!toSetCoexisting.Value.Equals(chield))
                                {
                                    if (!parent.Value.EqualsSwitch(chield) || !toSetCoexisting.Value.EqualsSwitch(coexist))
                                    {
                                        validParent = true;

                                        if (!coexitingForNodeHS.Contains(chield))
                                        {
                                            CostTreeNodeSolver.matchCounter++;
                                            coexitingForNodeHS.Add(chield);
                                            coexitingForNodeLL.AddFirst(chield);
                                        }

                                    }
                                }
                            }
                        }
                        parentToDelete = parent.Value;
                        parent = parent.Next;
                        if (!validParent)
                        {
                            toSetCoexisting.Value.removeParent(parentToDelete);
                            ans = 2;
                        }
                    }
                    toSetCoexisting.Value.setCoexist(coexitingForNodeLL, other.mddNum);
                    tempToSetCoexisting = toSetCoexisting;
                    toSetCoexisting = toSetCoexisting.Next;
                    if (tempToSetCoexisting.Value.getCoexistCount(other.mddNum) == 0)
                    {
                        tempToSetCoexisting.Value.delete();
                        ans = 2;
                    }
                }
                if (levels[0].First.Value.children.Count == 0)
                {
                    return 0;
                }
            }
            return ans;
        }


        public int getMddNum()
        {
            return mddNum;
        }
        public int getAgentNum()
        {
            return mddNum;
        }

        public void printGDD()
        {
            Console.WriteLine();
            Console.WriteLine();
            for (int j = 0; j < levels.Count(); j++)
            {
                Console.WriteLine("\n\nlevel: " + j + " total- " + levels[j].Count+"  *****");
                Console.WriteLine("------------------");
                foreach (MDDNode node in levels[j])
                {
                    Console.Write("\n\n-node- (" + node.getX() + "," + node.getY() + ") children: ");
                    foreach (MDDNode chield in node.children)
                    {
                        Console.Write("(" + chield.getX() + "," + chield.getY() + ") ");
                    }
                    Console.Write(" parents: ");
                    foreach (MDDNode parent in node.parents)
                    {
                        Console.Write("(" + parent.getX() + "," + parent.getY() + ") ");
                    }
                    Console.Write(" coexist: ");
                    int i = 0;
                    foreach (LinkedList<MDDNode> coexistList in node.coexistLinkedList)
                    {
                        Console.Write(" for agent - " + i++);
                        foreach (MDDNode coexist in coexistList)
                        {
                            Console.Write("(" + coexist.getX() + "," + coexist.getY() + ") ");
                        }
                    }
                }
            }
        }

    }
    class MDDNode
    {
        int pos_X;
        int pos_Y;
        public int level;
        public int setUntil;
        public LinkedList<MDDNode> children;
        public LinkedList<MDDNode> parents;
        public LinkedList<MDDNode>[] coexistLinkedList;
        public MDD father;
        LinkedListNode<MDDNode> myNode;
        public bool startOrGoal;
        public bool isDeleted; //to prevent delition loop
        public bool leagel;

        public MDDNode(int pos_X, int pos_Y,int level, int numOfAgents, MDD father)
        {
            this.pos_X = pos_X;
            this.pos_Y = pos_Y;
            this.level = level;
            this.father = father;
            setUntil = 0;
            children = new LinkedList<MDDNode>();
            parents = new LinkedList<MDDNode>();
            coexistLinkedList = new LinkedList<MDDNode>[numOfAgents];
            for (int i = 0; i < numOfAgents; i++)
            {
                coexistLinkedList[i] = new LinkedList<MDDNode>();
            }
        }

        public void delete()
        {
            if (isDeleted)
                return;
            isDeleted = true;
            LinkedListNode<MDDNode> toDelete = parents.First;
            LinkedListNode<MDDNode> nextToDelete;
            while (toDelete!=null)
            {
                nextToDelete = toDelete.Next;
                toDelete.Value.children.Remove(this);
                toDelete.Value.checkAndDelete();
                toDelete = nextToDelete;
            }
            toDelete = children.First;
            while (toDelete != null)
            {
                nextToDelete = toDelete.Next;
                toDelete.Value.parents.Remove(this);
                toDelete.Value.checkAndDelete();
                toDelete = nextToDelete;
            }
           // myNode.List.Remove(myNode);
        }
        public void checkAndDelete()
        {
            if (!isDeleted)
            {
                if (!startOrGoal)
                {
                    if (parents.Count == 0 || children.Count == 0)
                    {
                        this.delete();
                        return;
                    }
                }
                else
                {
                    if (parents.Count == 0 && children.Count == 0)
                    {
                        this.delete();
                        return;
                    }
                }
            }
        }

        public bool isCoexistingWithOtherMDDs(MDDNode toCheck, int otherAgent)
        {
            bool ans = false;
            for (int i = father.getMddNum() + 1 ; i < otherAgent; i++)
            {
                ans = false;
                foreach (MDDNode coexistingForOther in coexistLinkedList[i])
                {
                    if (coexistingForOther.coexistLinkedList[toCheck.father.getMddNum()].Contains(toCheck))
                    {
                        ans = true;
                        break;
                    }
                }
                if (!ans)
                    return false;
            }
            return true;
        }


        public void removeParent(MDDNode parent)
        {
            parent.children.Remove(this);
            this.parents.Remove(parent);
            parent.checkAndDelete();
            this.checkAndDelete(); //remove if there is a bug
        }
        public void addParent(MDDNode parent)
        {
            parents.AddLast(parent);
        }
        public void addChild(MDDNode child)
        {
            children.AddLast(child);
        }
        public void setCoexist(LinkedList<MDDNode> coexists, int agentNum)
        {
            coexistLinkedList[agentNum] = coexists;
        }
        public void setMyNode(LinkedListNode<MDDNode> me)
        {
            myNode = me;
        }
        public int getX()
        {
            return pos_X;
        }
        public int getY()
        {
            return pos_Y;
        }
        public int getVertxIndex()
        {
            return pos_X * CostTreeSearchSolver.maxY + pos_Y;
        }
        public override int GetHashCode()
        {
            return (pos_X + 1) * 3 + (pos_Y + 1) * 5;
        }
        public override bool Equals(object obj)
        {
            MDDNode comp = (MDDNode)obj;
            return this.pos_X == comp.pos_X && this.pos_Y == comp.pos_Y && this.level == comp.level; //if there is a bug return the level compare
        }
        public bool EqualsSwitch(object obj)
        {
            MDDNode comp = (MDDNode)obj;
            return this.pos_X == comp.pos_X && this.pos_Y == comp.pos_Y ;
        }
        public int getCoexistCount(int otherAgent)
        {
            return coexistLinkedList[otherAgent].Count;
        }
        public int getDirection(MDDNode other)
        {
            int ans = 0;
            ans += this.pos_X - other.pos_X + 1;
            ans += 2 * (this.pos_Y - other.pos_Y + 1);
            return ans - 1;
        }
    }
}
