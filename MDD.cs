using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// A compact representation of all the paths to the goal of a given agent that are of a given cost and length
    /// </summary>
    public class MDD
    {
        public LinkedList<MDDNode>[] levels;
        private int agentNum;
        private int mddNum;
        /// <summary>
        /// Just for printing the node
        /// </summary>
        private int cost;
        public ProblemInstance problem;
        public enum PruningDone : int
        {
            EVERYTHING = 0,
            NOTHING = 1,
            SOME = 2
        }
        protected HashSet_U<CbsConstraint> constraints;
        protected Dictionary<int, TimedMove>[] mustConstraints;
        protected bool supportPruning;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="mddNum"></param>
        /// <param name="agentNum"></param>
        /// <param name="start_pos"></param>
        /// <param name="cost">The MDD must be of this cost</param>
        /// <param name="numOfLevels">
        /// The MDD must be of this number of levels, not counting level zero.
        /// If higher than cost, the extra levels will be WAITs at the goal.
        /// </param>
        /// <param name="numOfAgents"></param>
        /// <param name="instance"></param>
        /// <param name="ignoreConstraints"></param>
        public MDD(int mddNum, int agentNum, Move start_pos, int cost, int numOfLevels, int numOfAgents, ProblemInstance instance, bool ignoreConstraints = false, bool supportPruning = true)
        {
            this.problem = instance;
            this.mddNum = mddNum;
            this.agentNum = agentNum;
            this.cost = cost;
            this.levels = new LinkedList<MDDNode>[numOfLevels + 1];
            this.supportPruning = supportPruning;

            if (ignoreConstraints == false && instance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS) &&
                    ((HashSet_U<CbsConstraint>)instance.parameters[CBS_LocalConflicts.CONSTRAINTS]).Count != 0)
            {
                this.queryConstraint = new CbsConstraint();
                this.queryConstraint.queryInstance = true;

                this.constraints = (HashSet_U<CbsConstraint>)instance.parameters[CBS_LocalConflicts.CONSTRAINTS];
            }

            if (ignoreConstraints == false && instance.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS) &&
                 ((HashSet_U<CbsConstraint>)instance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS]).Count != 0)
            {
                // TODO: Code dup with ClassicAStar's constructor
                var musts = (HashSet_U<CbsConstraint>)instance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                this.mustConstraints = new Dictionary<int, TimedMove>[musts.Max<CbsConstraint>(con => con.GetTimeStep()) + 1]; // To have index MAX, array needs MAX + 1 places.
                foreach (CbsConstraint con in musts)
                {
                    int timeStep = con.GetTimeStep();
                    if (this.mustConstraints[timeStep] == null)
                        this.mustConstraints[timeStep] = new Dictionary<int, TimedMove>();
                    this.mustConstraints[timeStep][con.agentNum] = con.move;
                }
            }

            var perLevelClosedList = new Dictionary<MDDNode, MDDNode>();
            var toDelete = new List<MDDNode>();

            for (int i = 0; i <= numOfLevels; i++)
            {
                levels[i] = new LinkedList<MDDNode>();
            }
            MDDNode root = new MDDNode(new TimedMove(start_pos, 0) , numOfAgents, this, supportPruning); // Root
            LinkedListNode<MDDNode> llNode = new LinkedListNode<MDDNode>(root);
            root.setMyNode(llNode);
            llNode.Value.startOrGoal = true;
            levels[0].AddFirst(llNode);

            for (int i = 0; i < numOfLevels ; i++) // For each level, populate the _next_ level
            {
                int heuristicBound = cost - i - 1; // We want g+h <= cost, so h <= cost-g. -1 because it's the bound of the _children_.
                if (heuristicBound < 0)
                    heuristicBound = 0;

                // Go over each MDDNode in this level
                foreach (MDDNode currentMddNode in levels[i]) // Since we're not deleting nodes in this method, we can use the simpler iteration method :)
                {
                    List<MDDNode> children = this.GetAllChildren(currentMddNode, heuristicBound, numOfAgents);
                    if (children.Count == 0) // Heuristic wasn't perfect because of constraints, illegal moves or other reasons
                        toDelete.Add(currentMddNode);

                    foreach (MDDNode child in children)
                    {
                        MDDNode toAdd = child; // The compiler won't let me assign to the foreach variable...
                        if (perLevelClosedList.ContainsKey(child))
                        {
                            toAdd = perLevelClosedList[child];
                        }
                        else
                        {
                            perLevelClosedList.Add(toAdd, toAdd);
                            llNode = new LinkedListNode<MDDNode>(toAdd);
                            toAdd.setMyNode(llNode);
                            levels[i + 1].AddLast(llNode);
                        }
                        currentMddNode.addChild(toAdd); // forward edge
                        toAdd.addParent(currentMddNode); // backward edge
                    }
                }
                perLevelClosedList.Clear();
            }

            foreach (MDDNode goal in levels[numOfLevels]) // The goal may be reached in more than one direction
            {
                goal.startOrGoal = true;
            }

            foreach (MDDNode remove in toDelete)
            {
                remove.delete();
            }
            
            // Make sure the goal was reached - imperfect heuristics, constraints or illegal moves can cause this to be false.
            if (levels[numOfLevels].Count == 0) // No possible route to goal was found
                levels = null;
        }

        /// <summary>
        /// Just an optimization
        /// </summary>
        private CbsConstraint queryConstraint;

        /// <summary>
        /// Returns all the children of a given MDD node that have a heuristic estimate that is not larger than the given heuristic bound.
        /// </summary>
        /// <param name="mdd"></param>
        /// <param name="heuristicBound">The heuristic estimate of the returned children must be lower than or equal to the bound</param>
        /// <param name="numOfAgents">The number of agents in the MDD node</param>
        /// <returns>A list of relevant MDD nodes</returns>
        private List<MDDNode> GetAllChildren(MDDNode father, int heuristicBound, int numOfAgents)
        {
            var children = new List<MDDNode>(); 
            foreach (TimedMove move in father.move.GetNextMoves())
            {
                if (this.problem.IsValid(move) &&
                    this.problem.GetSingleAgentOptimalCost(this.agentNum, move) <= heuristicBound) // Only nodes that can reach the goal in the given cost according to the heuristic.
                {
                    if (this.constraints != null)
                    {
                        queryConstraint.Init(agentNum, move);

                        if (this.constraints.Contains(queryConstraint))
                            continue;
                    }

                    if (this.mustConstraints != null && move.time < this.mustConstraints.Length && // There may be a constraint on the timestep of the generated node
                        this.mustConstraints[move.time] != null &&
                        this.mustConstraints[move.time].ContainsKey(this.agentNum)) // This agent has a must constraint for this time step
                    {
                        if (this.mustConstraints[move.time][this.agentNum].Equals(move) == false)
                            continue;
                    }

                    MDDNode child = new MDDNode(move, numOfAgents, this, this.supportPruning);
                    children.Add(child);
                }
            }
            return children;
        }

        /// <summary>
        /// Match and prune MDD according to another MDD.
        /// </summary>
        /// <param name="other"></param>
        /// <param name="checkTriples">If true, use the "3E" method.</param>
        public PruningDone SyncMDDs(MDD other, bool checkTriples)
        {
            PruningDone ans = PruningDone.NOTHING;
            if (this.levels == null || other.levels == null) // Either of the MDDs was already completely pruned already
                return PruningDone.EVERYTHING;

            // Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
            var coexistingNodesForLevelZero = new HashSet<MDDNode>();
            coexistingNodesForLevelZero.Add(other.levels[0].First.Value);
            levels[0].First.Value.SetCoexistingNodes(coexistingNodesForLevelZero, other.mddNum);

            for (int i = 1; i < levels.Length; i++)
            {
                foreach (var node in levels[i])
                {
                    var coexistingForNode = new HashSet<MDDNode>();

                    // Go over all the node's parents and test their coexisting nodes' children for coexistance with this node
                    foreach (var parent in node.parents)
                    {
                        bool validParent = false;
                        foreach (MDDNode parentCoexistingNode in parent.coexistingNodesFromOtherMdds[other.mddNum])
                        {
                            foreach (MDDNode childOfParentCoexistingNode in parentCoexistingNode.children)
                            {
                                if (node.move.IsColliding(childOfParentCoexistingNode.move) == false)
                                {
                                    if (checkTriples == false ||
                                        node.IsCoexistingWithOtherMDDs(childOfParentCoexistingNode, other.mddNum)) // The "3" part
                                    {
                                        validParent = true;

                                        if (coexistingForNode.Contains(childOfParentCoexistingNode) == false)
                                        {
                                            CostTreeNodeSolver.matchCounter++;
                                            coexistingForNode.Add(childOfParentCoexistingNode);
                                        }
                                    }
                                }
                            }
                        }
                        if (!validParent)
                        {
                            node.removeParent(parent); // And continue up the levels if necessary
                            ans = PruningDone.SOME;
                        }
                    }
                    node.SetCoexistingNodes(coexistingForNode, other.mddNum);
                    if (node.getCoexistingNodesCount(other.mddNum) == 0)
                    {
                        node.delete();
                        ans = PruningDone.SOME;
                    }
                }
                if (levels[0].Count == 0)
                {
                    return PruningDone.EVERYTHING;
                }
            }
            return ans;
        }

        // TODO: Make a Combine method to multiply with another MDD.

        public int getMddNum()
        {
            return mddNum;
        }

        public int getAgentNum()
        {
            return mddNum;
        }

        public void DebugPrint()
        {
            Debug.WriteLine($"MDD for agent {this.agentNum}, {this.levels.Length} steps, cost {this.cost}:");
            for (int j = 0; j < levels.Count(); j++)
            {
                Debug.WriteLine($"Level {j}, {levels[j].Count} nodes:");
                foreach (MDDNode node in levels[j])
                {
                    Debug.Write($"Node {node}");
                    Debug.Write(" children: ");
                    foreach (MDDNode child in node.children)
                    {
                        Debug.Write($"{child}, ");
                    }
                    Debug.Write(" parents: ");
                    foreach (MDDNode parent in node.parents)
                    {
                        Debug.Write($"{parent}, ");
                    }
                    Debug.Write(" coexist: ");
                    int i = 0;
                    foreach (HashSet<MDDNode> coexistingNodesFromOtherMdds in node.coexistingNodesFromOtherMdds)
                    {
                        Debug.Write($" for agents - {i++}");
                        foreach (MDDNode coexistingNode in coexistingNodesFromOtherMdds)
                        {
                            Debug.Write($"{coexistingNode}) ");
                        }
                    }
                }
            }
            Debug.WriteLine("------");
        }

    }

    [DebuggerDisplay("{move}")]
    public class MDDNode
    {
        public TimedMove move;
        public LinkedList<MDDNode> children;
        public LinkedList<MDDNode> parents;
        public HashSet<MDDNode>[] coexistingNodesFromOtherMdds;
        public MDD mdd;
        LinkedListNode<MDDNode> myNode;
        public bool startOrGoal;
        public bool isBeingDeleted; // Not actually needed
        public bool legal;  // For AstarMDD

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return this.move.ToString();
        }

        public MDDNode(TimedMove move, int numOfAgents, MDD mdd, bool supportPruning = true)
        {
            this.move = move;
            this.mdd = mdd;
            children = new LinkedList<MDDNode>();
            parents = new LinkedList<MDDNode>();
            if (supportPruning)
            {
                coexistingNodesFromOtherMdds = new HashSet<MDDNode>[numOfAgents];
                for (int i = 0; i < numOfAgents; i++)
                {
                    coexistingNodesFromOtherMdds[i] = new HashSet<MDDNode>();
                }
            }
        }

        public void delete()
        {
            Debug.Assert(this.isBeingDeleted == false, "This is unexpected");
            this.isBeingDeleted = true;
            LinkedListNode<MDDNode> toDelete = parents.First;
            LinkedListNode<MDDNode> nextToDelete;
            while (toDelete != null)
            {
                nextToDelete = toDelete.Next;
                toDelete.Value.children.Remove(this);
                toDelete.Value.deleteIfOrphanOrChildless();
                toDelete = nextToDelete;
            }
            toDelete = children.First;
            while (toDelete != null)
            {
                nextToDelete = toDelete.Next;
                toDelete.Value.parents.Remove(this);
                toDelete.Value.deleteIfOrphanOrChildless();
                toDelete = nextToDelete;
            }
            myNode.List.Remove(myNode);
        }
        
        public void deleteIfOrphanOrChildless()
        {
            Debug.Assert(this.isBeingDeleted == false, "unexpected");
            if (!this.startOrGoal)
            {
                if (parents.Count == 0 || children.Count == 0)
                {
                    this.delete();
                }
            }
            else
            {
                if (parents.Count == 0 && children.Count == 0) // A goal node has no children to begin with, a start node has no parents.
                {
                    this.delete();
                }
            }
        }

        /// <summary>
        /// Checks if given MDD node coexists with all nodes from earlier checked MDDs that were
        /// previously found to coexist with this node.
        /// Assumes before MDDs i,j are checked for coexistence, all MDDs k such that i &lt; k &lt; j
        /// were checked.
        /// </summary>
        /// <param name="toCheck"></param>
        /// <param name="otherAgentIndex"></param>
        /// <returns></returns>
        public bool IsCoexistingWithOtherMDDs(MDDNode toCheck, int otherAgentIndex)
        {
            Debug.Assert(toCheck.mdd.getMddNum() == otherAgentIndex, "unexpected");
            bool ans = false;
            for (int i = this.mdd.getMddNum() + 1 ; i < otherAgentIndex; i++)
            {
                ans = false;
                foreach (MDDNode coexistingNode in this.coexistingNodesFromOtherMdds[i])
                {
                    if (coexistingNode.coexistingNodesFromOtherMdds[toCheck.mdd.getMddNum()].Contains(toCheck))
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
            parent.deleteIfOrphanOrChildless();
            this.deleteIfOrphanOrChildless();
        }
        
        public void addParent(MDDNode parent)
        {
            parents.AddLast(parent);
        }
        
        public void addChild(MDDNode child)
        {
            children.AddLast(child);
        }
        
        public void SetCoexistingNodes(HashSet<MDDNode> coexistingNodes, int mddNum)
        {
            Debug.Assert(coexistingNodes.All<MDDNode>(node => node.mdd.getMddNum() == mddNum), "unexpected");
            this.coexistingNodesFromOtherMdds[mddNum] = coexistingNodes;
        }
        
        public void setMyNode(LinkedListNode<MDDNode> node)
        {
            this.myNode = node;
        }
        
        public int getVertexIndex()
        {
            return move.x * CostTreeSearchSolver.maxY + move.y;
        }
        
        /// <summary>
        /// Only uses the move
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                return move.GetHashCode();
            }
        }
        
        /// <summary>
        /// Only uses the move
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            MDDNode comp = (MDDNode)obj;
            return this.move.Equals(comp.move); //if there is a bug return the level compare
                                                // Behavior change: used to not compare the direction
        }
        
        public int getCoexistingNodesCount(int otherAgent)
        {
            return coexistingNodesFromOtherMdds[otherAgent].Count;
        }
        
        ///// <summary>
        ///// TODO: This doesn't seem... correct. Also, there's a similar static method in class Move
        ///// </summary>
        ///// <param name="other"></param>
        ///// <returns></returns>
        //public int getDirection(MDDNode other)
        //{
        //    int ans = 0;
        //    ans += this.move.x - other.move.x + 1;
        //    ans += 2 * (this.move.y - other.move.y + 1);
        //    return ans - 1;
        //}
    }
}
