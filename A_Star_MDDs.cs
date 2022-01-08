using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using ExtensionMethods;

namespace mapf;

/// <summary>
/// Finds the solution with the least number of conflicts, given a set of MDDs
/// </summary>
class A_Star_MDDs : IConflictReporting
{
    MDD[] problem;
    Dictionary<A_Star_MDDs_Node, A_Star_MDDs_Node> closedList;
    Run runner;
    BinaryHeap<A_Star_MDDs_Node> openList;
    public int expanded;
    public int generated;
    public int conflictCount;
    ConflictAvoidanceTable CAT;

    public A_Star_MDDs(MDD[] problem, Run runner, ConflictAvoidanceTable CAT)
    {
        this.expanded = 0;
        this.generated = 0;
        A_Star_MDDs_Node root;
        this.problem = problem;
        this.runner = runner;
        this.CAT = CAT;
        this.closedList = new Dictionary<A_Star_MDDs_Node, A_Star_MDDs_Node>();
        this.openList = new BinaryHeap<A_Star_MDDs_Node>();
        MDDNode[] sRoot = new MDDNode[problem.Length];
        for (int i = 0; i < problem.Length; i++)
        {
            sRoot[i] = problem[i].levels[0].First.Value;
        }
        root = new A_Star_MDDs_Node(sRoot, null);
        openList.Add(root);
        closedList.Add(root, root); // There will never be a hit. This is only done for consistancy
        conflictCount = 0;
    }
       
    public SinglePlan[] Solve()
    {
        A_Star_MDDs_Node currentNode;
        //A_Star_MDDs_Expander expander = new A_Star_MDDs_Expander();

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
                this.conflictCount = currentNode.conflictCount;
                this.conflictCounts = currentNode.conflictCounts;
                this.conflictTimes = currentNode.conflictTimes;
                return GetAnswer(currentNode);
            }

            // Expand
            expanded++;  // TODO: don't count re-expansions as expansions?
            Expand(currentNode);
            //expander.Setup(currentNode);
            //Expand(expander);  // TODO: the expander just generates all children. EPEA* its ass!!
        }
        return null;
    }

    public void Expand(A_Star_MDDs_Node node)
    {
        if (node.IsAlreadyExpanded() == false)
        {
            node.calcSingleAgentDeltaConflictCounts(this.CAT);
            node.alreadyExpanded = true;
            node.targetDeltaConflictCount = 0;
            node.remainingDeltaConflictCount = node.targetDeltaConflictCount; // Just for the following hasChildrenForCurrentDeltaConflictCount call.
            while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount() == false) // DeltaConflictCount==0 may not be possible if all agents have obstacles between their location and the goal
            {
                node.targetDeltaConflictCount++;
                node.remainingDeltaConflictCount = node.targetDeltaConflictCount;
            }
            if (node.hasMoreChildren() == false) // Node has no possible children at all
            {
                node.ClearExpansionData();
                return;
            }
        }

        var intermediateNodes = new List<A_Star_MDDs_Node>() { node };

        for (int mddIndex = 0; mddIndex < this.problem.Length && intermediateNodes.Count != 0; ++mddIndex)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                return;

            intermediateNodes = ExpandOneAgent(intermediateNodes, mddIndex);
        }

        var finalGeneratedNodes = intermediateNodes;

        foreach (var child in finalGeneratedNodes)
        {
            child.conflictCount = node.conflictCount + node.targetDeltaConflictCount;

            // Accumulating the conflicts count from parent to child
            // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
            child.conflictCounts = new Dictionary<int, int>(child.prev.conflictCounts);
            child.conflictTimes = new Dictionary<int, List<int>>();
            foreach (var kvp in child.prev.conflictTimes)
                child.conflictTimes[kvp.Key] = new List<int>(kvp.Value);
            child.IncrementConflictCounts(this.CAT);  // We're counting conflicts along the entire path, so the parent's conflicts count
                                                    // is added to the child's.

            bool was_closed = this.closedList.ContainsKey(child);
            if (was_closed)
            {
                A_Star_MDDs_Node inClosedList = this.closedList[child];

                if (inClosedList.conflictCount > child.conflictCount)
                {
                    closedList.Remove(inClosedList);
                    openList.Remove(inClosedList);
                    was_closed = false;
                }
            }
            if (!was_closed)
            {
                this.openList.Add(child);
                this.closedList.Add(child, child);
                generated++;
            }
        }

        // Prepare the node for the next partial expansion:
        if (node.IsAlreadyExpanded() == false)
        {
            // Node was cleared during expansion.
            // It's unnecessary and unsafe to continue to prepare it for the next partial expansion.
            return;
        }

        node.targetDeltaConflictCount++; // This delta F was exhausted
        node.remainingDeltaConflictCount = node.targetDeltaConflictCount;

        while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount() == false)
        {
            node.targetDeltaConflictCount++;
            node.remainingDeltaConflictCount = node.targetDeltaConflictCount; // Just for the following hasChildrenForCurrentDeltaF call.
        }

        if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaConflictCount())
        {
            // Re-insert node into open list
            openList.Add(node);
        }
        else
            node.ClearExpansionData();
    }

    protected List<A_Star_MDDs_Node> ExpandOneAgent(List<A_Star_MDDs_Node> intermediateNodes, int mddIndex)
    {
        var generated = new List<A_Star_MDDs_Node>();

        // Expand the mdd node
        foreach (A_Star_MDDs_Node node in intermediateNodes)
        {
            // Try all the children of this MDD node
            foreach ((int childIndex, MDDNode childMddNode) in node.allSteps[mddIndex].children.Enumerate())
            {
                if (node.currentMoves != null && childMddNode.move.IsColliding(node.currentMoves))  // Can happen. We only prune partially, we don't build the full k-agent MDD.
                    continue;

                var childNode = new A_Star_MDDs_Node(node, mddIndex != node.allSteps.Length - 1);
                childNode.allSteps[mddIndex] = childMddNode;

                // Update target conflict count and prune nodes that can't get to the target conflict count
                childNode.UpdateRemainingDeltaConflictCount(mddIndex, childIndex);
                if (childNode.remainingDeltaConflictCount == ushort.MaxValue || // Last move was bad - not sure this can happen here
                    (childNode.hasChildrenForCurrentDeltaConflictCount(mddIndex + 1) == false))  // No children that can reach the target
                    continue;

                if (mddIndex < node.allSteps.Length - 1) // More MDD nodes need to choose a child
                    childNode.currentMoves.Add(childMddNode.move);
                else // Moved the last agent
                    childNode.currentMoves = null; // To reduce memory load and lookup times

                // Set the node's prev to its real parent, skipping over the intermediate nodes.
                if (mddIndex != 0)
                    childNode.prev = node.prev;
                else
                    childNode.prev = node;

                generated.Add(childNode);
            }
        }
            
        return generated;
    }

    protected Dictionary<int, int> conflictCounts;
    protected Dictionary<int, List<int>> conflictTimes;

    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to the number of conflicts with their path the solution has</returns>
    public Dictionary<int, int> GetExternalConflictCounts()
    {
        return this.conflictCounts;
    }

    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to a list of times the solution has a conflict with theirs</returns>
    public Dictionary<int, List<int>> GetConflictTimes()
    {
        return this.conflictTimes;
    }

    public void Expand(A_Star_MDDs_Expander currentNode)
    {
        while (true)
        {
            A_Star_MDDs_Node child = currentNode.GetNextChild();
            if (child == null)
                break;

            if (IsLegalMove(child))
            {
                child.conflictCount = child.prev.conflictCount;
                child.UpdateConflicts(CAT);

                bool was_closed = this.closedList.ContainsKey(child);
                if (was_closed)
                {
                    A_Star_MDDs_Node inClosedList = this.closedList[child];

                    if (inClosedList.conflictCount > child.conflictCount)
                    {
                        closedList.Remove(inClosedList);
                        openList.Remove(inClosedList);
                        was_closed = false;
                    }
                }
                if (!was_closed)
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
        
    private bool GoalTest(A_Star_MDDs_Node toCheck)
    {
        if (toCheck.GetDepth() == problem[0].levels.Length - 1)
            return true;
        return false;
    }

    private SinglePlan[] GetAnswer(A_Star_MDDs_Node finish)
    {
        // TODO: Move the construction of the SinglePlans to a static method in SinglePlan
        var routes = new LinkedList<Move>[problem.Length];
        for (int i = 0; i < routes.Length; i++)
            routes[i] = new LinkedList<Move>();

        A_Star_MDDs_Node current = finish;
        while (current != null)
        {
            for (int i = 0; i < problem.Length; i++)
            {
                routes[i].AddFirst(new Move(current.allSteps[i].move));
            }
            current = current.prev;
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
        
    private bool IsLegalMove(A_Star_MDDs_Node to)
    {
        if (to == null)
            return false;
        if (to.prev == null)
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

class A_Star_MDDs_Node : IComparable<IBinaryHeapItem>, IBinaryHeapItem
{
    /// <summary>
    /// The last move of all agents that have already moved in this turn.
    /// Used for making sure the next agent move doesn't collide with moves already made.
    /// Used while generating this node, nullified when done.
    /// </summary>
    public HashSet<TimedMove> currentMoves;
    public MDDNode[] allSteps;
    public A_Star_MDDs_Node prev;
    public int conflictCount;
    public Dictionary<int, int> conflictCounts;
    public Dictionary<int, List<int>> conflictTimes;
    int binaryHeapIndex;

    public bool alreadyExpanded;
    /// <summary>
    /// Starts at zero, incremented after a node is expanded once. Set on Expand.
    /// </summary>
    public ushort targetDeltaConflictCount = 0;
    /// <summary>
    /// Remaining delta conflict count towards targetDeltaConflictCount. Reset on Expand.
    /// </summary>
    public ushort remainingDeltaConflictCount;
    /// <summary>
    /// For each MDD node and each child it has, the effect of that choosing that child on the conflict count.
    /// byte.MaxValue means this is an illegal move. Only computed on demand.
    /// </summary>
    protected byte[][] singleAgentDeltaConflictCounts;
    /// <summary>
    /// Only computed on demand
    /// </summary>
    protected ushort maxDeltaConflictCount;
    /// <summary>
    /// Per each MDD node and delta conflict count, has 1 if that delta F is achievable by choosing child MDD nodes starting from this one on,
    /// -1 if it isn't, and 0 if we don't know yet.
    /// Only computed on demand
    /// </summary>
    protected sbyte[][] conflictCountLookup;

    /// <summary>
    /// From generated nodes. Allows expansion table to be garbage collected before all generated nodes are expanded.
    /// </summary>
    public void ClearExpansionData()
    {
        this.singleAgentDeltaConflictCounts = null;
        this.conflictCountLookup = null;
        this.currentMoves = null;
    }

    /// <summary>
    /// Counts the number of times this node collides with each agent move in the conflict avoidance table.
    /// </summary>
    /// <param name="CAT"></param>
    /// <returns></returns>
    public virtual void IncrementConflictCounts(ConflictAvoidanceTable CAT)
    {
        foreach (var mddNode in this.allSteps)
        {
            mddNode.move.IncrementConflictCounts(CAT, this.conflictCounts, this.conflictTimes);
        }
    }

    /// <summary>
    /// Returns whether all possible f values were generated from this node already
    /// </summary>
    /// <returns></returns>
    public bool hasMoreChildren()
    {
        return this.targetDeltaConflictCount <= this.maxDeltaConflictCount;
    }

    public bool IsAlreadyExpanded()
    {
        return alreadyExpanded;
    }

    public bool hasChildrenForCurrentDeltaConflictCount(int agentNum = 0)
    {
        return existsChildForConflictCount(agentNum, this.remainingDeltaConflictCount);
    }

    /// <summary>
    /// An MDD child node was chosen between calculating the singleAgentDeltaConflictCounts and this call.
    /// Using the data that describes its delta conflict count potential before the move.
    /// </summary>
    /// <param name="mddIndex">which mdd's node was expanded to create this A*_MDDs node</param>
    /// <param name="childIndex">which of the mdd node's children was just chosen</param>
    public void UpdateRemainingDeltaConflictCount(int mddIndex, int childIndex)
    {
        if (this.remainingDeltaConflictCount == ushort.MaxValue)
            Trace.Assert(false,
                            $"Remaining deltaConflictCount is ushort.MaxValue, a reserved value with special meaning. agentIndex={mddIndex}");

        byte lastMoveDeltaConflictCount = this.singleAgentDeltaConflictCounts[mddIndex][childIndex];
        if (lastMoveDeltaConflictCount != byte.MaxValue && this.remainingDeltaConflictCount >= lastMoveDeltaConflictCount)
            this.remainingDeltaConflictCount -= lastMoveDeltaConflictCount;
        else
            this.remainingDeltaConflictCount = ushort.MaxValue; // Either because last move was illegal or because the delta F from the last move was more than the entire remaining delta F budget
    }

    /// <summary>
    /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
    /// </summary>
    /// <param name="mddIndex"></param>
    /// <param name="remainingTargetDeltaF"></param>
    /// <returns></returns>
    protected bool existsChildForConflictCount(int mddIndex, ushort remainingTargetDeltaConflictCount)
    {
        // Stopping conditions:
        if (mddIndex == this.allSteps.Length)
        {
            if (remainingTargetDeltaConflictCount == 0)
                return true;
            return false;
        }

        if (conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] != 0) // Answer known (arrays are initialized to zero). TODO: Replace the magic.
        {
            return conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] == 1; // Return known answer. TODO: Replace the magic
        }

        // Recursive actions:
        for (int i = 0; i < this.allSteps[mddIndex].children.Count; i++)
        {
            if (singleAgentDeltaConflictCounts[mddIndex][i] > remainingTargetDeltaConflictCount) // Small optimization - no need to make the recursive
                                                                                                // call just to request a negative target from it and
                                                                                                // get false (because we assume the heuristic function
                                                                                                // is consistent)
                continue;
            if (existsChildForConflictCount(mddIndex + 1,
                                            (byte)(remainingTargetDeltaConflictCount - singleAgentDeltaConflictCounts[mddIndex][i])))
            {
                conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] = 1;
                return true;
            }
        }
        conflictCountLookup[mddIndex][remainingTargetDeltaConflictCount] = -1;
        return false;
    }

    /// <summary>
    /// Calculates for each MDD node and each of its children, the effect of that move on the conflict count.
    /// Also calcs maxDeltaConflictCount.
    /// Note: Currently avoids the CAT's avoidanceGoal. To compute each individual agent's effect on the count of groups we conflict with would require
    /// tracking conflicts as a set of groups we conflict with instead of as a sum of conflicts, and would require 2^(num agents) cells in each singleAgentDeltaConflictCounts[i].
    /// </summary>
    /// <param name="CAT"></param>
    /// <returns></returns>
    public void calcSingleAgentDeltaConflictCounts(ConflictAvoidanceTable CAT)
    {
        // Init
        this.singleAgentDeltaConflictCounts = new byte[this.allSteps.Length][];
        for (int i = 0; i < this.allSteps.Length; i++)
        {
            this.singleAgentDeltaConflictCounts[i] = new byte[this.allSteps[i].children.Count];
        }

        int conflictCountAfter;

        this.maxDeltaConflictCount = 0;

        // Set values
        for (int i = 0; i < this.allSteps.Length; i++)
        {
            int singleAgentMaxLegalDeltaConflictCount = -1;

            foreach ((int childIndex, MDDNode child) in this.allSteps[i].children.Enumerate())
            {
                if (CAT != null)
                {
                    conflictCountAfter = CAT[child.move].Count;
                }
                else
                    conflictCountAfter = 0;

                singleAgentDeltaConflictCounts[i][childIndex] = (byte)conflictCountAfter;
                singleAgentMaxLegalDeltaConflictCount = Math.Max(singleAgentMaxLegalDeltaConflictCount, singleAgentDeltaConflictCounts[i][childIndex]);
            }

            if (singleAgentMaxLegalDeltaConflictCount == -1) // No legal action for this agent, so no legal children exist for this node
            {
                this.maxDeltaConflictCount = 0; // Can't make it negative without widening the field.
                break;
            }

            this.maxDeltaConflictCount += (byte)singleAgentMaxLegalDeltaConflictCount;
        }

        conflictCountLookup = new sbyte[this.allSteps.Length][];
        for (int i = 0; i < conflictCountLookup.Length; i++)
        {
            conflictCountLookup[i] = new sbyte[this.maxDeltaConflictCount + 1];  // Towards the last agents most of the row will be wasted (the last one can do delta F of 0 or 1),
                                                                                    // but it's easier than fiddling with array sizes
        }
    }

    public A_Star_MDDs_Node(MDDNode[] allSteps, A_Star_MDDs_Node prevStep)
    { 
        this.allSteps = allSteps;
        this.prev = prevStep;
        this.currentMoves = null;  // All non-intermediate nodes have currentMoves == null
        this.conflictCount = 0;

        // Initialize conflict tracking data structures
        this.conflictCounts = new Dictionary<int, int>();
        this.conflictTimes = new Dictionary<int, List<int>>();
    }

    /// <summary>
    /// Copy constructor
    /// </summary>
    public A_Star_MDDs_Node(A_Star_MDDs_Node cpy, bool createIntermediate)
    {
        this.allSteps = new MDDNode[cpy.allSteps.Length];
        for (int i = 0; i < allSteps.Length; i++)
        {
            this.allSteps[i] = cpy.allSteps[i];
        }
        this.prev = cpy.prev;
        if (cpy.currentMoves != null)
        {
            // cpy is an intermediate node
            if (createIntermediate)
                this.currentMoves = new HashSet<TimedMove>(cpy.currentMoves);
            else
                this.currentMoves = cpy.currentMoves;  // We're not going to add anything currentMoves
        }
        else
            // cpy is a concrete node
            this.currentMoves = new HashSet<TimedMove>(capacity: cpy.allSteps.Length);

        // The conflictTimes and conflictCounts are only copied later if necessary.

        alreadyExpanded = false; // Creating a new unexpanded node from cpy

        // For intermediate nodes created during expansion (fully expanded nodes have these fields recalculated when they're expanded)
        targetDeltaConflictCount = cpy.targetDeltaConflictCount;  // Just to ease debugging
        remainingDeltaConflictCount = cpy.remainingDeltaConflictCount;
        singleAgentDeltaConflictCounts = cpy.singleAgentDeltaConflictCounts; // For the UpdateRemainingDeltaConflictCount call on temporary nodes.
                                                                                // Notice that after an agent is moved its row won't be up-to-date.
        conflictCountLookup = cpy.conflictCountLookup; // For the hasChildrenForCurrentDeltaConflictCount call on temporary nodes.
                                                        // Notice that after an agent is moved, all rows up to and including the one of the agent that moved
                                                        // won't be up-to-date.
        maxDeltaConflictCount = cpy.maxDeltaConflictCount; // Not necessarily achievable after some of the agents moved.
                                                            // The above is OK because we won't be using data for agents that already moved.
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
        A_Star_MDDs_Node comp = (A_Star_MDDs_Node)obj;
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
    /// Updates the conflictCount member according to given CATs. Table may be null.
    /// </summary>
    /// <param name="CAT"></param>
    public void UpdateConflicts(ConflictAvoidanceTable CAT)
    {
        if (this.prev == null)
            return;
        if (CAT != null)
        {
            for (int i = 0; i < allSteps.Length; i++)
            {
                if (CAT.ContainsKey(allSteps[i].move))
                    conflictCount += CAT[allSteps[i].move].Count;
            }
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

    /// <summary>
    /// Prefers fewer conflicts. If the number of conflicts is the same, prefers more depth.
    /// </summary>
    /// <param name="other"></param>
    /// <returns></returns>
    public int CompareTo(IBinaryHeapItem other)
    {
        A_Star_MDDs_Node that = (A_Star_MDDs_Node)other;
        if (this.conflictCount + this.targetDeltaConflictCount < that.conflictCount + that.targetDeltaConflictCount)
            return -1;
        if (this.conflictCount + this.targetDeltaConflictCount > that.conflictCount + that.targetDeltaConflictCount)
            return 1;

        if (this.GetDepth() > that.GetDepth())
            return -1;
        if (this.GetDepth() < that.GetDepth())
            return 1;

        return 0;
    }
}

class A_Star_MDDs_Expander
{
    private A_Star_MDDs_Node a_star_mdd_node;
    /// <summary>
    /// For each MDD, the index of the next child in <children> to choose
    /// </summary>
    int[] chosenChild;

    public A_Star_MDDs_Expander() { }

    public A_Star_MDDs_Expander(A_Star_MDDs_Node a_star_mdd_node)
    {
        this.a_star_mdd_node = a_star_mdd_node;
        this.chosenChild = new int[a_star_mdd_node.allSteps.Length];
        foreach (MDDNode mddNode in a_star_mdd_node.allSteps)
        {
            if (mddNode.children.Count == 0)
            {
                chosenChild[0] = -1;
                break;
            }
        }
    }

    public void Setup(A_Star_MDDs_Node a_star_mdd_node)
    {
        this.a_star_mdd_node = a_star_mdd_node;
        this.chosenChild = new int[a_star_mdd_node.allSteps.Length];
        foreach (MDDNode mddNode in a_star_mdd_node.allSteps)
        {
            if (mddNode.children.Count == 0)
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
    public A_Star_MDDs_Node GetNextChild()
    {
        if (this.chosenChild[0] == -1)
            return null;
        var mddNodes = new MDDNode[a_star_mdd_node.allSteps.Length];
        for (int i = 0; i < mddNodes.Length; i++)
        {
            mddNodes[i] = a_star_mdd_node.allSteps[i].children.ElementAt(this.chosenChild[i]);
        }
        SetNextChildIndices();
        return new A_Star_MDDs_Node(mddNodes, a_star_mdd_node);
    }

    /// <summary>
    /// Increments the chosenChild indices array by 1
    /// </summary>
    private void SetNextChildIndices()
    {
        SetNextChildIndices(this.chosenChild.Length - 1);
    }

    private void SetNextChildIndices(int agentNum)
    {
        if (agentNum == -1)
            this.chosenChild[0] = -1;
        else if (this.chosenChild[agentNum] < a_star_mdd_node.allSteps[agentNum].children.Count - 1)
            this.chosenChild[agentNum]++;
        else
        {
            this.chosenChild[agentNum] = 0;
            SetNextChildIndices(agentNum - 1);
        }
    }
}
