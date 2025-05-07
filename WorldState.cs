using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

namespace mapf;

/// <summary>
/// Describes a node in the A* search space.
/// </summary>
public class WorldState : IComparable<IBinaryHeapItem>, IBinaryHeapItem, IHeuristicSearchNode
{
    public int Makespan { get; set; } // Total time steps passed, max(agent makespans)
    public int G { get; set; } // Value depends on Constants.costFunction and Constants.sumOfCostsVariant, Sum of agent makespans until they reach their goal
    public int H { get; set; }
    public int HBonus { get; set; }
    public AgentState[] AllAgentsState { get; private set; }
    public WorldState PrevStep { get; set; }
    private int _binaryHeapIndex;
    public MDDNode MDDNode { get; set; }
    public int Generated { get; set; }

    protected int _primaryTieBreaker;
    private int _secondaryTieBreaker;
    /// <summary>
    /// Maps from agent num to the number of times the path up to this node collides with that agent
    /// </summary>
    public Dictionary<int, int> ConflictCounts { get; set; }
    /// <summary>
    /// Maps from agent num to a list of the conflict times with it
    /// </summary>
    public Dictionary<int, List<int>> ConflictTimes { get; set; }
    /// <summary>
    /// The min depth (makespan) from which a node may be considered a goal.
    /// TODO: Consider moving out of the node object to a static variable or something.
    ///       It doesn't change between nodes.
    /// </summary>
    public int MinGoalTimeStep { get; private set; }
    /// <summary>
    /// The min cost (g) from which a node may be considered a goal.
    /// TODO: Consider moving out of the node object to a static variable or something.
    ///       It doesn't change between nodes.
    /// </summary>
    public int MinGoalCost { get; private set; }
    /// <summary>
    /// The last move of all agents that have already moved in this turn.
    /// Used for making sure the next agent move doesn't collide with moves already made.
    /// Used while generating this node, nullified when done.
    /// </summary>
    public Dictionary<TimedMove, int> CurrentMoves { get; set; }
    private const int NOT_SET = -1;
    /// <summary>
    /// For computing expansion delay
    /// </summary>
    public int ExpandedCountWhenGenerated { get; set; }
    ///// <summary>
    ///// For lazy heuristics
    ///// </summary>
    //public CBS cbsState;
    /// <summary>
    /// For MStar.
    /// Disjoint sets of agent indices, since only internal agents are considered.
    /// </summary>
    public DisjointSets<int> CollisionSets { get; set; }
    //public ISet<int> currentCollisionSet;
    public ISet<WorldState> BackPropagationSet { get; set; }

    /// <summary>
    /// Create a state with the given state for every agent.
    /// </summary>
    /// <param name="allAgentsState"></param>
    /// <param name="minDepth"></param>
    /// <param name="minCost"></param>
    /// <param name="mddNode"></param>
    public WorldState(AgentState[] allAgentsState, int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
    {
        AllAgentsState = [.. allAgentsState];
        Makespan = allAgentsState.Max(state => state.lastMove.Time); // We expect to only find at most two G values within the agent group
        CalculateG(); // G not necessarily zero when solving a partially solved problem.
        _primaryTieBreaker = 0;
        _secondaryTieBreaker = 0;
        ConflictCounts = [];  // Unused if not running under CBS, and we can't tell at this point easily
        ConflictTimes = [];  // Unused if not running under CBS, and we can't tell at this point easily
        MinGoalTimeStep = minDepth;
        MinGoalCost = minCost;
        if (mddNode == null)
            CurrentMoves = [];
        goalCost = NOT_SET;
        goalSingleCosts = null;
        singlePlans = null;
        HBonus = 0;
        MDDNode = mddNode;
    }

    /// <summary>
    /// Copy constructor.
    /// </summary>
    /// <param name="cpy"></param>
    public WorldState(WorldState cpy)
    {
        Makespan = cpy.Makespan;
        G = cpy.G;
        H = cpy.H;
        // The conflictTimes, conflictCounts and sumConflictCounts are only copied later if necessary.
        MinGoalTimeStep = cpy.MinGoalTimeStep;
        MinGoalCost = cpy.MinGoalCost;
        AllAgentsState = new AgentState[cpy.AllAgentsState.Length];
        for (int i = 0; i < AllAgentsState.Length; i++)
        {
            AllAgentsState[i] = new AgentState(cpy.AllAgentsState[i]);
            // Shallow copy - it's still the same lastMove inside the AgentState, until we set a new lastMove there.
        }
        if (cpy.CurrentMoves != null)
            // cpy is an intermediate node
            CurrentMoves = new Dictionary<TimedMove, int>(dictionary: cpy.CurrentMoves);
        else
            // cpy is a concrete node
            CurrentMoves = new Dictionary<TimedMove, int>(capacity: cpy.AllAgentsState.Length);
        goalCost = NOT_SET;
        goalSingleCosts = null;
        singlePlans = null;
        HBonus = 0;
        MDDNode = cpy.MDDNode;
        PrevStep = cpy;
    }

    /// <summary>
    /// Creates a new state by extracting a subset of the agents from
    /// the original WorldState. We overload the constructor because
    /// while building our pattern database, we rewrite the problem and
    /// therefore need to make a deep copy of the state data structures so
    /// as to not overwrite the original problem. The ultimate solution
    /// would be to rework the code to remove static variables so that we
    /// can instantiate subproblems without affecting the original data
    /// structures.
    /// </summary>
    /// <param name="allAgentsState">A set of agent states in the original problem.</param>
    /// <param name="agentIndicesToCopy">A list of indices referring to the subset of agents we want to extract.</param>
    public WorldState(AgentState[] allAgentsState, List<uint> agentIndicesToCopy)
        // Copy specified agents only
        : this(agentIndicesToCopy.Select(index => new AgentState(allAgentsState[index])).ToArray())
    {}
        
    public bool GoalTest()
    {
        // Check if this is a generalised goal node and its plan is long enough.
        // If we know the optimal solution, it doesn't matter if this is a real goal node or not, we can finish.
        if (singlePlans != null)
        {
            // Check if plans are long enough and costly enough
            if (singlePlans.All(plan => plan.GetSize() - 1 >= MinGoalTimeStep))
            {
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                {
	                if (singlePlans.Sum(plan => plan.GetCost()) >= MinGoalCost)
                        return true;
                }
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                {
                    if (singlePlans.Max(plan => plan.GetCost()) >= MinGoalCost)
    	                return true;
                }
                else
                    throw new Exception("Unsupported cost function");
            }
        }

        if (G < MinGoalCost)
            return false;

        if (Makespan < MinGoalTimeStep)
            return false;

        return H == 0; // This assumes the heuristic is consistent,
                            // or at least has the property of consistent heuristics that only the goal has h==0.
                            // SIC really is a consistent heuristic, so this is fine for now.
                            // TODO: Implement a proper goal test and use it when h==0.
    }

    protected SinglePlan[] singlePlans;

    /// <summary>
    /// Set the optimal solution of this node as a problem instance.
    /// Currently only used by CbsHeuristicForAStar, if a solution was found while running the heuristic.
    /// </summary>
    /// <param name="solution"></param>
    public virtual void SetSolution(SinglePlan[] solution)
    {
        singlePlans = SinglePlan.GetSinglePlans(this); // This node may be a partial solution itself, need to start from the real root.
        for (int i = 0; i < solution.Length; ++i)
            singlePlans[i].ContinueWith(solution[i]);
    }

    public SinglePlan[] GetSinglePlans()
    {
        if (singlePlans != null)
            return singlePlans;
        else
            return SinglePlan.GetSinglePlans(this);
    }

    /// <summary>
    /// Returns the optimal plan to the goal through this node, if this is a goal node (of any kind),
    /// else returns the optimal plan to this node.
    /// </summary>
    /// <returns></returns>
    public Plan GetPlan()
    {
        if (singlePlans != null)
            return new Plan(singlePlans);
        else
            return new Plan(this);
    }

    /// <summary>
    /// For generalized goal nodes.
    /// TODO: Get rid of this and just return the sum/max of the single costs where needed?
    /// </summary>
    protected int goalCost;

    /// <summary>
    /// Returns the optimal cost to the goal from the start through this node.
    /// </summary>
    /// <returns></returns>
    public int GetGoalCost()
    {
        Trace.Assert(GoalTest(), "Only call for goal nodes!");

        if (goalCost == NOT_SET) // This is just a proper goal
        {
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
            {
                return G;
            }
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
            {
                return Makespan;
            }
            return 0; // To quiet the compiler
        }
        else                     // This is a generalised goal node - it stores the optimal path to the goal through it
            return goalCost;
    }

    /// <summary>
    /// Set the optimal cost from the start to the goal through this node.
    /// Makes this a generalized goal node.
    /// Currently only used by CbsHeuristicForAStar.
    /// </summary>
    /// <param name="cost"></param>
    public void SetGoalCost(int cost)
    {
        goalCost = cost;
    }

    /// <summary>
    /// For generalized goal nodes
    /// </summary>
    protected int[] goalSingleCosts;

    public int[] GetSingleCosts()
    {
        Trace.Assert(GoalTest(), "Only call for goal nodes!");

        if (goalSingleCosts == null) // This is just a proper goal
            return AllAgentsState.Select(agent => agent.g).ToArray();
        else
            return goalSingleCosts;
    }

    /// <summary>
    /// Set the optimal cost from the start to the goal through this node for every agent.
    /// Makes this node a generalized goal node.
    /// Currently only used by CbsHeuristicForAStar.
    /// </summary>
    /// <param name="costs"></param>
    public void SetSingleCosts(int[] costs)
    {
        goalSingleCosts = costs;
    }

    /// <summary>
    /// Used when WorldState objects are put in the open list priority queue
    /// </summary>
    /// <param name="other"></param>
    /// <returns></returns>
    public virtual int CompareTo(IBinaryHeapItem other)
    {
        WorldState that = (WorldState)other;
        int thisF = F;
        int thatF = that.F;
        if (thisF < thatF)
            return -1;
        if (thisF > thatF)
            return 1;

        return TieBreak(that);
    }

    public int TieBreak(WorldState that)
    {
        bool thisIsGoal = GoalTest();
        bool thatIsGoal = that.GoalTest();
        if (thisIsGoal == true && thatIsGoal == false) // The elaborate form is necessary to keep the comparison consistent. Otherwise goalA<goalB and goalB<goalA
            return -1;
        if (thatIsGoal == true && thisIsGoal == false)
            return 1;


        // Prefer nodes that contain conflicts with fewer agents - when a conflict is resolved,
        // many times other conflicts with the same agent are resolved automatically thanks to conflict avoidance,
        // especially if the cost increases.
        // For ID, this may instead prefers nodes which conflict with smaller groups.
        // TODO: Ideally, prefer nodes where the minimum vertex cover of the conflict graph is smaller.
        //       Compute an MVC of the conflict graph without the node's agents in the CBS node
        //       before running the low level, and only compare the number of agents the node
        //       conflicts with that aren't in the MVC.
        //       Maybe even compute the MVC of the cardinal conflict graph and of the all conflict
        //       graph separately and tie-break first according to the number of agents we conflict
        //       with that aren't in the MVC of the conflict graph and then the number of agents
        //       we conflict with that aren't in the cardinal conflict graph
        if (_primaryTieBreaker < that._primaryTieBreaker)
            return -1;
        if (_primaryTieBreaker > that._primaryTieBreaker)
            return 1;

        // Prefer nodes with fewer conflicts - the probability that some of them are cardinal is lower
        if (_secondaryTieBreaker < that._secondaryTieBreaker)
            return -1;
        if (_secondaryTieBreaker > that._secondaryTieBreaker)
            return 1;

        // //M-Star: prefer nodes with smaller collision sets:
        //if (collisionSets != null) // than M-Star is running
        //{
        //    // The collision sets change during collision set backpropagation and closed list hits.
        //    // Backpropagation goes from a node's child to the node, so it's tempting to think
        //    // it only happens when the node is already expanded and out of the open list,
        //    // but partial expansion makes that false. 
        //    // Closed list hits can also happen while the node is waiting to be expanded.
        //    // So the max rank can change while the node is in the open list - 
        //    // it can't be used for tie breaking :(.
        //    if (collisionSets.maxRank < that.collisionSets.maxRank)
        //        return -1;
        //    if (that.collisionSets.maxRank > collisionSets.maxRank)
        //        return 1;
        //}

        // f, collision sets, conflicts and internal conflicts being equal, prefer nodes with a larger g
        // - they're closer to the goal so less nodes would probably be generated by them on the way to it.
        if (G < that.G)
            return 1;
        if (G > that.G)
            return -1;

        return 0;
    }

    /// <summary>
    /// Calculate and set the g of the state as the sum of the different agent g values.
    /// </summary>
    public virtual void CalculateG()
    {
        G = Constants.costFunction switch
        {
            Constants.CostFunction.SUM_OF_COSTS => AllAgentsState.Sum(agent => agent.g),
            Constants.CostFunction.MAKESPAN or Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS => Makespan,// Let's hope makespan var is correct
            _ => throw new Exception($"Unsupported cost function {Constants.costFunction}"),
        };
    }

    /// <summary>
    /// Prepare for re-insertion into the open list
    /// </summary>
    public virtual void Clear() { }

    public virtual int F => G + H;

    public int GetTargetH(int f) => f - G;

    public override string ToString()
    {
        var builder = new System.Text.StringBuilder($"{Generated} f:{F} makespan:{Makespan} h:{H} g:{G} ");
        foreach (AgentState temp in AllAgentsState)
        {
            builder.Append("|");
            builder.Append(temp.lastMove);
        }
        builder.Append("|");
        return builder.ToString();
    }

    /// <summary>
    /// Returns the last move of all the agents in this state.
    /// </summary>
    /// <returns>A list of Moves</returns>
    public List<Move> GetAgentsMoves() => [.. AllAgentsState.Select<AgentState, Move>(state => state.lastMove)];

    /// <summary>
    /// Returns the last move of the requested agent.
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    public Move GetSingleAgentMove(int index) => AllAgentsState[index].lastMove;

    /// <summary>
    /// BH_Item implementation
    /// </summary>
    /// <returns></returns>
    public int GetIndexInHeap() => _binaryHeapIndex;

    /// <summary>
    /// BH_Item implementation
    /// </summary>
    /// <returns></returns>
    public void SetIndexInHeap(int index) { _binaryHeapIndex = index; }

    /// <summary>
    /// Checks for internal conflicts
    /// </summary>
    /// <returns></returns>
    public bool isValid()
    {
        for (int i = 0; i < AllAgentsState.Length; i++)
        {
            for (int j = i+1; j < AllAgentsState.Length; j++)
            {
                // Internal conflict
                if (AllAgentsState[i].lastMove.IsColliding(AllAgentsState[j].lastMove))
                    return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Only the agent states are used in the hash.
    /// The g, makespan, h, potentialConflictsCount, sumConflictCounts and others are ignored, as neccesary.
    /// </summary>
    /// <returns></returns>
    public override int GetHashCode()
    {
        int ans = 0;
        unchecked
        {
            for (int i = 0 ; i < AllAgentsState.Length; i++)
            {
                ans += AllAgentsState[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
            }
        }
        return ans;
    }

    /// <summary>
    /// Only the AgentStates are compared.
    /// g, makespan, h, potentialConflictsCount, sumConflictCounts and others are ignored, as necessary.
    /// </summary>
    /// <param name="obj"></param>
    /// <returns></returns>
    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        WorldState that = (WorldState)obj;
        return AllAgentsState.SequenceEqual(that.AllAgentsState);
    }

    /// <summary>
    /// Counts the number of times this node collides with each agent move in the conflict avoidance table.
    /// Also compute the representative primary and secondary tie-breaking values according to the CAT's avoidance goal
    /// </summary>
    /// <param name="CAT"></param>
    /// <returns></returns>
    public virtual void IncrementConflictCounts(ConflictAvoidanceTable CAT)
    {
        for (int i = 0; i < AllAgentsState.Length; i++)
        {
            AllAgentsState[i].lastMove.IncrementConflictCounts(CAT, ConflictCounts, ConflictTimes);
        }

        if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_CONFLICTS)  // For ID, the original rule
            _primaryTieBreaker = ConflictCounts.Sum(pair => pair.Value);
        else if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_CONFLICTING_GROUPS)
            _primaryTieBreaker = ConflictCounts.Keys.Count;
        else if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_CONFLICTING_GROUPS_THEN_CONFLICTS)
            // For CBS, minimizes the number of conflicting groups and then the number of conflicts with them
        {
            _primaryTieBreaker = ConflictCounts.Keys.Count;
            _secondaryTieBreaker = ConflictCounts.Sum(pair => pair.Value);
        }
        else if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_LARGEST_CONFLICTING_GROUP_THEN_NUMBER_OF_SUCH_GROUPS)
            // For ID, minimizes the size of the largest group we conflict with and then 
            // the number of conflicting groups with that size. The idea was to minimize conflicts that matter, and conflicts with
            // non-max-size groups don't.
            // Kept mostly for reference.
        {
            if (ConflictCounts.Count != 0)
            {
                _primaryTieBreaker = ConflictCounts.Max(pair => CAT.AgentSizes[pair.Key]);
                _secondaryTieBreaker = ConflictCounts.Where(pair => CAT.AgentSizes[pair.Key] == _primaryTieBreaker).Count();
            }
            else
            {
                _primaryTieBreaker = 0;
                _secondaryTieBreaker = 0;
            }
        }
        else if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_LARGEST_CONFLICTING_GROUP_THEN_MAXIMIZE_CONFLICT_COUNTS_WITH_OTHERS)
        // For ID, minimizes the size of the largest group we conflict with and then 
        // maximizes the number of conflicts the two groups have with other groups
        {
            if (ConflictCounts.Count != 0)
            {
                _primaryTieBreaker = ConflictCounts.Max(pair => CAT.AgentSizes[pair.Key]);
                _secondaryTieBreaker = -(ConflictCounts.Sum(pair => pair.Value) +
                    ConflictCounts.Where(pair => CAT.AgentSizes[pair.Key] == _primaryTieBreaker).Max(pair => CAT.AgentConflictCounts[pair.Key]));
            }
            else
            {
                _primaryTieBreaker = 0;
                _secondaryTieBreaker = 0;
            }
        }
        else if (CAT.AvoidanceGoal == AvoidanceGoal.MINIMIZE_CONFLICTING_GROUP_SIZE_AND_COUNT)
        {
            _primaryTieBreaker = ConflictCounts.Sum(pair => 1 << (CAT.AgentSizes[pair.Key] - 1));
        }
    }

    /// <summary>
    /// Currently only used by CbsHeuristicForAStar, where each A* node is converted to a new
    /// MAPF problem to be solved by CBS (or ICTS, theoretically)
    /// </summary>
    /// <param name="initial"></param>
    /// <returns></returns>
    public virtual (ProblemInstance, ISet<CbsConstraint>) ToProblemInstance(ProblemInstance initial)
    {
        // Notice this is not a subproblem in the number of agents but
        // in the steps from the start.
        // It might even be harder if the steps were away from the goal.
        return (initial.Subproblem(AllAgentsState), new HashSet<CbsConstraint>());
    }
}
