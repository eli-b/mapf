using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Text;
using ExtensionMethods;

namespace mapf;

/// <summary>
/// Represents a plan for a set of agents.
/// </summary>
public class Plan
{
    private readonly List<List<Move>> _locationsAtTimes = [];

    /// <summary>
    /// Reconstructs the plan by goind backwards from the goal.
    /// </summary>
    /// <param name="goalState">The goal state from which to start going backwards</param>
    public Plan(WorldState goalState)
    {
        WorldState currentNode = goalState;
        // TODO: Initialize list with #agents
        while (currentNode != null)
        {
            List<Move> agentMoves = currentNode.GetAgentsMoves();
            _locationsAtTimes.Add(agentMoves);
            currentNode = currentNode.prevStep;
        }
        _locationsAtTimes.Reverse();
    }

    /// <summary>
    /// Reconstructs the plan by going backwards from the goal.
    /// </summary>
    /// <param name="goalState">The goal state from which to start going backwards</param>
    public Plan(AgentState goalState)
    {
        AgentState currentNode = goalState;
        // TODO: Initialize list with #agents
        while (currentNode != null)
        {
            List<Move> l = [];
            l.Add(currentNode.GetMove());
            _locationsAtTimes.Add(l);
            currentNode = currentNode.prev;
        }
        _locationsAtTimes.Reverse();
    }

    /// <summary>
    /// Assumes all routes are of the same length.
    /// </summary>
    public Plan(LinkedList<Move>[] routePerAgent)
    {
        for (int i = 0; i < routePerAgent[0].Count; i++)
        {
            _locationsAtTimes.Add([]);
        }

        int index = 0;
        foreach (LinkedList<Move> agentRoute in routePerAgent)
        {
            List<Move> locationsAtTime = _locationsAtTimes[index];
            foreach (Move agentLocation in agentRoute)
            {
                locationsAtTime.Add(agentLocation);
                index++;
            }
        }
    }

    /// <summary>
    /// Generates a big plan from a collection of smaller plans.
    /// </summary>
    public Plan(IEnumerable<Plan> subplans)
    {
        int maxSize = subplans.Max(plan => plan.GetSize());
        _locationsAtTimes = [];

        for (int time = 0; time < maxSize; time++)
        {
            List<Move> allMoves = [];
            foreach (Plan plan in subplans)
                foreach (Move move in plan.GetLocationsAt(time))
                    allMoves.Add(move);

            _locationsAtTimes.Add(allMoves);
        }
    }

    public Plan(IEnumerable<SinglePlan> subplans) // FIXME: Almost fully duplicates the previous method
    {
        int maxSize = subplans.Max(plan => plan.GetSize());

        for (int time = 0; time < maxSize; time++)
        {
            List<Move> allMoves = [];
            foreach (SinglePlan plan in subplans)
            {
                allMoves.Add(plan.GetLocationAt(time));
            }

            _locationsAtTimes.Add(allMoves);
        }
    }

    /// <summary>
    /// Medium-depth copy constructor - uses same Move objects.
    /// </summary>
    public Plan(Plan cpy)
    {
        foreach (List<Move> cpyStep in cpy._locationsAtTimes)
        {
            _locationsAtTimes.Add([.. cpyStep]);
        }
    }

    /// <summary>
    /// Add actions of other plan after actions of plan.
    /// If this plan ends where the other starts,
    /// the first timestep of the other plan is skipped.
    /// </summary>
    public void ContinueWith(Plan other)
    {
        bool first = true;
        foreach (List<Move> newLocationsAtTime in other._locationsAtTimes)
        {
            if (first)
            {
                first = false;
                if (newLocationsAtTime.SequenceEqual<Move>(_locationsAtTimes.Last()))
                    continue;
                else
                    Trace.Assert(false, "Continuing a plan doesn't start from the same state");
            }
            _locationsAtTimes.Add(newLocationsAtTime);
        }
    }

    public void Check(ProblemInstance problem)
    {
        SinglePlan[] singles = new SinglePlan[_locationsAtTimes.First().Count];
        for (int i = 0; i < singles.Length; i++)
        {
            singles[i] = new SinglePlan(this, i, problem.agents[i].agent.agentNum);
            foreach ((int time, var move) in singles[i].LocationAtTimes.Enumerate())
                Trace.Assert(problem.IsValid(move), $"Plan of agent {i} uses an invalid location {move} at time {time}!");
        }

        // Check in every time step that the plans do not collide
        for (int time = 1; time < _locationsAtTimes.Count; time++) // Assuming no conflicts exist in time zero.
        {
            // Check all pairs of agents for a collision at the given time step
            foreach ((int i1, var plan1) in singles.Enumerate())
            {
                foreach ((int i2, var plan2) in singles.Enumerate())
                {
                    if (i1 < i2)
                        Trace.Assert(plan1.IsColliding(time, plan2) == false, $"Plans of agents {i1} and {i2} collide at time {time}!");
                }
            }
        }
    }

    // TODO: Add GetCost and GetMakespan methods!

    /// <summary>
    /// Returns the location of the agents at a given time. 
    /// If the requested time is after the last step of the plan,
    /// the agents are assumed to stay at their final location.
    /// </summary>
    /// <param name="time">The requested time</param>
    /// <returns>A list of Moves that are the locations of the different agents at the requested time</returns>
    public List<Move> GetLocationsAt(int time)
    {
        if (time < _locationsAtTimes.Count)
            return _locationsAtTimes.ElementAt(time); // FIXME: Expensive!
        else
        {
            List<Move> toCopy = _locationsAtTimes.Last();
            List<Move> atRest = [.. toCopy];
            for (int i = 0; i < atRest.Count; i++)
            {
                atRest[i] = new Move(atRest[i].X, atRest[i].Y, Direction.Wait);
            }
            return atRest;
        }
    }

    public List<List<Move>> GetLocations() => _locationsAtTimes;

    /// <summary>
    /// NOT the cost, which:
    /// A) could depend on steps taken before solving started,
    /// B) is 1 smaller than the size (a plan that starts at the goal costs zero)
    /// C) under sum-of-costs, is the sum of the agent costs
    /// Useful only for iteration over the relevant part of the plan.
    /// </summary>
    /// <returns>The size of the plan, assuming is doesn't end with steps where all agents WAIT at the goal (which should be discounted).</returns>
    public int GetSize() => _locationsAtTimes.Count;

    /// <summary>
    /// Check if this plan collides with another plan at a given time
    /// </summary>
    /// <param name="time">The time at which to check if the collision occured</param>
    /// <param name="otherPlan">The plan to check against</param>
    public bool IsColliding(int time, Plan otherPlan)
    {
        List<Move> thisLocations = GetLocationsAt(time);
        List<Move> otherLocations = otherPlan.GetLocationsAt(time);

        // TODO: Think of a better implementation of this
        foreach (Move aMove in thisLocations)
            foreach (Move bMove in otherLocations)
                if (aMove.IsColliding(bMove) == true)
                    return true;

        return false;
    }

    /// <summary>
    /// Print plan if it would fit nicely in the console, otherwise print why it wasn't printed
    /// </summary>
    public void PrintPlanIfShort()
    {
        var planSize = GetSize();
        var numAgents = _locationsAtTimes.First().Count;
        if (planSize < 200 && numAgents < 30)
            PrintPlan();
        else if (planSize >= 200)
            Console.WriteLine($"Plan is too long to print ({planSize} steps).");
        else
            Console.WriteLine($"Plan is too wide to print ({numAgents} agents).");
    }

    /// <summary>
    /// Prints the plan to the Console. 
    /// This is used for debugging purposes.
    /// </summary>
    public void PrintPlan()
    {
        foreach (List<Move> locationsAtTime in _locationsAtTimes)
        {
            Console.Write("|");
            foreach (Move aMove in locationsAtTime)
            {
                Console.Write(aMove.ToString() + "|");
            }
            Console.WriteLine("");
        }
    }

    public override string ToString()
    {
        StringBuilder s = new();
        foreach (List<Move> locationsAtTime in _locationsAtTimes)
        {
            s.Append($"|{String.Join("|", locationsAtTime)}|\n");
        }
        return s.ToString();
    }

    public HashSet<TimedMove> AddPlanToHashSet(HashSet<TimedMove> addTo, int until)
    {
        for (int i = 1; i < until; i++)  // i = 1 because we assume start positions don't overlap
        {
            List<Move> step = GetLocationsAt(i);
            foreach (Move move in step)
            {
                addTo.Add(new TimedMove(move,i));
            }
        }
        return addTo;
    }

}

public class SinglePlan
{
    public List<Move> LocationAtTimes { get; private set; }
    public int AgentNum { get; set; }

    /// <summary>
    /// Not used
    /// </summary>
    /// <param name="goalState"></param>
    /// <param name="agentIndex"></param>
    public SinglePlan(WorldState goalState, int agentIndex)
    {
        AgentNum = goalState.allAgentsState[agentIndex].agent.agentNum;
        WorldState currentNode = goalState;
        LinkedList<Move> locations = [];
        while (currentNode != null)
        {
            locations.AddFirst(currentNode.GetSingleAgentMove(agentIndex));
            currentNode = currentNode.prevStep;
        }
        LocationAtTimes = [.. locations];
    }

    /// <summary>
    /// Get a SinglePlan from a Plan
    /// </summary>
    /// <param name="plan"></param>
    /// <param name="agentIndex">In this plan</param>
    /// <param name="agentNum">To put in the returned SinglePlan</param>
    public SinglePlan(Plan plan, int agentIndex, int agentNum)
    {
        AgentNum = agentNum;
        LocationAtTimes = [];
        foreach (List<Move> movesAtTimestep in plan.GetLocations())
        {
            LocationAtTimes.Add(movesAtTimestep[agentIndex]);
        }
    }

    public SinglePlan(AgentState goalState)
    {
        AgentNum = goalState.agent.agentNum;
        AgentState currentNode = goalState;
        List<Move> locations = [];
        while (currentNode != null)
        {
            locations.Add(currentNode.GetMove());
            currentNode = currentNode.prev;
        }
        locations.Reverse();
        LocationAtTimes = locations;
    }

    public SinglePlan(List<Move> route, int agentNum)
    {
        AgentNum = agentNum;
        LocationAtTimes = route;
    }

    public SinglePlan(SinglePlan cpy)
    {
        LocationAtTimes = [.. cpy.LocationAtTimes]; // Behavior change: used to do a deep copy, with cloned moves.
        AgentNum = cpy.AgentNum;
    }

    /// <summary>
    /// TODO: Get rid of the else
    /// </summary>
    public Move GetLocationAt(int time)
    {
        if (time < LocationAtTimes.Count)
            return LocationAtTimes[time];
        else
        {
            var rest = new TimedMove(LocationAtTimes[LocationAtTimes.Count - 1], time);
            rest.Direction = Direction.Wait;
            return rest;
        }
    }

    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        SinglePlan other = (SinglePlan)obj;
        return AgentNum == other.AgentNum && LocationAtTimes.SequenceEqual<Move>(other.LocationAtTimes);
    }

    public override int GetHashCode()
    {
        int ret;
        unchecked // wrap-around is fine in hash functions
        {
            ret = Constants.PRIMES_FOR_HASHING[0] * AgentNum.GetHashCode();

            // Hash the contents and order of locationsAtTimes
            int i = 0;
            foreach (var move in LocationAtTimes)
            {
                ret += Constants.PRIMES_FOR_HASHING[1] * i + Constants.PRIMES_FOR_HASHING[2] * move.GetHashCode();
                i++;
            }
        }
        return ret;
    }

    /// <summary>
    /// Add actions of other plan after actions of plan.
    /// If this plan ends where the other starts,
    /// the first timestep of the other plan is skipped
    /// </summary>
    /// <param name="other"></param>
    public void ContinueWith(SinglePlan other)
    {
        bool first = true;
        foreach (Move newLocationAtTime in other.LocationAtTimes)
        {
            if (first)
            {
                first = false;
                if (LocationAtTimes[^1].Equals(newLocationAtTime))
                    continue;
                else
                    Trace.Assert(false, "Continuing a plan doesn't start from the same state");
            }
            LocationAtTimes.Add(newLocationAtTime);
        }
    }

    /// <summary>
    /// NOT the cost, which:
    /// A) could depend on steps taken before solving started,
    /// B) is 1 smaller than the size (a plan that starts at the goal costs zero)
    /// Useful only for iteration over the relevant part of the plan.
    /// </summary>
    /// <returns>The size of the plan, excluding WAITs at the goal</returns>
    public int GetSize()
    {
        int lastNonWaitIndex = LocationAtTimes.Count - 1;
        while (lastNonWaitIndex != 0 && LocationAtTimes[lastNonWaitIndex].Direction == Direction.Wait)
            lastNonWaitIndex--;
        return lastNonWaitIndex + 1;
    }

    /// <summary>
    /// TODO: Find all "GetSize() - 1" uses and replace them with this method
    /// </summary>
    /// <returns></returns>
    public int GetCost()
    {
        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
        {
            if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
            {
                return GetSize() - 1;
            }
            else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
            {
                int cost = 0;
                Move goal = LocationAtTimes.Last<Move>(); // Assuming the plan ends at the goal
                for (int i = 1; i < LocationAtTimes.Count; i++) // The beginning position isn't a move
                {
                    Move move = LocationAtTimes[i];
                    if (move.X == goal.X &&
                        move.Y == goal.Y &&
                        move.Direction == Direction.Wait) // Waiting at the goal is free
                        continue;
                    cost += 1;
                }
                return cost;
            }
        }
        else if (Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
        {
            return GetSize() - 1;
        }
        return 0; // To quiet the compiler
    }

    /// <summary>
    /// Check if this plan collides with another plan at a given time
    /// </summary>
    /// <param name="time">The time at which to check if the collision occured</param>
    /// <param name="otherPlan">The plan to check against</param>
    public bool IsColliding(int time, SinglePlan otherPlan)
    {
        Move thisLocation = GetLocationAt(time);
        Move otherLocation = otherPlan.GetLocationAt(time);

        if (thisLocation.IsColliding(otherLocation) == true) // IsColliding isn't virtual,
                                                                // so it doesn't matter whether the moves are actually TimedMoves
                                                                // with incorrect time
            return true;

        return false;
    }

    /// <summary>
    /// Prints the plan to the Console. 
    /// This is used for debugging purposes.
    /// </summary>
    public void DebugPrint()
    {
        for (int time = 0; time < LocationAtTimes.Count; time++)
        {
            Debug.WriteLine($"|{GetLocationAt(time)}|");
        }
    }

    public override string ToString()
    {
        string s = "";
        for (int time = 0; time < LocationAtTimes.Count; time++)
        {
            s += $"|{GetLocationAt(time)}|\n";
        }
        return s;
    }

    public static SinglePlan[] GetSinglePlans(WorldState goalState) // FIXME: Duplication with other methods.
    {
        List<Move>[] allroutes = new List<Move>[goalState.allAgentsState.Length];
        for (int i = 0; i < allroutes.Length; i++)
            allroutes[i] = [];

        WorldState currentNode = goalState;
        while (currentNode != null)
        {
            for (int i = 0; i < allroutes.Length; i++)
                allroutes[i].Add(currentNode.GetSingleAgentMove(i));
            currentNode = currentNode.prevStep;
        }

        SinglePlan[] ans = new SinglePlan[goalState.allAgentsState.Length];
        for (int i = 0; i < ans.Length; i++)
        {
            allroutes[i].Reverse();
            ans[i] = new SinglePlan(allroutes[i], goalState.allAgentsState[i].agent.agentNum);
        }
        return ans;
    }

    /// <summary>
    /// Creates SinglePlans with agentIndex as agentNum. Not suitable for subproblems.
    /// </summary>
    /// <param name="allRoutes"></param>
    /// <returns></returns>
    public static SinglePlan[] GetSinglePlans(List<Move>[] allRoutes)
    {
        SinglePlan[] ans = new SinglePlan[allRoutes.Length];
        for (int i = 0; i < ans.Length; i++)
        {
            ans[i] = new SinglePlan(allRoutes[i], i);
        }
        return ans;
    }
}
