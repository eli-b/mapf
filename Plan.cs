using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.Text;
using ExtensionMethods;

namespace mapf
{
    /// <summary>
    /// Represents a plan for a set of agents.
    /// </summary>
    public class Plan
    {
        private LinkedList<List<Move>> locationsAtTimes;

        /// <summary>
        /// Reconstructs the plan by goind backwards from the goal.
        /// </summary>
        /// <param name="goalState">The goal state from which to start going backwards</param>
        public Plan(WorldState goalState)
        {
            WorldState currentNode = goalState;
            this.locationsAtTimes = new LinkedList<List<Move>>(); // TODO: Initialize list with #agents
            while (currentNode != null)
            {
                List<Move> agentMoves = currentNode.GetAgentsMoves();
                this.locationsAtTimes.AddFirst(agentMoves);
                currentNode = currentNode.prevStep;
            }
        }

        /// <summary>
        /// Reconstructs the plan by going backwards from the goal.
        /// </summary>
        /// <param name="goalState">The goal state from which to start going backwards</param>
        public Plan(AgentState goalState)
        {
            AgentState currentNode = goalState;
            this.locationsAtTimes = new LinkedList<List<Move>>(); // TODO: Initialize list with #agents
            while (currentNode != null)
            {
                var l = new List<Move>();
                l.Add(currentNode.GetMove());
                this.locationsAtTimes.AddFirst(l);
                currentNode = currentNode.prev;
            }
        }

        /// <summary>
        /// Assumes all routes are of the same length.
        /// </summary>
        /// <param name="routePerAgent"></param>
        public Plan(LinkedList<Move>[] routePerAgent)
        {
            this.locationsAtTimes = new LinkedList<List<Move>>();
            for (int i = 0; i < routePerAgent[0].Count; i++)
            {
                locationsAtTimes.AddLast(new List<Move>());
            }
            
            foreach (LinkedList<Move> agentRoute in routePerAgent)
            {
                LinkedListNode<List<Move>> locationsAtTime = this.locationsAtTimes.First;
                foreach (Move agentLocation in agentRoute)
                {
                    locationsAtTime.Value.Add(agentLocation);
                    locationsAtTime = locationsAtTime.Next;
                }
            }
        }

        /// <summary>
        /// Generates a big plan from a collection of smaller plans.
        /// </summary>
        /// <param name="subplans"></param>
        public Plan(IEnumerable<Plan> subplans)
        {
            int maxSize = subplans.Max(plan => plan.GetSize());
            this.locationsAtTimes = new LinkedList<List<Move>>();

            for (int time = 0; time < maxSize; time++)
            {
                var allMoves = new List<Move>();
                foreach (Plan plan in subplans)
                    foreach (Move move in plan.GetLocationsAt(time))
                        allMoves.Add(move);

                this.locationsAtTimes.AddLast(allMoves);
            }
        }

        public Plan(IEnumerable<SinglePlan> subplans) // FIXME: Almost fully duplicates the previous method
        {
            int maxSize = subplans.Max(plan => plan.GetSize());
            this.locationsAtTimes = new LinkedList<List<Move>>();

            for (int time = 0; time < maxSize; time++)
            {
                var allMoves = new List<Move>();
                foreach (SinglePlan plan in subplans)
                {
                    allMoves.Add(plan.GetLocationAt(time));
                }

                this.locationsAtTimes.AddLast(allMoves);
            }
        }

        /// <summary>
        /// Medium-depth copy constructor - uses same Move objects.
        /// </summary>
        /// <param name="cpy"></param>
        public Plan(Plan cpy)
        {
            this.locationsAtTimes = new LinkedList<List<Move>>();

            foreach (List<Move> cpyStep in cpy.locationsAtTimes)
            {
                this.locationsAtTimes.AddLast(cpyStep.ToList<Move>());
            }
        }

        /// <summary>
        /// Add actions of other plan after actions of plan.
        /// If this plan ends where the other starts,
        /// the first timestep of the other plan is skipped.
        /// </summary>
        /// <param name="other"></param>
        public void ContinueWith(Plan other)
        {
            bool first = true;
            foreach (List<Move> newLocationsAtTime in other.locationsAtTimes)
            {
                if (first)
                {
                    first = false;
                    if (newLocationsAtTime.SequenceEqual<Move>(this.locationsAtTimes.Last.Value))
                        continue;
                    else
                        Trace.Assert(false, "Continuing a plan doesn't start from the same state");
                }
                this.locationsAtTimes.AddLast(newLocationsAtTime);
            }
        }

        public void Check(ProblemInstance problem)
        {
            SinglePlan[] singles = new SinglePlan[this.locationsAtTimes.First().Count];
            for (int i = 0; i < singles.Length; i++)
            {
                singles[i] = new SinglePlan(this, i, problem.agents[i].agent.agentNum);
                foreach ((int time, var move) in singles[i].locationAtTimes.Enumerate())
                    Trace.Assert(problem.IsValid(move), $"Plan of agent {i} uses an invalid location {move} at time {time}!");
            }

            // Check in every time step that the plans do not collide
            for (int time = 1; time < this.locationsAtTimes.Count; time++) // Assuming no conflicts exist in time zero.
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
            if (time < this.locationsAtTimes.Count)
                return this.locationsAtTimes.ElementAt(time); // FIXME: Expensive!
            else
            {
                var toCopy = this.locationsAtTimes.Last.Value;
                var atRest = toCopy.ToList();
                for (int i = 0; i < atRest.Count; i++)
                {
                    atRest[i] = new Move(atRest[i].x, atRest[i].y, Move.Direction.Wait);
                }
                return atRest;
            }
        }

        public LinkedList<List<Move>> GetLocations()
        {
            return this.locationsAtTimes;
        }

        /// <summary>
        /// NOT the cost, which:
        /// A) could depend on steps taken before solving started,
        /// B) is 1 smaller than the size (a plan that starts at the goal costs zero)
        /// C) under sum-of-costs, is the sum of the agent costs
        /// Useful only for iteration over the relevant part of the plan.
        /// </summary>
        /// <returns>The size of the plan, assuming is doesn't end with steps where all agents WAIT at the goal (which should be discounted).</returns>
        public int GetSize()
        {
            return this.locationsAtTimes.Count;
        }

        /// <summary>
        /// Check if this plan collides with another plan at a given time
        /// </summary>
        /// <param name="time">The time at which to check if the collision occured</param>
        /// <param name="otherPlan">The plan to check against</param>
        public bool IsColliding(int time, Plan otherPlan)
        {
            List<Move> thisLocations = this.GetLocationsAt(time);
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
            var numAgents = this.locationsAtTimes.First.Value.Count;
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
            foreach (List<Move> locationsAtTime in this.locationsAtTimes)
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
            var s = new StringBuilder();
            foreach (List<Move> locationsAtTime in this.locationsAtTimes)
            {
                s.Append($"|{String.Join("|", locationsAtTime)}|\n");
            }
            return s.ToString();
        }

        public HashSet<TimedMove> AddPlanToHashSet(HashSet<TimedMove> addTo, int until)
        {
            for (int i = 1; i < until; i++)  // i = 1 because we assume start positions don't overlap
            {
                List<Move> step = this.GetLocationsAt(i);
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
        public List<Move> locationAtTimes { get; private set; }
        public int agentNum;

        /// <summary>
        /// Not used
        /// </summary>
        /// <param name="goalState"></param>
        /// <param name="agentIndex"></param>
        public SinglePlan(WorldState goalState, int agentIndex)
        {
            this.agentNum = goalState.allAgentsState[agentIndex].agent.agentNum;
            WorldState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.GetSingleAgentMove(agentIndex));
                currentNode = currentNode.prevStep;
            }
            this.locationAtTimes = locations.ToList<Move>();
        }

        /// <summary>
        /// Get a SinglePlan from a Plan
        /// </summary>
        /// <param name="plan"></param>
        /// <param name="agentIndex">In this plan</param>
        /// <param name="agentNum">To put in the returned SinglePlan</param>
        public SinglePlan(Plan plan, int agentIndex, int agentNum)
        {
            this.agentNum = agentNum;
            this.locationAtTimes = new List<Move>();
            foreach (List<Move> movesAtTimestep in plan.GetLocations())
            {
                this.locationAtTimes.Add(movesAtTimestep[agentIndex]);
            }
        }

        public SinglePlan(AgentState goalState)
        {
            this.agentNum = goalState.agent.agentNum;
            AgentState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.GetMove());
                currentNode = currentNode.prev;
            }
            this.locationAtTimes = locations.ToList<Move>();
        }

        public SinglePlan(LinkedList<Move> route, int agentNum)
        {
            this.agentNum = agentNum;
            locationAtTimes = route.ToList<Move>();
        }

        public SinglePlan(SinglePlan cpy)
        {
            this.locationAtTimes = cpy.locationAtTimes.ToList<Move>(); // Behavior change: used to do a deep copy, with cloned moves.
            this.agentNum = cpy.agentNum;
        }

        /// <summary>
        /// TODO: Get rid of the else
        /// </summary>
        /// <param name="time"></param>
        /// <returns></returns>
        public Move GetLocationAt(int time)
        {
            if (time < this.locationAtTimes.Count)
                return this.locationAtTimes[time];
            else
            {
                var rest = new TimedMove(this.locationAtTimes[this.locationAtTimes.Count - 1], time);
                rest.direction = Move.Direction.Wait;
                return rest;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            SinglePlan other = (SinglePlan)obj;
            return this.agentNum == other.agentNum && this.locationAtTimes.SequenceEqual<Move>(other.locationAtTimes);
        }

        public override int GetHashCode()
        {
            int ret;
            unchecked // wrap-around is fine in hash functions
            {
                ret = Constants.PRIMES_FOR_HASHING[0] * this.agentNum.GetHashCode();

                // Hash the contents and order of locationsAtTimes
                int i = 0;
                foreach (var move in this.locationAtTimes)
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
            foreach (Move newLocationAtTime in other.locationAtTimes)
            {
                if (first)
                {
                    first = false;
                    if (this.locationAtTimes[this.locationAtTimes.Count - 1].Equals(newLocationAtTime))
                        continue;
                    else
                        Trace.Assert(false, "Continuing a plan doesn't start from the same state");
                }
                this.locationAtTimes.Add(newLocationAtTime);
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
            int lastNonWaitIndex = this.locationAtTimes.Count - 1;
            while (lastNonWaitIndex != 0 && locationAtTimes[lastNonWaitIndex].direction == Move.Direction.Wait)
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
                    return this.GetSize() - 1;
                }
                else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                {
                    int cost = 0;
                    Move goal = this.locationAtTimes.Last<Move>(); // Assuming the plan ends at the goal
                    for (int i = 1; i < this.locationAtTimes.Count; i++) // The beginning position isn't a move
                    {
                        Move move = this.locationAtTimes[i];
                        if (move.x == goal.x &&
                            move.y == goal.y &&
                            move.direction == Move.Direction.Wait) // Waiting at the goal is free
                            continue;
                        cost += 1;
                    }
                    return cost;
                }
            }
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
            {
                return this.GetSize() - 1;
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
            Move thisLocation = this.GetLocationAt(time);
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
            for (int time = 0; time < this.locationAtTimes.Count; time++)
            {
                Debug.WriteLine($"|{this.GetLocationAt(time)}|");
            }
        }

        public override string ToString()
        {
            string s = "";
            for (int time = 0; time < this.locationAtTimes.Count; time++)
            {
                s += $"|{this.GetLocationAt(time)}|\n";
            }
            return s;
        }

        public static SinglePlan[] GetSinglePlans(WorldState goalState) // FIXME: Duplication with other methods.
        {
            LinkedList<Move>[] allroutes = new LinkedList<Move>[goalState.allAgentsState.Length];
            for (int i = 0; i < allroutes.Length; i++)
                allroutes[i] = new LinkedList<Move>();

            WorldState currentNode = goalState;
            while (currentNode != null)
            {
                for (int i = 0; i < allroutes.Length; i++)
                    allroutes[i].AddFirst(currentNode.GetSingleAgentMove(i));
                currentNode = currentNode.prevStep;
            }

            SinglePlan[] ans = new SinglePlan[goalState.allAgentsState.Length];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = new SinglePlan(allroutes[i], goalState.allAgentsState[i].agent.agentNum);
            }
            return ans;
        }

        /// <summary>
        /// Creates SinglePlans with agentIndex as agentNum. Not suitable for subproblems.
        /// </summary>
        /// <param name="allRoutes"></param>
        /// <returns></returns>
        public static SinglePlan[] GetSinglePlans(LinkedList<Move>[] allRoutes)
        {
            SinglePlan[] ans = new SinglePlan[allRoutes.Length];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = new SinglePlan(allRoutes[i], i);
            }
            return ans;
        }
    }
}
