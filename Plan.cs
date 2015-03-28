using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace CPF_experiment
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
                        Debug.Assert(false, "Continuing a plan doesn't start from the same state");
                }
                this.locationsAtTimes.AddLast(newLocationsAtTime);
            }
        }

        // TODO: Add GetCost and GetMakespan methods!

        ///// <summary>
        ///// Not used. Creates SinglePlans with possibly wrong agentNum (agent index used for agentNum)
        ///// </summary>
        ///// <returns></returns>
        //public SinglePlan[] GetSinglePlans()
        //{
        //    SinglePlan[] ans = new SinglePlan[this.locationsAtTimes.First().Count];
        //    for (int i = 0; i < ans.Length; i++)
        //    {
        //        ans[i] = new SinglePlan(this, i);
        //    }
        //    return ans;
        //}

        /// <summary>
        /// Returns the location of the agents at a given time. 
        /// If the requested time is after the last step of the plan,
        /// the agents are assumed to stay at their final location.
        /// </summary>
        /// <param name="time">The requested time</param>
        /// <returns>A list of Moves that are the locations of the different agents at the requested time</returns>
        public List<Move> GetLocationsAt(int time)
        {
            time = Math.Min(time, this.locationsAtTimes.Count - 1);
            return this.locationsAtTimes.ElementAt<List<Move>>(time); // FIXME: Expensive!
        }

        public LinkedList<List<Move>> GetLocations()
        {
            return this.locationsAtTimes;
        }

        /// <summary>
        /// NOT the cost, which:
        /// A) could depend on steps taken before solving started,
        /// B) is 1 smaller than the size (a plan that starts at the goal costs zero)
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
                Console.WriteLine();
            }
        }

        public HashSet<TimedMove> AddPlanToHashSet(HashSet<TimedMove> addTo, int until)
        {
            for (int i = 1; i <= until; i++)
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
        /// Not used
        /// </summary>
        /// <param name="plan"></param>
        /// <param name="agentIndex"></param>
        /// <param name="agentNum"></param>
        public SinglePlan(Plan plan, int agentIndex, int agentNum)
        {
            this.agentNum = agentNum;
            this.locationAtTimes = new List<Move>();
            foreach (List<Move> movesAtTimestep in plan.GetLocations())
            {
                this.locationAtTimes.Add(movesAtTimestep.ElementAt<Move>(agentIndex));
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

        public override bool Equals(object obj) // TODO: Implement GetHashCode!
        {
            SinglePlan other = (SinglePlan)obj;
            return this.agentNum == other.agentNum && this.locationAtTimes.SequenceEqual<Move>(other.locationAtTimes);
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
                        Debug.Assert(false, "Continuing a plan doesn't start from the same state");
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
            if (Constants.Variant == Constants.ProblemVariant.ORIG)
            {
                return this.GetSize() - 1;
            }
            else if (Constants.Variant == Constants.ProblemVariant.NEW)
            {
                int cost = 0;
                Move goal = this.locationAtTimes.Last<Move>(); // Assuming the plan ends at the goal
                for (int i = 1; i < this.locationAtTimes.Count; i++) // The beginning position isn't a move
                {
                    Move move = this.locationAtTimes[i];
                    if (move.x == goal.x && move.y == goal.y && move.direction == Move.Direction.Wait) // Waiting at the goal is free
                        continue;
                    cost += 1;
                }
                return cost;
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
        public void PrintPlan()
        {
            for (int time = 0; time < this.locationAtTimes.Count; time++)
            {
                Console.Write("|");
                Console.Write(this.GetLocationAt(time).ToString() + "|");
                Console.WriteLine();
            }
        }

        public override string ToString()
        {
            string s = "";
            for (int time = 0; time < this.locationAtTimes.Count; time++)
            {
                s += "|" + this.GetLocationAt(time).ToString() + "|\n";
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
