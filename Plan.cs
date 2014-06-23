using System;
using System.Collections.Generic;
using System.Linq;

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
        /// Reconstructs the plan by goind backwards from the goal.
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
        /// the first timestep of the other plan is skipped
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
                }
                this.locationsAtTimes.AddLast(newLocationsAtTime);
            }
        }

        // TODO: Add GetCost and GetMakespan methods!

        public SinglePlan[] GetSinglePlans()
        {
            LinkedList<Move>[] allroutes = new LinkedList<Move>[this.locationsAtTimes.First().Count];
            for (int i = 0; i < allroutes.Length; i++)
            {
                allroutes[i] = new LinkedList<Move>();
            }
            foreach (List<Move> timeStep in this.locationsAtTimes)
            {
                for (int j = 0; j < timeStep.Count; ++j)
                {
                    allroutes[j].AddLast(timeStep[j]);
                }
            }

            SinglePlan[] ans = new SinglePlan[this.locationsAtTimes.First().Count];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = new SinglePlan(allroutes[i], i);
            }
            return ans;
        }

        /// <summary>
        /// Returns the location of the agents at a given time. 
        /// If the requested time is after the last step of the plan,
        /// the agents are assumed to stay at their final location.
        /// </summary>
        /// <param name="time">The requested time</param>
        /// <returns>A list of points that are the locations of the different agents at the requested time</returns>
        public List<Move> GetLocationsAt(int time)
        {
            time = Math.Min(time, this.locationsAtTimes.Count - 1);
            return this.locationsAtTimes.ElementAt<List<Move>>(time); // FIXME: Expensive!
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

            // TODO: Think about better implementation of this
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
        int agentIndex;

        /// <summary>
        /// Not used
        /// </summary>
        /// <param name="goalState"></param>
        /// <param name="agentIndex"></param>
        public SinglePlan(WorldState goalState, int agentIndex)
        {
            this.agentIndex = agentIndex;
            WorldState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.GetSingleAgentMove(agentIndex));
                currentNode = currentNode.prevStep;
            }
            this.locationAtTimes = locations.ToList<Move>();
        }

        public SinglePlan(AgentState goalState)
        {
            this.agentIndex = 0;
            AgentState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.GetMove());
                currentNode = currentNode.prev;
            }
            this.locationAtTimes = locations.ToList<Move>();
        }

        public SinglePlan(LinkedList<Move> route, int agentIndex)
        {
            this.agentIndex = agentIndex;
            locationAtTimes = route.ToList<Move>();
        }

        public SinglePlan(SinglePlan cpy)
        {
            this.locationAtTimes = cpy.locationAtTimes.ToList<Move>(); // Behavior change: used to do a deep copy, with cloned moves.
            this.agentIndex = cpy.agentIndex;
        }

        public Move GetLocationAt(int time)
        {
            if (time < this.locationAtTimes.Count)
                return this.locationAtTimes[time];
            else
                return this.locationAtTimes[this.locationAtTimes.Count - 1];
        }

        public override bool Equals(object obj)
        {
            SinglePlan other = (SinglePlan)obj;
            return this.agentIndex == other.agentIndex && this.locationAtTimes.SequenceEqual<Move>(other.locationAtTimes);
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
                    if (this.locationAtTimes[this.locationAtTimes.Count - 1].Equals(newLocationAtTime)) // Order important - we're comparing a Move to a TimedMove
                        continue;
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
            int ans = this.locationAtTimes.Count;
            Move goal = this.locationAtTimes[ans - 1];
            while (ans >= 2 && goal.x == locationAtTimes[ans - 2].x && goal.y == locationAtTimes[ans - 2].y)
                ans--;
            return ans;
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

            if (thisLocation.IsColliding(otherLocation) == true)
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

        public HashSet<TimedMove> AddPlanToHashSet(HashSet<TimedMove> addTo, int until)
        {
            for (int i = 0; i <= until; i++)
            {
                Move step = this.GetLocationAt(i);
                addTo.Add(new TimedMove(step, i));
            }
            return addTo;
        }

        public static SinglePlan[] GetSinglePlans(WorldState goalState) // FIXME: Duplication with other methods.
        {
            WorldState currentNode = goalState;
            LinkedList<List<Move>> locationsAtTimes = new LinkedList<List<Move>>();
            while (currentNode != null)
            {
                locationsAtTimes.AddFirst(currentNode.GetAgentsMoves());
                currentNode = currentNode.prevStep;
            }

            LinkedList<Move>[] allroutes = new LinkedList<Move>[locationsAtTimes.First().Count];
            for (int i = 0; i < allroutes.Length; i++)
            {
                allroutes[i] = new LinkedList<Move>();
            }
            foreach (List<Move> timeStep in locationsAtTimes)
            {
                for (int j = 0; j < timeStep.Count; ++j)
                {
                    allroutes[j].AddLast(timeStep[j]);
                }
            }

            SinglePlan[] ans = new SinglePlan[locationsAtTimes.First().Count];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = new SinglePlan(allroutes[i], i);
            }
            return ans;
        }

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
