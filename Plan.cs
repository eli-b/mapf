using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// Returns a plan for a set of agents.
    /// </summary>
    public class Plan
    {
        private IList<LinkedList<Move>> locationsAtTime;

        /// <summary>
        /// Reconstructs the plan by goind backwards from the goal.
        /// </summary>
        /// <param name="goalState">The goal state from which to start going backwards</param>
        public Plan(WorldState goalState)
        {
            WorldState currentNode = goalState;
            this.locationsAtTime = new List<LinkedList<Move>>(); // TODO: Initialize list with #agents
            while (currentNode != null)
            {
                this.locationsAtTime.Insert(0,currentNode.GetAgentsMoves());
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
            this.locationsAtTime = new List<LinkedList<Move>>(); // TODO: Initialize list with #agents
            while (currentNode != null)
            {
                this.locationsAtTime.Insert(0, currentNode.GetMove());
                currentNode = currentNode.prev;
            }
        }

        public Plan(LinkedList<Move>[] route)
        {
            this.locationsAtTime = new List<LinkedList<Move>>();
            for (int i = 0; i < route[0].Count; i++)
            {
                locationsAtTime.Add(new LinkedList<Move>());
            }
            LinkedListNode<Move> move;
            foreach (LinkedList<Move> routePerAgent in route)
            {
                move = routePerAgent.First;
                for (int i = 0; i < locationsAtTime.Count; i++)
                {
                    this.locationsAtTime[i].AddLast(move.Value);
                    move = move.Next;
                }
            }
        }

        /// <summary>
        /// Reconstructs a plan by going backwards from a goal state, returning only moves in a full states
        /// and ignoring intermediate states.
        /// </summary>
        /// <param name="goalState"></param>
        public Plan(WorldStateWithOD goalState)
        {
            WorldState currentNode = goalState;
            this.locationsAtTime = new List<LinkedList<Move>>(); // TODO: Initialize list with #agents
            this.locationsAtTime.Insert(0, goalState.GetAgentsMoves());
            currentNode = currentNode.prevStep;
            int currentMakespan = goalState.makespan;
            while (currentNode != null)
            {
                // Insert agent's moves only if all the agents have moved (in order to return only full states and not intermediate states)
                if (currentNode.makespan < currentMakespan)
                {
                    this.locationsAtTime.Insert(0, currentNode.GetAgentsMoves());
                    currentMakespan = currentNode.makespan;
                }
                currentNode = currentNode.prevStep;
            }
        }

        /// <summary>
        /// Generates a big plan from a collection of smaller plans.
        /// </summary>
        /// <param name="subplans"></param>
        public Plan(IEnumerable<Plan> subplans)
        {
            int maxSize = subplans.Max(plan => plan.GetSize());
            this.locationsAtTime = new List<LinkedList<Move>>();
            LinkedList<Move> allMoves;

            for (int time = 0; time < maxSize; time++)
            {
                allMoves = new LinkedList<Move>();
                foreach (Plan plan in subplans)
                    foreach (Move move in plan.GetLocationsAt(time))
                        allMoves.AddLast(move);

                this.locationsAtTime.Insert(time,allMoves);
            }
        }

        public Plan(IEnumerable<SinglePlan> subplans)
        {
            int maxSize = subplans.Max(plan => plan.GetSize());
            this.locationsAtTime = new List<LinkedList<Move>>();
            LinkedList<Move> allMoves;

            for (int time = 0; time < maxSize; time++)
            {
                allMoves = new LinkedList<Move>();
                foreach (SinglePlan plan in subplans)
                {
                    allMoves.AddLast(plan.GetLocationsAt(time));
                }

                this.locationsAtTime.Insert(time, allMoves);
            }
        }

        public Plan(Plan cpy)
        {
            this.locationsAtTime = new List<LinkedList<Move>>();
            LinkedList<Move> movesInStep;

            foreach (LinkedList<Move> cpyStep in cpy.locationsAtTime)
            {
                movesInStep = new LinkedList<Move>();
                foreach (Move move in cpyStep)
                {
                    movesInStep.AddLast(move);
                }
                this.locationsAtTime.Add(movesInStep);
            }
        }

        public SinglePlan[] getSinglePlans()
        {
            LinkedList<Move>[] allroutes = new LinkedList<Move>[this.locationsAtTime.First().Count];
            for (int i = 0; i < allroutes.Length; i++)
            {
                allroutes[i] = new LinkedList<Move>();
            }
            SinglePlan[] ans = new SinglePlan[this.locationsAtTime.First().Count];
            int j;
            foreach (LinkedList<Move> timeStep in this.locationsAtTime)
            {
                j = 0;
                foreach (Move agentStep in timeStep)
                {
                    allroutes[j].AddLast(agentStep);
                    j++;
                }
            }
            for (int i = 0; i < allroutes.Length; i++)
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
        public LinkedList<Move> GetLocationsAt(int time)
        {
            if (time < this.locationsAtTime.Count)
                return this.locationsAtTime[time];
            else
                return this.locationsAtTime[this.locationsAtTime.Count - 1];
        }

        public int GetSize()
        {
            return this.locationsAtTime.Count;
        }

        /// <summary>
        /// Check if this plan collides with another plan at a given time
        /// </summary>
        /// <param name="time">The time at which to check if the collision occured</param>
        /// <param name="otherPlan">The plan to check against</param>
        public bool IsColliding(int time, Plan otherPlan)
        {
            LinkedList<Move> thisLocations = this.GetLocationsAt(time);
            LinkedList<Move> otherLocations = otherPlan.GetLocationsAt(time);

            // TODO: Think about better implementation of this
            foreach (Move aMove in thisLocations)
                foreach (Move bMove in otherLocations)
                    if (aMove.isColliding(bMove) == true)
                        return true;

            return false;
        }

        /// <summary>
        /// Prints the plan to the Console. 
        /// This is used for debugging purposes.
        /// </summary>
        public void PrintPlan()
        {
            for (int time = 0; time < this.locationsAtTime.Count; time++)
            {
                Console.Write("|");
                foreach (Move aMove in this.GetLocationsAt(time))
                {
                    Console.Write(aMove.ToString() + "|");
                }                
                Console.WriteLine();
            }
        }

        public HashSet<TimedMove> addPlanToHashSet(HashSet<TimedMove> addTo, int until)
        {
            for (int i = 1; i <= until; i++)
            {
                LinkedList<Move> step = this.GetLocationsAt(i);
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
        public Move[] locationsAtTime { get; private set; }
        int agentIndex;

        public SinglePlan(WorldState goalState,int agentIndex)
        {
            this.agentIndex = agentIndex;
            WorldState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.getSingleAgentMove(agentIndex));
                currentNode = currentNode.prevStep;
            }
        }

        public SinglePlan(AgentState goalState)
        {
            this.agentIndex = 0;
            AgentState currentNode = goalState;
            LinkedList<Move> locations = new LinkedList<Move>();
            while (currentNode != null)
            {
                locations.AddFirst(currentNode.GetMove().First);
                currentNode = currentNode.prev;
            }
            this.locationsAtTime = locations.ToArray();
        }

        public SinglePlan(LinkedList<Move> route, int agentIndex)
        {
            this.agentIndex = agentIndex;
            locationsAtTime = route.ToArray();
        }

        ///// <summary>
        ///// Reconstructs a plan by going backwards from a goal state, returning only moves in a full states
        ///// and ignoring intermediate states.
        ///// </summary>
        ///// <param name="goalState"></param>
        //public SinglePlan(WorldStateWithOD goalState)
        //{
        //    WorldState currentNode = goalState;
        //    LinkedList<Move> locations = new LinkedList<Move>();
        //    this.locationsAtTime.Insert(0, goalState.GetAgentsMoves());
        //    currentNode = currentNode.prevStep;
        //    int currentMakespan = goalState.makespan;
        //    while (currentNode != null)
        //    {
        //        // Insert agent's moves only if all the agents have moved (in order to return only full states and not intermediate states)
        //        if (currentNode.makespan < currentMakespan)
        //        {
        //            this.locationsAtTime.Insert(0, currentNode.GetAgentsMoves());
        //            currentMakespan = currentNode.makespan;
        //        }
        //        currentNode = currentNode.prevStep;
        //    }
        //}

        public SinglePlan(SinglePlan cpy)
        {
            this.locationsAtTime = new Move[cpy.locationsAtTime.Length];
            for (int i = 0; i < locationsAtTime.Length; i++)
            {
                this.locationsAtTime[i] = new Move(cpy.locationsAtTime[i]);
            }
            this.agentIndex = cpy.agentIndex;
        }

        public Move GetLocationsAt(int time)
        {
            if (time < this.locationsAtTime.Length)
                return this.locationsAtTime[time];
            else
                return this.locationsAtTime[this.locationsAtTime.Length - 1];
        }

        public int GetSize()
        {
            int ans = this.locationsAtTime.Length;
            Move goal = this.locationsAtTime[ans - 1];
            while (ans > 1 && goal.x == locationsAtTime[ans - 2].x && goal.y == locationsAtTime[ans - 2].y)
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
            Move thisLocation = this.GetLocationsAt(time);
            Move otherLocation = otherPlan.GetLocationsAt(time);

            if (thisLocation.isColliding(otherLocation) == true)
                return true;

            return false;
        }

        /// <summary>
        /// Prints the plan to the Console. 
        /// This is used for debugging purposes.
        /// </summary>
        public void PrintPlan()
        {
            for (int time = 0; time < this.locationsAtTime.Length; time++)
            {
                Console.Write("|");
                Console.Write(this.GetLocationsAt(time).ToString() + "|");
                Console.WriteLine();
            }
        }

        public HashSet<TimedMove> addPlanToHashSet(HashSet<TimedMove> addTo, int until)
        {
            for (int i = 0; i <= until; i++)
            {
                Move step = this.GetLocationsAt(i);
                addTo.Add(new TimedMove(step, i));
            }
            return addTo;
        }

        public static SinglePlan[] getSinglePlans(WorldState goalState)
        {
            WorldState currentNode = goalState;
            List<LinkedList<Move>> locationsAtTime = new List<LinkedList<Move>>();
            while (currentNode != null)
            {
                locationsAtTime.Add(currentNode.GetAgentsMoves());
                currentNode = currentNode.prevStep;
            }

            LinkedList<Move>[] allrouts = new LinkedList<Move>[locationsAtTime.First().Count];
            for (int i = 0; i < allrouts.Length; i++)
            {
                allrouts[i] = new LinkedList<Move>();
            }
            SinglePlan[] ans = new SinglePlan[locationsAtTime.First().Count];
            int j;
            foreach (LinkedList<Move> timeStep in locationsAtTime)
            {
                j = 0;
                foreach (Move agentStep in timeStep)
                {
                    allrouts[j].AddFirst(agentStep);
                    j++;
                }
            }
            for (int i = 0; i < allrouts.Length; i++)
            {
                ans[i] = new SinglePlan(allrouts[i], i);
            }
            return ans;
        }

        public static SinglePlan[] getSinglePlans(LinkedList<Move>[] allRouts)
        {
            SinglePlan[] ans = new SinglePlan[allRouts.Length];
            for (int i = 0; i < ans.Length; i++)
            {
                ans[i] = new SinglePlan(allRouts[i], i);
            }
            return ans;
        }
    }
}
