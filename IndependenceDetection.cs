using System;
using System.Linq;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Diagnostics;

//using MongoDB.Bson;
//using MongoDB.Driver;

namespace CPF_experiment
{
    class IndependenceDetection : ISolver
    {
        // The key of the illegal moves table in the ProblemInstance (used in ImprovedID())
        public static string ILLEGAL_MOVES_KEY = "ID-reserved";
        // The key of the maximal solution cost of the agent group in the ProblemInstance (used in ImprovedID())
        public static string MAXIMUM_COST_KEY = "ID-max-cost";
        // The key of the conflict avoidance table
        public static string CONFLICT_AVOIDANCE = "ID-ConflictAvoidance";

        protected LinkedList<AgentsGroup> allGroups;
        protected ProblemInstance instance;
        protected int expanded;
        protected int generated;
        protected int accExpanded;
        protected int accGenerated;
        public int totalCost;
        protected Run runner;

        /// <summary>
        /// The complete plan for all the agents that was found.
        /// </summary>
        public Plan plan;
        
        protected int maxGroupSize;
        protected int minGroupSize;
        protected int accMaxGroupSize;
        protected int accMinGroupSize;
        public ISolver groupSolver;
        public ISolver singleAgentSolver;
        protected HeuristicCalculator heuristic;
        private ISet<Conflict> allConflicts;
        private int solutionDepth;
        private Dictionary<TimedMove, List<int>> conflictAvoidance;
        private int maxSolutionCostFound;
        private string name;

        public IndependenceDetection(HeuristicCalculator heuristic)
            : this(new ClassicAStar(heuristic), new AStarWithOD(heuristic), heuristic) { }

        public IndependenceDetection(ISolver singleAgentSolver, ISolver groupSolver, HeuristicCalculator heuristic)
        {
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
            this.heuristic = heuristic;
        }

        public void Clear()
        {
            this.allGroups.Clear();
            this.groupSolver.Clear();
            this.allConflicts.Clear();
            this.conflictAvoidance.Clear();
        }

        public void Setup(ProblemInstance instance, Run runner)
        {
            this.instance = instance;
            this.runner = runner;
            this.totalCost = 0;
            this.ClearStatistics();
            //this.accMaxGroupSize = 1;
            this.conflictAvoidance = new Dictionary<TimedMove, List<int>>();
            this.allConflicts = new HashSet<Conflict>();
            this.allGroups = new LinkedList<AgentsGroup>();
            // Initialize the agent group collection with a group for every agent
            foreach (AgentState agentStartState in instance.m_vAgents)
                this.allGroups.AddLast(new AgentsGroup(this.instance, new AgentState[1] { agentStartState }, this.singleAgentSolver, this.groupSolver));
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.heuristic = heuristic;
            this.groupSolver.SetHeuristic(heuristic);
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.heuristic;
        }

        public virtual String GetName() { return groupSolver.GetName() + "+ID"; }

        /// <summary>
        /// Calculate the full plan for all the agents that has been found by the algorithm
        /// </summary>
        public Plan CalculateJointPlan() 
        {
            AgentsGroup[] sortedGroups = this.allGroups.ToArray<AgentsGroup>();
            Array.Sort<AgentsGroup>(sortedGroups,
                (x, y) => x.allAgentsState[0].agent.agentNum.CompareTo(y.allAgentsState[0].agent.agentNum));
            IEnumerable<Plan> plans = sortedGroups.Select<AgentsGroup, Plan>(group => group.GetPlan());
            return new Plan(plans);
        }

        public Plan GetPlan()
        {
            return this.plan;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            // TODO: Use the solver's statistics, as done in CBS.
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group Size");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Min Group Size");
            output.Write(Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)//, BsonDocument row)
        {
            Console.WriteLine("Total Expanded Nodes: {0}", this.expanded);
            Console.WriteLine("Total Generated Nodes: {0}", this.generated);

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);

            // Compute and output the maximum group size
            this.maxGroupSize = 0;
            this.solutionDepth = 0;
            this.minGroupSize = this.instance.m_vAgents.Length;
            foreach (AgentsGroup group in this.allGroups)
            {
                this.solutionDepth += group.depthOfSolution;
                if (group.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = group.allAgentsState.Length;
                if (group.allAgentsState.Length < this.minGroupSize)
                    this.minGroupSize = group.allAgentsState.Length;
            }

            Console.WriteLine("Max Group: {0}", this.maxGroupSize);
            Console.WriteLine("Min Group: {0}", this.minGroupSize);

            output.Write(this.maxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.minGroupSize + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 4;
            }
        }

        public void ClearStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
            this.maxGroupSize = 1;
            this.minGroupSize = instance.m_vAgents.Length;
            this.heuristic.ClearStatistics();
        }

        public void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
            this.accMaxGroupSize = 1;
            this.accMinGroupSize = this.instance.m_vAgents.Length;
        }

        public void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accMaxGroupSize = Math.Max(this.accMaxGroupSize, this.maxGroupSize);
            this.accMinGroupSize = Math.Min(this.accMinGroupSize, this.minGroupSize);
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGenerated);

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);

            Console.WriteLine("{0} Accumualted Max Group (Low-Level): {1}", this, this.accMaxGroupSize);
            Console.WriteLine("{0} Accumulated Min Group (Low-Level): {1}", this, this.accMinGroupSize);

            output.Write(this.accMaxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.accMinGroupSize + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Also calculates min group size and max solution depth on the way.
        /// FIXME: Code dup!
        /// </summary>
        /// <returns></returns>
        public int GetMaxGroupSize()
        {
            this.solutionDepth = 0;
            this.maxGroupSize = 0;
            this.minGroupSize = int.MaxValue;
            foreach (AgentsGroup group in this.allGroups)
            {
                this.solutionDepth += group.depthOfSolution;
                if (group.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = group.allAgentsState.Length;
                if (group.allAgentsState.Length < this.minGroupSize)
                    this.minGroupSize = group.allAgentsState.Length;
            }
            return this.maxGroupSize;
        }
        /// <summary>
        /// Simulates the execution of the plans found for the different groups. 
        /// If there are conflicting plans - return the conflicting groups.
        /// </summary>
        /// <returns>A conflict object with data about the found conflict, or null if no conflict exists</returns>
        public Conflict FindConflictingGroups()
        {
            if (this.allGroups.Count == 1) return null;
            
            // Find the longest plan among all the groups
            int maxPlanSize = this.allGroups.Select<AgentsGroup, int>(group => group.GetPlan().GetSize()).Max();

            // Check in every time step that the plans do not collide
            for(int time = 1 ; time < maxPlanSize ; time++) // Assuming no conflicts exist in time zero.
            {
                int i1 = -1;
                // Check all pairs of groups for a conflict at the given time step
                foreach (AgentsGroup group1 in this.allGroups)
                {
                    i1++;
                    Plan group1Plan = group1.GetPlan();
                    int i2 = -1;
                    foreach (AgentsGroup group2 in this.allGroups)
                    {
                        i2++;
                        Plan group2Plan = group2.GetPlan();
                        if (i1 < i2 &&
                            group1Plan.IsColliding(time, group2Plan))
                            return new Conflict(group1, group2, time);
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Search for an optimal solution using the Simple Independence Detection algorithm in Trevor Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool SimpleID(Run runner)
        {
            while (true)
            {
                Conflict conflict = FindConflictingGroups();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                AgentsGroup compositeGroup = this.JoinGroups(conflict);
                
                // Solve composite group with A*
                bool solved = compositeGroup.Solve(runner);
                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost;
                    return false;
                }

                allGroups.AddFirst(compositeGroup);
            }
            return true;
        }

        public bool debug = false;

        /// <summary>
        /// Search for an optimal solution using the Independence Detection algorithm in Standley's paper,
        /// which utilises a CAT.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ImprovedID(Run runner)
        {
            while (true)
            {
                Conflict conflict = FindConflictingGroups();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                
                // Try to solve the current conflict by replanning one of the groups
                if (this.allConflicts.Contains(conflict) == false)
                {
                    // Add to all conflicts to prevent trying to replan this conflict again
                    this.allConflicts.Add(conflict);

                    // Add plan of group2 to illegal moves table and replan group1 with equal cost
                    if ((conflict.timeOfConflict < conflict.group1.GetPlan().GetSize() - 1) ||
                        (conflict.group1.allAgentsState.Length > 1)) // Otherwise the conflict is while a single agent is at its goal, no chance of an alternate path with the same cost that avoids the conflict
                    {
                        conflict.group1.removeGroupFromCAT(conflictAvoidance);
                        bool resolved = conflict.group1.ReplanUnderConstraints(conflict.group2.GetPlan(), runner);
                        conflict.group1.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                        if (resolved == true)
                            continue;
                    }
                    // Add plan of group1 to illegal moves table and replan group2 with equal cost
                    if ((conflict.timeOfConflict < conflict.group2.GetPlan().GetSize() - 1) ||
                        (conflict.group2.allAgentsState.Length > 1))
                    {
                        conflict.group2.removeGroupFromCAT(conflictAvoidance);
                        bool resolved = conflict.group2.ReplanUnderConstraints(conflict.group1.GetPlan(), runner);
                        conflict.group2.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                        if (resolved == true)
                            continue;
                    }
                }

                // Groups are conflicting - need to join them to a single group
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                // Remove both groups from avoidance table
                conflict.group1.removeGroupFromCAT(conflictAvoidance);
                conflict.group2.removeGroupFromCAT(conflictAvoidance);
                if (this.debug)
                    Debug.WriteLine("Merging " + conflict);
                AgentsGroup compositeGroup = this.JoinGroups(conflict);

                compositeGroup.instance.parameters[CONFLICT_AVOIDANCE] = conflictAvoidance;

                // Solve composite group with A*
                bool solved = compositeGroup.Solve(runner);

                if (compositeGroup.solutionCost > maxSolutionCostFound)
                    maxSolutionCostFound = compositeGroup.solutionCost;

                //add group to conflict avoidance table
                compositeGroup.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                allGroups.AddFirst(compositeGroup);

                this.expanded += compositeGroup.expanded;
                this.generated += compositeGroup.generated;

                if (compositeGroup.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = compositeGroup.allAgentsState.Length;

                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost; // Should be some error code from Constants
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Join the conflicting groups into a single group
        /// </summary>
        /// <param name="conflict">An object that describes the conflict</param>
        /// <returns>The composite group of agents</returns>
        protected virtual AgentsGroup JoinGroups(Conflict conflict)
        {
            return conflict.group1.Join(conflict.group2);
        }

        /// <summary>
        /// Run the A* algorithm with Standley's ID and OD improvements.
        /// </summary>
        /// <returns>true if optimal solution has been found</returns>
        public bool Solve()
        {
            bool solved;
            // Solve the single agent problems independently
            this.maxSolutionCostFound = 0;

            foreach (AgentsGroup group in this.allGroups)
            {
                group.instance.parameters[CONFLICT_AVOIDANCE] = this.conflictAvoidance;
                solved = group.Solve(runner);

                // Check if max time has been exceeded or search failed for another reason
                if (solved == false)
                {
                    this.totalCost = group.solutionCost; // Should be some error code from Constants.
                    this.Clear();
                    return false;
                }

                if (group.solutionCost > this.maxSolutionCostFound)
                    this.maxSolutionCostFound = group.solutionCost;
                // Add group to conflict avoidance table
                group.addGroupToCAT(this.conflictAvoidance, this.maxSolutionCostFound);

                this.expanded += group.expanded;
                this.generated += group.generated;
            }

            //solved = this.SimpleID(runner);
            solved = this.ImprovedID(runner);
            // Record found solution
            if (solved == true)
            {
                // Store solution details
                this.totalCost = this.allGroups.Select<AgentsGroup, int>(group => group.solutionCost).Sum();
                this.plan = this.CalculateJointPlan();
            }
            else
            {
                this.plan = null;
            }

            Console.WriteLine();
            Console.WriteLine(this.allGroups.Count + " - Independent Groups");
            Console.WriteLine(this.maxGroupSize + " - Size Of Largest ID Group");
            Console.WriteLine();
            return solved;
        }

        public override string ToString()
        {
            return GetName();
        }

        private void print()
        {
            Console.WriteLine("Expanded - " + expanded);
            Console.WriteLine("Generated - " + generated);
            Console.WriteLine("Total cost - " + totalCost);
        }

        public int GetExpanded() { return this.expanded; }
        public int GetGenerated() { return this.generated; }
        public int GetSolutionDepth() { return solutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
    }


    /// <summary>
    /// This class represents a group of agents that need to be solved together.
    /// </summary>
    class AgentsGroup
    {
        public AgentState[] allAgentsState;
        public int solutionCost;
        public ProblemInstance instance;
        public int expanded;
        public int generated;
        public int depthOfSolution;

        private ISolver singleAgentSolver;
        private ISolver groupSolver;
        private Plan plan;
        
        public AgentsGroup(ProblemInstance instance, AgentState[] allAgentsState, ISolver singleAgentSolver, ISolver groupSolver)
        {
            this.allAgentsState = allAgentsState;
            this.instance = instance.Subproblem(allAgentsState);
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
        }

        /// <summary>
        /// Solve the group of agents together.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns>true if optimal solution for the group of agents were found, false otherwise</returns>
        public bool Solve(Run runner)
        {
            ISolver relevantSolver = this.groupSolver;
            if (this.allAgentsState.Length == 1)
                relevantSolver = this.singleAgentSolver; // TODO: Consider using CBS's root trick to really get single agent paths fast. Though it won't respect illegal moves and such.

            relevantSolver.Setup(this.instance, runner);
            bool solved = relevantSolver.Solve();
            this.solutionCost = relevantSolver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.plan = relevantSolver.GetPlan();
            this.expanded = relevantSolver.GetExpanded();
            this.generated = relevantSolver.GetGenerated();
            this.depthOfSolution = relevantSolver.GetSolutionDepth();

            // Clear memory
            relevantSolver.Clear();
            return true;
        }

        /// <summary>
        /// Returns the plan for the group of agents. This is a collection of Moves for every time step until all the agents reach their goal.
        /// </summary>
        public Plan GetPlan()
        {
            return this.plan;
        }

        /// <summary>
        /// Joins this and another group to a single group with all of the agents together.
        /// </summary>
        /// <param name="other"></param>
        /// <returns>A new Trevor_Group object with the agents from both this and the other group</returns>
        public AgentsGroup Join(AgentsGroup other)
        {
            AgentState[] joinedAgentStates = new AgentState[allAgentsState.Length + other.allAgentsState.Length];
            this.allAgentsState.CopyTo(joinedAgentStates, 0);
            other.allAgentsState.CopyTo(joinedAgentStates, this.allAgentsState.Length);
            Array.Sort<AgentState>(joinedAgentStates, (x, y) => x.agent.agentNum.CompareTo(y.agent.agentNum));

            return new AgentsGroup(this.instance, joinedAgentStates, this.singleAgentSolver, this.groupSolver);
        }

        /// <summary>
        /// Returns the number of agents in the group.
        /// </summary>
        public int Size()
        {
            return this.allAgentsState.Length;
        }

        public override bool Equals(object obj) // TODO: Implement GetHashCode()
        {
            AgentsGroup other = (AgentsGroup)obj;
            return allAgentsState.SequenceEqual<AgentState>(other.allAgentsState);
        }

        /// <summary>
        /// Tries to find a plan for this group, that will not conflict with the given plan,
        /// and still has the same solution cost as the current solution cost.
        /// This is used in the ImprovedID() method.
        /// </summary>
        /// <param name="plan"></param>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ReplanUnderConstraints(Plan plan, Run runner)
        {
            int oldCost = this.solutionCost;
            Plan oldPlan = this.plan;
            HashSet<TimedMove> reserved = new HashSet<TimedMove>();
            plan.AddPlanToHashSet(reserved, Math.Max(plan.GetSize(), this.plan.GetSize()) - 1); // TODO: Why -1?

            this.instance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY] = reserved;
            this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = solutionCost;
            bool success = this.Solve(runner);
            this.instance.parameters.Remove(IndependenceDetection.ILLEGAL_MOVES_KEY);
            this.instance.parameters.Remove(IndependenceDetection.MAXIMUM_COST_KEY);
            if (success == false)
            {
                this.solutionCost = oldCost;
                this.plan = oldPlan;
            }
            return success;
        }

        public void addGroupToCAT(Dictionary<TimedMove, List<int>> CAT, int maxTimeStep)
        {
            if (this.plan == null)
                return;
            
            for (int i = 1; i <= maxTimeStep * 2; i++) // TODO: Use the ConflictAvoidanceTable class instead
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    TimedMove timedMove = new TimedMove(move, i);
                    if (CAT.ContainsKey(timedMove) == false)
                        CAT.Add(timedMove, new List<int>(this.allAgentsState.Length));
                    CAT[timedMove].AddRange(this.allAgentsState.Select<AgentState, int>(state => state.agent.agentNum));
                }
            }
        }

        public void removeGroupFromCAT(Dictionary<TimedMove, List<int>> CAT)
        {
            int i = 1;
            bool stop = false;
            while (stop == false)
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    var S = new TimedMove(move, i);
                    if (CAT.ContainsKey(S))
                        CAT.Remove(S);
                    else
                    {
                        stop = true;
                        break;
                    }
                }
                i++;
            }
        }

        public override string ToString()
        {
            string ans = "group {";
            foreach (var agentState in this.allAgentsState)
            {
                ans += agentState.agent.agentNum + ", ";
            }
            ans += "}";
            return ans;
        }
    }
}
