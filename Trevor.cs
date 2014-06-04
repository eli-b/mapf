using System;
using System.Linq;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    class Trevor : ISolver
    {
        // The key of the illegal moves table in the ProblemInstance (used in IndependenceDetection())
        public static string ILLEGAL_MOVES_KEY = "reserved";
        // The key of the maximal solution cost of the agent group in the ProblemInstance (used in IndependenceDetection())
        public static string MAXIMUM_COST_KEY = "cost";
        // The key of the conflict avoidance table
        public static string CONFLICT_AVOIDANCE = "ConflictAvoidance";

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
        public Plan foundPlan;
        
        protected int maxGroup;
        protected int minGroup;
        protected int accMaxGroup;
        protected int accMinGroup;
        public ISolver groupSolver;
        protected HeuristicCalculator heuristic;
        private IList<Conflict> allConflicts;
        private int maxSolutionDepth;
        private HashSet<TimedMove> conflictAvoidance;
        private int maxDepth;
        private string name;

        public Trevor() : this(new AStarWithOD())
        {}

        public Trevor(ISolver groupSolver)
        {
            this.groupSolver = groupSolver;
            this.allConflicts = new List<Conflict>();
            this.allGroups = new LinkedList<AgentsGroup>();
            this.name = groupSolver + "+ID";
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
            this.expanded = 0;
            this.generated = 0;
            this.totalCost = 0;
            this.maxGroup = 1;
            this.minGroup = instance.m_vAgents.Length;
            this.accMaxGroup = 1;
            this.minGroup = instance.m_vAgents.Length;
            this.conflictAvoidance = new HashSet<TimedMove>();
            // Initialize the agent group collection with a group for every agent
            foreach (AgentState agentStartState in instance.m_vAgents)
                this.allGroups.AddFirst(new AgentsGroup(this.instance, new AgentState[1] { agentStartState }, this.groupSolver));
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

        public virtual String GetName() { return name; }

        /// <summary>
        /// Calculate the full plan for all the agents that has been found by the algorithm
        /// </summary>
        public Plan CalculateJointPlan() 
        {
            IList<Plan> plans = new List<Plan>();

            // Find the longest plan among all the groups
            foreach (AgentsGroup group in this.allGroups)
                plans.Add(group.GetPlan());

            return new Plan(plans);
        }

        public Plan GetPlan()
        {
            return this.foundPlan;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            // TODO: Use the solver's statistics, as done in CBS.
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Min Group");
            output.Write(Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes: {0}", this.expanded);
            Console.WriteLine("Total Generated Nodes: {0}", this.generated);

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);

            // Output the maximum group size
            this.maxGroup = 0;
            this.maxSolutionDepth = 0;
            this.minGroup = this.instance.m_vAgents.Length;

            foreach (AgentsGroup group in this.allGroups)
            {
                if (group.allAgentsState.Length > this.maxGroup)
                {
                    this.maxGroup = group.allAgentsState.Length;
                    this.maxSolutionDepth = group.depthOfSolution; // The solution depth of the max size group is always the max solution depth?
                }
                if (group.allAgentsState.Length < this.minGroup)
                    this.minGroup = group.allAgentsState.Length;
            }

            Console.WriteLine("Max Group: {0}", this.maxGroup);
            Console.WriteLine("Min Group: {0}", this.minGroup);

            output.Write(this.maxGroup + Run.RESULTS_DELIMITER);
            output.Write(this.minGroup + Run.RESULTS_DELIMITER);
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
            // Own statistics cleared on Setup.
            this.heuristic.ClearStatistics();
        }

        public void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
        }

        public void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accMaxGroup = Math.Max(this.accMaxGroup, this.maxGroup);
            this.accMinGroup = Math.Min(this.accMinGroup, this.minGroup);
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (Low-Level): {0}", this.accExpanded);
            Console.WriteLine("Total Generated Nodes (Low-Level): {0}", this.accGenerated);

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);

            Console.WriteLine("Max Group (Low-Level): {0}", this.accMaxGroup);
            Console.WriteLine("Min Group (Low-Level): {0}", this.accMinGroup);

            output.Write(this.accMaxGroup + Run.RESULTS_DELIMITER);
            output.Write(this.accMinGroup + Run.RESULTS_DELIMITER);
        }

        public int GetMaxGroupSize()
        {
            foreach (AgentsGroup group in this.allGroups)
            {
                if (group.allAgentsState.Length > this.maxGroup)
                {
                    this.maxGroup = group.allAgentsState.Length;
                    this.maxSolutionDepth = group.depthOfSolution;
                }
                if (group.allAgentsState.Length < this.minGroup)
                    this.minGroup = group.allAgentsState.Length;
            }
            return this.maxGroup;
        }
        /// <summary>
        /// Simulates the execution of the plans found for the different groups. 
        /// If there are conflicting plans - return the conflicting groups.
        /// </summary>
        /// <returns>A conflict object with data about the found conflict, or null if no conflict exists</returns>
        public Conflict FindConflictingGroups()
        {
            if (this.allGroups.Count == 1) return null;
            int maxPlanSize = 0;
            int planSize = -1;
            
            // Find the longest plan among all the groups
            foreach (AgentsGroup group in this.allGroups)
            {
                planSize = group.GetPlan().GetSize();
                if (planSize > maxPlanSize)
                    maxPlanSize = planSize;
            }

            // Check in every time step that the plans do not collide
            LinkedListNode<AgentsGroup> group1Iterator;
            LinkedListNode<AgentsGroup> group2Iterator;
            Plan group1Plan;
            Plan group2Plan;
            for(int time = 1 ; time < maxPlanSize ; time++)
            {
                // Check all pairs of groups if they are conflicting at the given time step
                group1Iterator = this.allGroups.First;
                while (group1Iterator != null)
                {
                    group1Plan = group1Iterator.Value.GetPlan();
                    group2Iterator = group1Iterator.Next;
                    while (group2Iterator != null)
                    {
                        group2Plan = group2Iterator.Value.GetPlan();
                        if (group1Plan.IsColliding(time,group2Plan))
                            return new Conflict(group1Iterator.Value, group2Iterator.Value,time);

                        group2Iterator = group2Iterator.Next;
                    }
                    group1Iterator = group1Iterator.Next;
                }
            }
            return null;
        }

        /// <summary>
        /// Search for an optimal solution using the Simple Independence Detection algorithm in Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool SimpleIndependenceDetection(Run runner)
        {
            Conflict conflict;
            bool solved;
            AgentsGroup compositeGroup;
            while (true)
            {
                conflict = FindConflictingGroups();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                compositeGroup = this.JoinGroups(conflict);
                
                // Solve composite group with A*
                solved = compositeGroup.Solve(runner);
                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost;
                    return false;
                }

                allGroups.AddFirst(compositeGroup);
            }
            return true;
        }

        /// <summary>
        /// Search for an optimal solution using the Simple Independence Detection algorithm in Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool IndependenceDetection(Run runner)
        {
            Conflict conflict;
            bool solved;
            AgentsGroup compositeGroup;
            while (true)
            {
                conflict = FindConflictingGroups();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                
                // Check if can solve the current conflict by replanning one of the groups
                if (this.allConflicts.Contains(conflict) == false)
                {
                    // Add to all conflicts to prevent trying to replan this conflict again
                    this.allConflicts.Add(conflict);

                    // Add plan of group2 to illegal moves table and replan group1 with equal cost
                    if (conflict.timeOfConflict < conflict.group1.GetPlan().GetSize())
                    {
                        conflict.group1.removeGroupFromCA(conflictAvoidance);
                        if (conflict.group1.ReplanUnderConstraints(conflict.group2.GetPlan(), runner) == true)
                        {
                            conflict.group1.addGroupToCA(conflictAvoidance, maxDepth);
                            continue;
                        }
                        conflict.group1.addGroupToCA(conflictAvoidance, maxDepth);
                    }
                    // Add plan of group1 to illegal moves table and replan group2 with equal cost
                    if (conflict.timeOfConflict < conflict.group2.GetPlan().GetSize() - 1)
                    {
                        conflict.group2.removeGroupFromCA(conflictAvoidance);
                        if (conflict.group2.ReplanUnderConstraints(conflict.group1.GetPlan(), runner) == true)
                        {
                            conflict.group2.addGroupToCA(conflictAvoidance, maxDepth);
                            continue;
                        }
                        conflict.group2.addGroupToCA(conflictAvoidance, maxDepth);
                    }
                }

                // Groups are conflicting - need to join them to a single group
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                // Remove both groups from avoidance table
                conflict.group1.removeGroupFromCA(conflictAvoidance);
                conflict.group2.removeGroupFromCA(conflictAvoidance);

                compositeGroup = this.JoinGroups(conflict);

                compositeGroup.instance.parameters[CONFLICT_AVOIDANCE] = conflictAvoidance;

                // Solve composite group with A*
                solved = compositeGroup.Solve(runner);

                if (compositeGroup.solutionCost > maxDepth)
                    maxDepth = compositeGroup.solutionCost;

                //add group to conflict avoidance table
                compositeGroup.addGroupToCA(conflictAvoidance, maxDepth);
                allGroups.AddFirst(compositeGroup);

                this.expanded += compositeGroup.expanded;
                this.generated += compositeGroup.generated;

                if (compositeGroup.allAgentsState.Length > this.maxGroup)
                    this.maxGroup = compositeGroup.allAgentsState.Length;

                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost;
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
            // Solve the single agent problems independently
            LinkedListNode<AgentsGroup> agentGroupNode = this.allGroups.First;
            maxDepth = 0;

            while (agentGroupNode != null)
            {
                AgentsGroup group = agentGroupNode.Value;
                group.instance.parameters[CONFLICT_AVOIDANCE] = conflictAvoidance;
                group.Solve(runner);
                if (group.solutionCost > maxDepth)
                    maxDepth = group.solutionCost;
                // Add group to conflict avoidance table
                group.addGroupToCA(conflictAvoidance, maxDepth);

                this.expanded += group.expanded;
                this.generated += group.generated;

                agentGroupNode = agentGroupNode.Next;
            }

            //bool solved = this.SimpleIndependenceDetection(runner);
             bool solved = this.IndependenceDetection(runner);
            // Record found solution
            if (solved == true)
            {
                // Store solution details
                this.totalCost = 0;
                foreach (AgentsGroup group in this.allGroups)
                {
                    this.totalCost += group.solutionCost;
                }

                this.foundPlan = this.CalculateJointPlan();
            }
            else
            {
                this.foundPlan = null;
            }
            Console.WriteLine();
            Console.WriteLine(this.allGroups.Count + " - Independent Groups");
            Console.WriteLine(this.maxGroup + " - Size Of Largest ID Group");
            Console.WriteLine();
            return solved;
        }

        /// <summary>
        /// Print the solution to the console. This is a method that is used for debug.
        /// </summary>
        /// <param name="end"></param>
        private void printSolution(WorldState end)
        {
            int step = 0;
            Console.WriteLine("solution back to front");
            Console.WriteLine("------------------------------\n");
            while (end != null)
            {
                Console.WriteLine("step " + step);
                Console.WriteLine(end.ToString());
                step++;
                end = end.prevStep;
            }
        }

        private void print()
        {
            Console.WriteLine("Expanded - " + expanded);
            Console.WriteLine("Generated - " + generated);
            Console.WriteLine("Total cost - " + totalCost);
        }

        public int GetExpanded() { return this.expanded; }
        public int GetGenerated() { return this.generated; }
        public int GetSolutionDepth() { return maxSolutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
    }


    /// <summary>
    /// This class represents a group of agents that need to be solved together.
    /// </summary>
    class AgentsGroup
    {
        public AgentState[] allAgentsState;
        public WorldState solution;
        public int solutionCost;
        public ProblemInstance instance;
        public int expanded;
        public int generated;
        public int depthOfSolution;
        public long timeTillLastNode;

        private ISolver solver;
        private Plan plan;
        
        public AgentsGroup(ProblemInstance instance, AgentState[] allAgentsState,ISolver solver)
        {
            this.allAgentsState = allAgentsState;
            this.instance = instance.Subproblem(allAgentsState); 
            this.solver = solver;
        }

        /// <summary>
        /// Solve the group of agents together.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns>true if optimal solution for the group of agents were found, false otherwise</returns>
        public bool Solve(Run runner)
        {
            this.solver.Setup(this.instance, runner);
            bool solved = this.solver.Solve();
            this.solutionCost = this.solver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.plan = this.solver.GetPlan();
            this.expanded = solver.GetExpanded();
            this.generated = solver.GetGenerated();
            this.depthOfSolution = solver.GetSolutionDepth();
            this.timeTillLastNode = solver.GetMemoryUsed();

            // Clear memory
            this.solver.Clear();
            return true;
        }

        /// <summary>
        /// Returns the plan for the group of agents. This is a collection of points for every time step until all the agents reach their goal.
        /// </summary>
        /// <returns>A collection of points for every time step until all the agents reach their goal.</returns>
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
            int i;
            for (i = 0; i < allAgentsState.Length; i++)
            {
                joinedAgentStates[i] = allAgentsState[i];
            }
            for (int j = 0; j < other.allAgentsState.Length; j++)
            {
                joinedAgentStates[i + j] = other.allAgentsState[j];
            }

            return new AgentsGroup(this.instance, joinedAgentStates, this.solver);
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
        /// This is used in the IndependenceDetection() method.
        /// </summary>
        /// <param name="plan"></param>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ReplanUnderConstraints(Plan plan, Run runner)
        {
            bool success;
            int oldCost = this.solutionCost;
            WorldState oldSolution = this.solution;
            Plan oldPlan=this.plan;
            HashSet<TimedMove> reserved = new HashSet<TimedMove>();
            plan.addPlanToHashSet(reserved, Math.Max(plan.GetSize(), this.plan.GetSize())-1);

            this.instance.parameters[Trevor.ILLEGAL_MOVES_KEY] = reserved;
            this.instance.parameters[Trevor.MAXIMUM_COST_KEY] = solutionCost;
            success = this.Solve(runner);
            this.instance.parameters.Remove(Trevor.ILLEGAL_MOVES_KEY);
            this.instance.parameters.Remove(Trevor.MAXIMUM_COST_KEY);
            if (success == false)
            {
                this.solutionCost = oldCost;
                this.solution=oldSolution;
                this.plan = oldPlan;
            }
            return success;
        }

        public void addGroupToCA(HashSet<TimedMove> CA, int maxTimeStep)
        {
            if (this.plan == null)
                return;
            
            for (int i = 1; i <= maxTimeStep*2; i++)
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    CA.Add(new TimedMove(move, i));
                }
            }
        }

        public void removeGroupFromCA(HashSet<TimedMove> CA)
        {
            int i = 1;
            bool stop = false;
            while (stop == false)
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    var S = new TimedMove(move, i);
                    if (CA.Contains(S))
                        CA.Remove(new TimedMove(move, i));
                    else
                    {
                        stop = true;
                        break;
                    }
                }
                i++;
            }
        }

    }
}
