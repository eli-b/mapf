using System;
using System.Collections.Generic;
using System.Collections;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    class Trevor : ISolver
    {
        // The key of the illegal moves table in the ProblemInstance (used in IndependentDetection())
        public static string ILLEGAL_MOVES_KEY = "reserved";
        // The key of the maximal solution cost of the agent group in the ProblemInstance (used in IndependentDetection())
        public static string MAXIMUM_COST_KEY = "cost";
        // The key of the conflict avoidence table
        public static string CONFLICT_AVOIDENCE = "ConflictAvoidence";

        protected LinkedList<AgentsGroup> allGroups;
        protected ProblemInstance instance;
         int expandedHL;
         int generatedHL;
         int expandedLL;
         int generatedLL;
        public int totalCost;

        /// <summary>
        /// The complete plan for all the agents that was found.
        /// </summary>
        public Plan foundPlan;
        
        public int maxGroup;
        public int minGroup;
        public ISolver groupSolver;
        private IList<Conflict> allConflicts;
        private int maxSolutionDepth;
        private HashSet<TimedMove> conflictAvoidence;
        private int maxDepth;
        private int passed;
        private string name;

        public Trevor() : this(new AStarWithOD())
        {}

        public Trevor(ISolver groupSolver)
        {
            this.groupSolver = groupSolver;
            this.allConflicts = new List<Conflict>();
            this.allGroups = new LinkedList<AgentsGroup>();
            this.name = groupSolver.GetName() + "+ID";
        }

        public void Clear()
        {
            this.allGroups.Clear();
            this.groupSolver.Clear();
            this.allConflicts.Clear();
            this.conflictAvoidence.Clear();
        }

        public void Setup(ProblemInstance instance)
        {
            this.instance = instance;
            this.expandedHL = 0;
            this.generatedHL = 0;
            this.totalCost = 0;
            this.maxGroup = 1;
            this.passed = 0;
            this.minGroup = instance.m_vAgents.Length;
            this.conflictAvoidence = new HashSet<TimedMove>();
            // Initialize the agent group collection with a group for every agent
            foreach (AgentState agentStartState in instance.m_vAgents)
                this.allGroups.AddFirst(new AgentsGroup(this.instance,  new AgentState[1] { agentStartState }, this.groupSolver));
        }

        public virtual String GetName() { return name; }

        public WorldState GetGoal() { throw new NotSupportedException("Standley's A* might find a goal without having an explicit goal state, due to independance checks"); }


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

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)
        {
            output.Write(this.expandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedHL + Run.RESULTS_DELIMITER);

            // Output the maximum group size
            this.maxGroup = 0;
            this.maxSolutionDepth = 0;
            this.minGroup = this.instance.m_vAgents.Length;

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

            output.Write(this.maxGroup + Run.RESULTS_DELIMITER);
            output.Write(this.minGroup + Run.RESULTS_DELIMITER);
            output.Write(this.maxSolutionDepth + Run.RESULTS_DELIMITER);
            output.Write(this.passed + Run.RESULTS_DELIMITER);
            output.Write(/*Process.GetCurrentProcess().VirtualMemorySize64*/"NA" + Run.RESULTS_DELIMITER);
        }

        public int getMaxGroupSize()
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
            int planSize=-1;
            
            // Find the longest plan among all the groups
            foreach (AgentsGroup group in this.allGroups)
            {
                planSize = group.GetPlan().GetSize();
                if(planSize>maxPlanSize)
                    maxPlanSize=planSize;
            }

            // Check in every time step that the plans do not collide
            LinkedListNode<AgentsGroup> group1Iterator;
            LinkedListNode<AgentsGroup> group2Iterator;
            Plan group1Plan;
            Plan group2Plan;
            for(int time=1;time<maxPlanSize;time++)
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
        /// Search for an optimal solution using the Simple Independance Detection algorithm in Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool SimpleIndependanceDetection(Run runner)
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
        /// Search for an optimal solution using the Simple Independance Detection algorithm in Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool IndependanceDetection(Run runner)
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
                if(this.allConflicts.Contains(conflict)==false)
                {
                    // Add to all conflicts to prevent trying to replan this conflict again
                    this.allConflicts.Add(conflict);

                    // Add plan of group2 to illegal moves table and replan group1 with equal cost
                    if (conflict.timeOfConflict < conflict.group1.GetPlan().GetSize())
                    {
                        conflict.group1.removeGroupFromCA(conflictAvoidence);
                        if (conflict.group1.ReplanUnderConstraints(conflict.group2.GetPlan(), runner) == true)
                        {
                            conflict.group1.addGroupToCA(conflictAvoidence, maxDepth);
                            continue;
                        }
                        conflict.group1.addGroupToCA(conflictAvoidence, maxDepth);
                    }
                    // Add plan of group1 to illegal moves table and replan group2 with equal cost
                    if (conflict.timeOfConflict < conflict.group2.GetPlan().GetSize() - 1)
                    {
                        conflict.group2.removeGroupFromCA(conflictAvoidence);
                        if (conflict.group2.ReplanUnderConstraints(conflict.group1.GetPlan(), runner) == true)
                        {
                            conflict.group2.addGroupToCA(conflictAvoidence, maxDepth);
                            continue;
                        }
                        conflict.group2.addGroupToCA(conflictAvoidence, maxDepth);
                    }
                }

                // Groups are conflicting - need to join them to a single group
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                //remove both groups from avoidence table
                conflict.group1.removeGroupFromCA(conflictAvoidence);
                conflict.group2.removeGroupFromCA(conflictAvoidence);

                compositeGroup = this.JoinGroups(conflict);

                compositeGroup.instance.parameters[CONFLICT_AVOIDENCE] = conflictAvoidence;

                // Solve composite group with A*
                solved = compositeGroup.Solve(runner);

                if (compositeGroup.solutionCost > maxDepth)
                    maxDepth = compositeGroup.solutionCost;

                //add group to conflict avoidence table
                compositeGroup.addGroupToCA(conflictAvoidence, maxDepth);
                allGroups.AddFirst(compositeGroup);

                this.expandedHL += compositeGroup.expandedHL;
                this.generatedHL += compositeGroup.generatedHL;
                this.expandedLL += compositeGroup.expandedLL;
                this.generatedLL += compositeGroup.generatedLL;
                this.passed += compositeGroup.passed;

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
        /// <param name="runner"></param>
        /// <returns>true if optimal solution has been found</returns>
        public bool Solve(Run runner)
        {
            // Solve the single agent problems independantly
            LinkedListNode<AgentsGroup> agentGroupNode = this.allGroups.First;
            maxDepth=0;

            while (agentGroupNode != null)
            {
                agentGroupNode.Value.instance.parameters[CONFLICT_AVOIDENCE] = conflictAvoidence;
                agentGroupNode.Value.Solve(runner);
                if (agentGroupNode.Value.solutionCost > maxDepth)
                    maxDepth = agentGroupNode.Value.solutionCost;
                //add group to conflict avoidence table
                agentGroupNode.Value.addGroupToCA(conflictAvoidence, maxDepth);

                this.expandedHL += agentGroupNode.Value.expandedHL;
                this.generatedHL += agentGroupNode.Value.generatedHL;
                this.expandedLL += agentGroupNode.Value.expandedLL;
                this.generatedLL += agentGroupNode.Value.generatedLL;

                this.passed += agentGroupNode.Value.passed;

                agentGroupNode = agentGroupNode.Next;
            }

            //bool solved = this.SimpleIndependanceDetection(runner);
             bool solved = this.IndependanceDetection(runner);
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
            Console.WriteLine("Expanded - " + expandedHL);
            Console.WriteLine("Generated - " + generatedHL);
            Console.WriteLine("Total cost - " + totalCost);
        }
        public int getHighLevelExpanded() { return this.expandedHL; }
        public int getHighLevelGenerated() { return this.generatedHL; }
        public int getLowLevelExpanded() { return this.expandedLL; }
        public int getLowLevelGenerated() { return this.generatedLL; }
        public int getSolutionDepth() { return maxSolutionDepth; }
        public int getNodesPassedPruningCounter() { return passed; }
        public long getMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
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
        public int expandedHL;
        public int generatedHL;
        public int expandedLL;
        public int generatedLL;
        public int depthOfSolution;
        public int passed;
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
            this.solver.Setup(this.instance);
            bool solved = this.solver.Solve(runner);
            this.solutionCost = this.solver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.solutionCost = this.solver.GetSolutionCost();
            this.plan = this.solver.GetPlan();
            this.expandedHL = solver.getHighLevelExpanded();
            this.generatedHL = solver.getHighLevelGenerated();
            this.expandedLL = solver.getLowLevelExpanded();
            this.generatedLL = solver.getLowLevelGenerated();
            this.depthOfSolution = solver.getSolutionDepth();
            this.passed = solver.getNodesPassedPruningCounter();
            this.timeTillLastNode = solver.getMemoryUsed();

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
            for ( i = 0; i < allAgentsState.Length; i++)
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

        public override bool Equals(object obj)
        {
            AgentsGroup other = (AgentsGroup)obj;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if(allAgentsState[i].Equals(other.allAgentsState[i]) == false)
                    return false;
            }
            return true;
        }


        /// <summary>
        /// Tries to find a plan for this group, that will not conflict with the given plan,
        /// and stil have the same solution cost as the current solution cost.
        /// This is used in the IndependentDetection() method.
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
            LinkedList<Move> step;
            for (int i = 1; i <= maxTimeStep*2; i++)
            {
                step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    CA.Add(new TimedMove(move, i));
                }
            }
        }

        public void removeGroupFromCA(HashSet<TimedMove> CA)
        {
            LinkedList<Move> step;
            TimedMove S;
            int i = 1;
            bool stop = false;
            while (stop == false)
            {
                step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    S = new TimedMove(move, i);
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
