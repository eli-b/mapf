using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{   
    /// <summary>
    /// This is an implementation of the classic A* algorithm for the MAPF problem.
    /// </summary>
    public class ClassicAStar : ICbsSolver 
    {
        protected ProblemInstance instance;
        protected HeuristicCalculator heuristic;
        public BinaryHeap openList;
        public Dictionary<WorldState, WorldState> closedList;
        public int solutionDepth;
        public int expanded;
        public int generated;
        public int reopened;
        public int bpmxBoosts;
        public int reopenedWithOldH;
        public int noReopenHUpdates;
        public int totalCost;
        public int numOfAgents;
        protected int maxCost;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<CbsConstraint> constraintList;
        /// <summary>
        /// For each constrained timestep, a list of must constraints.
        /// </summary>
        protected List<CbsConstraint>[] mustConstraints;
        protected Run runner;
        protected Plan solution;
        /// <summary>
        /// For CBS-A*
        /// </summary>
        protected int minDepth;
        protected int internalConflictCount;
        protected int externalConflictCount;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public ClassicAStar(HeuristicCalculator heuristic = null)
        {
            this.closedList = new Dictionary<WorldState, WorldState>();
            this.openList = new BinaryHeap();
            this.heuristic = heuristic;
        }

        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;
            WorldState root = this.CreateSearchRoot();
            root.h = (int)this.heuristic.h(root); // g was already set in the constructor
            this.openList.Add(root);
            this.closedList.Add(root, root);
            this.expanded = 0;
            this.generated = 1;
            this.reopened = 0;
            this.bpmxBoosts = 0;
            this.reopenedWithOldH = 0;
            this.noReopenHUpdates = 0;
            this.totalCost = 0;
            this.solutionDepth = -1;
            this.numOfAgents = problemInstance.m_vAgents.Length;

            // Store parameters used by Trevor's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;

            this.minDepth = 0;
        }

        /// <summary>
        /// Factory method. Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot()
        {
            return new WorldState(this.instance.m_vAgents);
        }

        /// <summary>
        /// Clears the relevant data structures and variables to free memory.
        /// </summary>
        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.mustConstraints = null;
            this.illegalMoves = null;
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.heuristic = heuristic;
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.heuristic;
        }

        public virtual String GetName() { return "A*"; }

        public override string ToString()
        {
            return this.GetName() + "/" + this.heuristic;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " BPMX boosts (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened With Old H (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " H Updated From Other Area (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            
            this.heuristic.OutputStatisticsHeader(output);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (Low-Level): {0}", this.GetLowLevelExpanded());
            Console.WriteLine("Total Generated Nodes (Low-Level): {0}", this.GetLowLevelGenerated());
            Console.WriteLine("Total Reopened Nodes (Low-Level): {0}", this.reopened);
            Console.WriteLine("Num BPMX boosts (Low-Level): {0}", this.bpmxBoosts);
            Console.WriteLine("Reopened Nodes With Old H (Low-Level): {0}", this.reopenedWithOldH);
            Console.WriteLine("No Reopen H Updates (Low-Level): {0}", this.noReopenHUpdates);

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write(this.reopened + Run.RESULTS_DELIMITER);
            output.Write(this.bpmxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.reopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.noReopenHUpdates + Run.RESULTS_DELIMITER);

            this.heuristic.OutputStatistics(output);
        }

        public int NumStatsColumns
        {
            get
            {
                return 6 + this.heuristic.NumStatsColumns;
            }
        }

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public bool Solve()
        {
            int initialEstimate = ((WorldState)openList.Peek()).h;

            int lastF = -1;
            WorldState lastNode = null;
            bool debug = false;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    return false;
                }

                var currentNode = (WorldState)openList.Remove();

                Debug.Assert(currentNode.g + currentNode.h >= lastF,
                             "A* node with decreasing F: " + (currentNode.g + currentNode.h) + " < " + lastF + ".");
                lastF = currentNode.g + currentNode.h;
                lastNode = currentNode;

                // Check if node is the goal, or knows how to get to it
                if (currentNode.GoalTest(minDepth))
                {
                    this.totalCost = currentNode.GetGoalCost();
                    this.singleCosts = currentNode.GetSingleCosts();
                    this.solution = currentNode.GetPlan();
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.Clear();
                    return true;
                }

                // Expand
                Expand(currentNode);
                expanded++;
            }

            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implemented together with the closed list.
        /// </summary>
        /// <param name="node"></param>
        public virtual void Expand(WorldState node)
        {
            //Debug.Print("Expanding node " + node);
            var intermediateNodes = new List<WorldState>();
            intermediateNodes.Add(node);

            for (int agentIndex = 0; agentIndex < this.instance.m_vAgents.Length ; ++agentIndex)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;

                intermediateNodes = ExpandOneAgent(intermediateNodes, agentIndex);
            }
            var finalGeneratedNodes = intermediateNodes;

            foreach (var currentNode in finalGeneratedNodes)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;
                currentNode.CalculateG();
                currentNode.makespan++;
                currentNode.h = (int)this.heuristic.h(currentNode);
            }

            // BPMX (Felner et al. 2005) stage:
            // Reverse Path-Max
            WorldState parent = node;
            int maxChildH = -1;
            int deltaGOfChildWithMaxH = 0;
            foreach (var child in finalGeneratedNodes)
            {
                if (child.h > maxChildH)
                {
                    maxChildH = child.h;
                    deltaGOfChildWithMaxH = child.g - parent.g;
                }
            }
            if (parent.h < maxChildH - deltaGOfChildWithMaxH)
            {
                parent.h = maxChildH - deltaGOfChildWithMaxH;
                ++bpmxBoosts;
            }
            // Forward Path-Max
            foreach (var child in finalGeneratedNodes)
            {
                int deltaG = child.g - parent.g; // == (parent.g + c(parent, current)) - parent.g == c(parent, current)

                if (child.h < parent.h - deltaG)
                {
                    child.h = parent.h - deltaG;
                    ++bpmxBoosts;
                }
            }
            
            // Enter the generated nodes into the open list
            foreach (var child in finalGeneratedNodes)
            {
                ProcessGeneratedNode(child);
            }
        }
        
        /// <summary>
        /// Expands a single agent in the nodes.
        /// This includes:
        /// - Generating the children
        /// - Inserting them into OPEN
        /// - Insert node into CLOSED
        /// Returns the child nodes
        /// </summary>
        protected virtual List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            var GeneratedNodes = new List<WorldState>();
            WorldState childNode;

            foreach (var currentNode in intermediateNodes)
            {
                // Try all legal moves of the agents
                foreach (TimedMove agentLocation in currentNode.allAgentsState[agentIndex].lastMove.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
                {
                    if (IsValid(agentLocation, currentNode.currentMoves, currentNode.makespan + 1,
                        instance.m_vAgents[agentIndex].agent.agentNum) == false)
                        continue;

                    childNode = CreateSearchNode(currentNode);
                    childNode.allAgentsState[agentIndex].MoveTo(agentLocation);
                    childNode.prevStep = currentNode.prevStep; // Most node objects are just temporary ones used during expansion process - skip them
                    if (agentIndex == 0)
                    {
                        childNode.prevStep = currentNode;
                    }
                    childNode.currentMoves.Add(agentLocation);

                    GeneratedNodes.Add(childNode);
                }
            }
            return GeneratedNodes;
        }

        /// <summary>
        /// Factory method.
        /// </summary>
        /// <param name="from"></param>
        /// <returns></returns>
        protected virtual WorldState CreateSearchNode(WorldState from)
        {
            return new WorldState(from);
        }

        /// <summary>
        /// Check if the move is valid, i.e. not colliding into walls or other agents.
        /// This method is here instead of in ProblemInstance to enable algorithmic tweaks.
        /// </summary>
        /// <param name="possibleMove">The move to check if possible</param>
        /// <returns>true, if the move is possible.</returns>
        protected bool IsValid(TimedMove possibleMove, HashSet<TimedMove> currentMoves, int makespan, int agentNum)
        {
            // Check if the proposed move is reserved in the plan of another agent.
            // This is used in Trevor's IndependenceDetection.
            if (this.illegalMoves != null) 
            {
                if (possibleMove.isColliding(illegalMoves))
                    return false;
            } // FIXME: Also checked in IsValid later.

            if (this.constraintList != null)
            {
                CbsConstraint nextStepLocation = new CbsConstraint(agentNum, possibleMove);

                if (this.constraintList.Contains(nextStepLocation))
                    return false;
            }

            if (this.mustConstraints != null && makespan < this.mustConstraints.Length && // There may be a constraint on the timestep of the generated node
                this.mustConstraints[makespan] != null)
            {
                if (this.mustConstraints[makespan].Any<CbsConstraint>(
                    con => con.ViolatesMustConstraint((byte)agentNum, possibleMove)))
                    return false;
            }

            // If the tile is not free (out of the grid or with an obstacle)
            if (this.instance.IsValid(possibleMove) == false)
                return false;

            // Check against all the agents that have already moved to see if current move collides with their move
            return (possibleMove.isColliding(currentMoves) == false);
        }

        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        public virtual SinglePlan[] GetSinglePlans()
        {
            return this.GetPlan().getSinglePlans();
        }

        protected int[] singleCosts;
        public virtual int[] GetSingleCosts()
        {
            return this.singleCosts;
        }

        public int getExpanded() { return this.expanded; }
        public int getGenerated() { return this.generated; }
        public int GetSolutionDepth() { return this.solutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }

        // CBS SOLVER
        public void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.Setup(problemInstance, runner);
            this.minDepth = minDepth;
            this.internalConflictCount = 0;
            this.externalConflictCount = 0;

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
                this.constraintList = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS];

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS))
            {
                List<CbsConstraint> musts = (List<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                if (musts != null && musts.Count > 0)
                {
                    this.mustConstraints = new List<CbsConstraint>[musts.Max<CbsConstraint>(con => con.GetTimeStep()) + 1]; // To have index MAX, array needs MAX + 1 places.
                    foreach (CbsConstraint con in musts)
                    {
                        int timeStep = con.GetTimeStep();
                        if (this.mustConstraints[timeStep] == null)
                            this.mustConstraints[timeStep] = new List<CbsConstraint>();
                        this.mustConstraints[timeStep].Add(con);
                    }
                }
            }
        }

        /// <summary>
        /// Returns whether the node was inserted into the open list.
        /// </summary>
        /// <param name="currentNode"></param>
        /// <returns></returns>
        protected virtual bool ProcessGeneratedNode(WorldState currentNode)
        {
            if (currentNode.h + currentNode.g <= this.maxCost)
            // Assuming h is an admissable heuristic, no need to generate nodes that won't get us to the goal
            // within the budget
            {
                if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
                {
                    currentNode.potentialConflictsCount = currentNode.prevStep.potentialConflictsCount;
                    currentNode.potentialConflictsCount += currentNode.ConflictsCount(
                        ((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDANCE]));
                }

                if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                {
                    currentNode.cbsInternalConflictsCount = currentNode.prevStep.cbsInternalConflictsCount;
                    currentNode.cbsInternalConflictsCount += currentNode.ConflictsCount(
                        ((HashSet_U<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                }

                // If in closed list - only reopen if F is lower
                if (this.closedList.ContainsKey(currentNode) == true)
                {
                    WorldState inClosedList = this.closedList[currentNode];
                    // Since the nodes are equal, give them both the max of their H
                    bool improvedHOfThisNode = false;
                    bool improvedHOfOldNode = false;
                    if (currentNode.h < inClosedList.h)
                    {
                        currentNode.h = inClosedList.h;
                        improvedHOfThisNode = true;
                    }
                    if (inClosedList.h < currentNode.h)
                    {
                        inClosedList.h = currentNode.h;
                        improvedHOfOldNode = true;
                    }

                    int compareVal = currentNode.CompareTo(inClosedList);
                    if (compareVal == -1) // This node has smaller f, or preferred due to other consideration.
                                          // Since we equalised their h, a smaller f means smaller g.
                    {
                        this.reopened++;
                        this.closedList.Remove(inClosedList);
                        this.openList.Remove(inClosedList);
                        // Items are searched for in the heap using their binaryHeapIndex, which is only initialized when they're put into it,
                        // and not their hash or their Equals or CompareTo methods, so it's important to call Remove with inClosedList,
                        // which might be in the heap, and not currentNode, which may be Equal to it, but was never in the heap so it
                        // doesn't have a binaryHeapIndex initialized.
                        if (improvedHOfThisNode)
                            ++reopenedWithOldH;
                    }
                    else if (improvedHOfOldNode)
                    {
                        // Reinsert old node with new higher F, if it's still in the closed list
                        if (this.openList.Remove(inClosedList)) // Cheap if it isn't there
                        {
                            this.openList.Add(inClosedList);
                            ++noReopenHUpdates;
                        }
                    }
                }

                if (this.closedList.ContainsKey(currentNode) == false)
                {
                    this.closedList.Add(currentNode, currentNode);
                    this.generated++; // Reopned nodes are also recounted here.
                    this.openList.Add(currentNode);
                    return true;
                }

                // What if in open list? This implementation immediately puts _generated_ nodes in the closed list,
                // so it only needs to check it and not the open list.
                // That actually makes a lot of sense: membership tests in heaps are expensive, and in hashtables are cheap.
                // This way we only need to _search_ the open list if we encounter a node that was already visited.
            }
            return false;
        }

        public int GetHighLevelExpanded() { return 0; }
        public int GetHighLevelGenerated() { return 0; }
        public int GetLowLevelExpanded() { return expanded; }
        public int GetLowLevelGenerated() { return generated; }
        public int GetMaxGroupSize() { return numOfAgents; }
    }
}

