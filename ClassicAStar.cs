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
        public OpenList openList;
        public Dictionary<WorldState, WorldState> closedList;
        public int solutionDepth;
        protected int expanded;
        protected int generated;
        protected int reopened;
        protected int bpmxBoosts;
        protected int reopenedWithOldH;
        protected int noReopenHUpdates;
        protected int maxExpansionDelay;
        protected int closedListHits;
        protected int accExpanded;
        protected int accGenerated;
        protected int accReopened;
        protected int accBpmxBoosts;
        protected int accReopenedWithOldH;
        protected int accNoReopenHUpdates;
        protected int accMaxExpansionDelay;
        protected int accClosedListHits;
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
        /// For CBS/A*
        /// </summary>
        protected int minDepth;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public ClassicAStar(HeuristicCalculator heuristic = null)
        {
            this.closedList = new Dictionary<WorldState, WorldState>();
            this.openList = new OpenList(this);
            this.heuristic = heuristic;
            
            this.queryConstraint = new CbsConstraint();
            this.queryConstraint.queryInstance = true;
        }

        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;
            WorldState root = this.CreateSearchRoot(minDepth);
            root.h = (int)this.heuristic.h(root); // g was already set in the constructor
            this.openList.Add(root);
            this.closedList.Add(root, root);
            this.ClearPrivateStatistics();
            this.generated++; // The root
            this.totalCost = 0;
            this.solutionDepth = -1;
            this.numOfAgents = problemInstance.m_vAgents.Length;

            // Store parameters used by Trevor's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY) &&
                ((HashSet<TimedMove>)problemInstance.parameters[Trevor.ILLEGAL_MOVES_KEY]).Count != 0)
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS) &&
                ((HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS]).Count != 0)
                 this.constraintList = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS];
 
             if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS) &&
                 ((List<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS]).Count != 0)
             {
                 List<CbsConstraint> musts = (List<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
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

        /// <summary>
        /// Factory method. Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot(int minDepth = -1)
        {
            return new WorldState(this.instance.m_vAgents, minDepth);
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
            string ret = this.GetName() + "/" + this.heuristic;
            if (this.openList.GetType() != typeof(OpenList))
                ret += " with " + this.openList;
            return ret;
        }

        public int GetSolutionCost() { return this.totalCost; }

        protected void ClearPrivateStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
            this.reopened = 0;
            this.bpmxBoosts = 0;
            this.reopenedWithOldH = 0;
            this.noReopenHUpdates = 0;
            this.closedListHits = 0;
            this.maxExpansionDelay = -1;
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " BPMX boosts");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reopened With Old H");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " H Updated From Other Area");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max expansion delay");
            output.Write(Run.RESULTS_DELIMITER);
            
            this.heuristic.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes: {0}", this.GetExpanded());
            Console.WriteLine("Total Generated Nodes: {0}", this.GetGenerated());
            Console.WriteLine("Total Reopened Nodes: {0}", this.reopened);
            Console.WriteLine("Num BPMX boosts: {0}", this.bpmxBoosts);
            Console.WriteLine("Closed list hits: {0}", this.closedListHits);
            Console.WriteLine("Reopened Nodes With Old H: {0}", this.reopenedWithOldH);
            Console.WriteLine("No Reopen H Updates: {0}", this.noReopenHUpdates);
            Console.WriteLine("Max expansion delay: {0}", this.maxExpansionDelay);

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write(this.reopened + Run.RESULTS_DELIMITER);
            output.Write(this.bpmxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.reopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.noReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.maxExpansionDelay + Run.RESULTS_DELIMITER);

            this.heuristic.OutputStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 8 + this.heuristic.NumStatsColumns + this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            this.ClearPrivateStatistics();
            this.heuristic.ClearStatistics();
            this.openList.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
            this.accReopened = 0;
            this.accBpmxBoosts = 0;
            this.accClosedListHits = 0;
            this.accReopenedWithOldH = 0;
            this.accNoReopenHUpdates = 0;
            this.accMaxExpansionDelay = 0;

            this.heuristic.ClearAccumulatedStatistics();

            this.openList.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accReopened += this.reopened;
            this.accBpmxBoosts += this.bpmxBoosts;
            this.accClosedListHits += this.closedListHits;
            this.accReopenedWithOldH += this.reopenedWithOldH;
            this.accNoReopenHUpdates += this.noReopenHUpdates;
            this.accMaxExpansionDelay = Math.Max(this.accMaxExpansionDelay, this.maxExpansionDelay);

            this.heuristic.AccumulateStatistics();

            this.openList.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGenerated);
            Console.WriteLine("{0} Accumulated Reopened Nodes (Low-Level): {1}", this, this.accReopened);
            Console.WriteLine("{0} Accumulated BPMX boosts (Low-Level): {1}", this, this.accBpmxBoosts);
            Console.WriteLine("{0} Accumulated Closed list hits (Low-Level): {1}", this, this.accClosedListHits);
            Console.WriteLine("{0} Accumulated Reopened Nodes With Old H (Low-Level): {1}", this, this.accReopenedWithOldH);
            Console.WriteLine("{0} Accumulated No Reopen H Updates (Low-Level): {1}", this, this.accNoReopenHUpdates);
            Console.WriteLine("{0} Accumulated Max expansion delay (Low-Level): {1}", this, this.accMaxExpansionDelay);

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accReopened + Run.RESULTS_DELIMITER);
            output.Write(this.accBpmxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.accReopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.accNoReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.accMaxExpansionDelay + Run.RESULTS_DELIMITER);

            this.heuristic.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public virtual bool Solve()
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
                // FIXME: Better to use the whole rich comparison. Save the last node and use compareTo.
                //        The only time a node is allowed to be smaller than the last one is if it's the goal.
                lastF = currentNode.g + currentNode.h;
                lastNode = currentNode;

                // Calculate expansion delay
                int expansionDelay = this.expanded - currentNode.expandedCountWhenGenerated - 1; // -1 to make the delay zero when a node is expanded immediately after being generated.
                maxExpansionDelay = Math.Max(maxExpansionDelay, expansionDelay);

                // Check if node is the goal, or knows how to get to it
                if (currentNode.GoalTest())
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
                parent.h = maxChildH - deltaGOfChildWithMaxH; // Good for partial expansion algs that reinsert the expanded node into the open list.
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
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    break;

                // Try all legal moves of the agents
                foreach (TimedMove agentLocation in currentNode.allAgentsState[agentIndex].lastMove.GetNextMoves())
                {
                    if (IsValid(agentLocation, currentNode.currentMoves, currentNode.makespan + 1,
                        instance.m_vAgents[agentIndex].agent.agentNum) == false)
                        continue;

                    childNode = CreateSearchNode(currentNode);
                    childNode.allAgentsState[agentIndex].MoveTo(agentLocation);

                    childNode.prevStep = currentNode.prevStep; // Skip temporary node objects used during expansion process.
                    if (agentIndex == 0)
                        childNode.prevStep = currentNode;

                    if (agentIndex < currentNode.allAgentsState.Length - 1) // More agents need to move
                        childNode.currentMoves.Add(agentLocation);
                    else // Moved the last agent
                        childNode.currentMoves.Clear(); // To reduce memory load and lookup times, even though it's correct to leave the old moves since they're timed.

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
        /// Just an optimization
        /// </summary>
        private CbsConstraint queryConstraint;

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
                if (possibleMove.IsColliding(illegalMoves))
                    return false;
            } // FIXME: Also checked in IsValid later.

            if (this.constraintList != null)
            {
                queryConstraint.Init(agentNum, possibleMove);

                if (this.constraintList.Contains(queryConstraint))
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
            return (possibleMove.IsColliding(currentMoves) == false);
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
            return this.GetPlan().GetSinglePlans();
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

        public void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.Setup(problemInstance, -1, runner);
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
                    // We're counting conflicts along the entire path, so the parent's conflicts count
                    // is added to the child's
                }

                if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                {
                    currentNode.cbsInternalConflictsCount = currentNode.prevStep.cbsInternalConflictsCount;
                    currentNode.cbsInternalConflictsCount += currentNode.ConflictsCount(
                        ((HashSet_U<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                    // We're counting conflicts along the entire path, so the parent's conflicts count
                    // is added to the child's
                }

                // If in closed list - only reopen if F is lower
                if (this.closedList.ContainsKey(currentNode) == true)
                {
                    ++this.closedListHits;
                    WorldState inClosedList = this.closedList[currentNode];
                    // Notice the agents may have gotten to their location from a different direction in this node.

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
                        // Reinsert old node with new higher F, if it's still in the closed list.
                        // This pushes it further back in the open list so it certainly won't be smaller than the currently expanded node, so monotonicity is maintained.
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
                    currentNode.expandedCountWhenGenerated = this.expanded;
                    return true;
                }

                // What if in open list? This implementation immediately puts _generated_ nodes in the closed list,
                // so it only needs to check it and not the open list.
                // That actually makes a lot of sense: membership tests in heaps are expensive, and in hashtables are cheap.
                // This way we only need to _search_ the open list if we encounter a node that was already visited.
            }
            return false;
        }

        public int GetExpanded() { return expanded; }
        public int GetGenerated() { return generated; }
        public int GetAccumulatedExpanded() { return accExpanded; }
        public int GetAccumulatedGenerated() { return accGenerated; }
        public int GetMaxGroupSize() { return numOfAgents; }

        public virtual float GetEffectiveBranchingFactor()
        {
            return ((float)this.GetGenerated() - 1) / this.GetExpanded();
        }
    }
}

