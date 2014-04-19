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
        public int totalCost;
        public int numOfAgents;
        protected int maxCost;
        /// <summary>
        /// For derived algorithms that use partial expansion
        /// </summary>
        protected int expandedFullStates;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<CbsConstraint> constraintList;
        protected Run runner;
        public WorldState goal; // This will contain the goal node if such was found
        /// <summary>
        /// For CBS-A*
        /// </summary>
        protected int minDepth;
        protected int internalConflictCount;
        protected int externalConflictCount;
        /// <summary>
        /// For CBS-IDA* - not used
        /// </summary>
        protected List<CbsConstraint>[] mustConstraints;

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
            root.h = (int)this.heuristic.h(root); // g was already set to 0 in the constructor
            this.openList.Add(root);
            this.closedList.Add(root, root);
            this.expanded = 0;
            this.generated = 0;
            this.expandedFullStates = 0;
            this.totalCost = 0;
            this.solutionDepth = 0;
            this.goal = null;
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
        /// Clears the relevant data structures and variables to free memory usage.
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

        public WorldState GetGoal() { return this.goal; }

        public int GetSolutionCost() { return this.totalCost; }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            //output.Write(solutionDepth + Run.RESULTS_DELIMITER);
            // output.Write(expandedFullStates + Run.RESULTS_DELIMITER);
            // output.Write("NA"/*Process.GetCurrentProcess().VirtualMemorySize64*/ + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public bool Solve()
        {
            this.solutionDepth = ((WorldState)openList.Peek()).h;

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

                // Check if node is the goal
                if (currentNode.GoalTest(minDepth))
                {
                    this.totalCost = currentNode.g;
                    this.goal = currentNode;
                    this.solutionDepth = this.totalCost - this.solutionDepth;
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
                currentNode.h = (int)this.heuristic.h(currentNode);
                currentNode.makespan++;
                currentNode.CalculateG();
                ProcessGeneratedNode(currentNode);
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
            CbsConstraint nextStepLocation = null;
            if (this.constraintList != null)
                nextStepLocation = new CbsConstraint();
            WorldState childNode;

            foreach (var currentNode in intermediateNodes)
            {
                // Try all legal moves of the agents
                foreach (TimedMove agentLocation in currentNode.allAgentsState[agentIndex].last_move.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
                {
                    if (IsValid(agentLocation, currentNode.currentMoves) == false)
                        continue;

                    if (this.constraintList != null)
                    {
                        nextStepLocation.init(instance.m_vAgents[agentIndex].agent.agentNum, agentLocation); // Throws nullreference exception when running CBS. Has something to do 
                        
                        if (this.constraintList.Contains(nextStepLocation))
                            continue;
                    }
                    if (this.mustConstraints != null && this.mustConstraints.Length > currentNode.makespan + 1 && // not >=?
                        this.mustConstraints[currentNode.makespan + 1] != null)
                    {
                        if (this.mustConstraints[currentNode.makespan + 1].Any<CbsConstraint>(
                            con => con.violatesMustCond((byte)instance.m_vAgents[agentIndex].agent.agentNum, agentLocation)))
                            continue;
                    }

                    childNode = CreateSearchNode(currentNode);
                    childNode.allAgentsState[agentIndex].move(agentLocation);
                    childNode.prevStep = currentNode.prevStep; // Most node objects are just temporary ones used during expansion process - skip them
                    if (agentIndex == 0)
                    {
                        childNode.prevStep = currentNode;
                    }
                    childNode.currentMoves.Add(agentLocation);

                    GeneratedNodes.Add(childNode);
                }
            }
            intermediateNodes.Clear();
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
        /// This method is here instead of in ProblemInstance to enable unused algorithmic tweaks.
        /// </summary>
        /// <param name="possibleMove">The move to check if possible</param>
        /// <returns>true, if the move is possible.</returns>
        protected bool IsValid(TimedMove possibleMove, HashSet<TimedMove> currentMoves)
        {
            // Check if the proposed move is reserved in the plan of another agent.
            // This is used in Trevor's IndependenceDetection.
            if (this.illegalMoves != null) 
            {
                if (possibleMove.isColliding(illegalMoves))
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
        /// This never returns null, don't lie to me.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return new Plan(this.GetGoal());
        }

        public virtual SinglePlan[] getSinglePlans()
        {
            return SinglePlan.getSinglePlans(this.GetGoal());
        }
        
        /// <summary>
        /// Prints the actual solutions. Mainly for debug purposes
        /// </summary>
        /// <param name="end"></param>
        private void PrintSolution(WorldState end)
        {
            int step = 0;
            Console.WriteLine("solution back to front");
            while (end != null)
            {
                Console.WriteLine("step " + step);
                Console.WriteLine(end.ToString());
                step++;
                end = end.prevStep;
            }
        }

        public int getExpanded() { return this.expanded; }
        public int getGenerated() { return this.generated; }
        public int getSolutionDepth() { return this.solutionDepth; }
        public virtual int getTrueNagativeCount() { return -1; }
        public virtual int getNodesPassedPruningCounter() { return expandedFullStates; }
        public long getMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }

        //CBS SOLVER
        public void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.Setup(problemInstance, runner);
            this.minDepth = minDepth;
            this.internalConflictCount = 0;
            this.externalConflictCount = 0;
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
                this.constraintList = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS];

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTSP))
            {
                List<CbsConstraint> lc = (List<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTSP];
                if (lc != null && lc.Count > 0)
                {
                    mustConstraints = new List<CbsConstraint>[lc[lc.Count - 1].getTimeStep() + 1];
                    foreach (CbsConstraint con in lc)
                    {
                        if (mustConstraints[con.getTimeStep()] == null)
                            mustConstraints[con.getTimeStep()] = new List<CbsConstraint>();
                        mustConstraints[con.getTimeStep()].Add(con);
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
                if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                {
                    currentNode.potentialConflictsCount = currentNode.prevStep.potentialConflictsCount;
                    currentNode.potentialConflictsCount += currentNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDENCE]));
                }

                if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                {
                    currentNode.cbsInternalConflictsCount = currentNode.prevStep.cbsInternalConflictsCount;
                    currentNode.cbsInternalConflictsCount += currentNode.conflictsCount(((HashSet_U<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                }

                //if in closed list
                if (this.closedList.ContainsKey(currentNode) == true)
                {
                    var inClosedList = this.closedList[currentNode];
                    // TODO: Some code dup with CompareTo method of WorldState, not sure if avoidable
                    var g_inClosedList = inClosedList.g;
                    var potentialConflictsCount_inClosedList = inClosedList.potentialConflictsCount;
                    var cbsInternalConflictsCount_inClosedList = inClosedList.cbsInternalConflictsCount;

                    // if g is smaller or
                    //    g is equal but current node has fewer potential conflicts or
                    //                   current node has same number of potential conflicts but current node has fewer CBS internal conflicts than remove the old world state
                    if (g_inClosedList > currentNode.g ||
                        (g_inClosedList == currentNode.g && (potentialConflictsCount_inClosedList > currentNode.potentialConflictsCount ||
                                                                (potentialConflictsCount_inClosedList == currentNode.potentialConflictsCount && cbsInternalConflictsCount_inClosedList > currentNode.cbsInternalConflictsCount))))
                    // Alternative view:
                    // if g is smaller than remove the old world state
                    // if g is equal but current node has fewer potential conflicts than remove the old world state
                    // if g is equal and current node has same number of potential conflicts but current node has fewer CBS internal conflicts than remove the old world state
                    //if (g_inClosedList > currentNode.g || 
                    //    (g_inClosedList == currentNode.g && potentialConflictsCount_inClosedList > currentNode.potentialConflictsCount) ||
                    //    (g_inClosedList == currentNode.g && potentialConflictsCount_inClosedList == currentNode.potentialConflictsCount && cbsInternalConflictsCount_inClosedList > currentNode.cbsInternalConflictsCount))
                    {
                        closedList.Remove(inClosedList);
                        openList.Remove(inClosedList);
                        // Items are searched for in the heap using their binaryHeapIndex, which is only intialized when they're put into it,
                        // and not their hash or their Equals or CompareTo methods, so it's important to call Remove with inClosedList,
                        // which might be in the heap, and not currentNode, which may be Equal to it, but was never in the heap so it
                        // doesn't have a binaryHeapIndex initialized.
                    }
                }

                if (this.closedList.ContainsKey(currentNode) == false)
                {
                    this.closedList.Add(currentNode, currentNode);
                    this.generated++;

                    this.openList.Add(currentNode);
                    return true;
                }

                // What if in open list?? It seems this impl immediately puts _generated_ nodes in the closed list,
                // so it only needs to check it and not the open list.
                // That actually makes a lot of sense: membership tests in heaps are expensive, and in hashtables are cheap.
                // This way we only need to _search_ the open list if we encounter a node that was already visited.
            }
            return false;
        }

        public int getHighLevelExpanded() { return 0; }
        public int getHighLevelGenerated() { return 0; }
        public int getLowLevelExpanded() { return expanded; }
        public int getLowLevelGenerated() { return generated; }
        public int getMaxGroupSize() { return numOfAgents; }
    }
}

