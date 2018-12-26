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
    public class ClassicAStar : ICbsSolver, IMStarSolver, IHeuristicSolver<WorldState>
    {
        protected ProblemInstance instance;
        protected IHeuristicCalculator<WorldState> heuristic;
        public OpenList<WorldState> openList;
        public Dictionary<WorldState, WorldState> closedList;
        /// <summary>
        /// How much more expensive the solution was than the heuristic's initial estimate
        /// </summary>
        protected int solutionDepth;
        protected Dictionary<int, int> conflictCounts;
        protected Dictionary<int, List<int>> conflictTimes;
        protected int expanded;
        protected int generated;
        protected int reopened;
        /// <summary>
        /// Note we can boost nodes that aren't eventually counted as generated (already in the closed list, too costly, ...)
        /// </summary>
        protected int bpmxBoosts;
        protected int reopenedWithOldH;
        protected int noReopenHUpdates;
        protected int maxExpansionDelay;
        protected int closedListHits;
        protected int mstarBackprops;
        protected int mstarShuffles;
        protected int accExpanded;
        protected int accGenerated;
        protected int accReopened;
        protected int accBpmxBoosts;
        protected int accReopenedWithOldH;
        protected int accNoReopenHUpdates;
        protected int accMaxExpansionDelay;
        protected int accClosedListHits;
        protected int accMstarBackprops;
        protected int accMstarShuffles;
        public int totalCost;
        public int numOfAgents;
        protected int maxSolutionCost;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<CbsConstraint> constraints;
        /// <summary>
        /// An array of dictionaries that map constrained timesteps to must constraints.
        /// </summary>
        protected Dictionary<int, TimedMove>[] mustConstraints;
        protected bool mstar = false;
        protected bool doMstarShuffle = false;
        //protected Dictionary<WorldState, SinglePlan[]> mstarPlanBasesToTheirPlans;
        protected Dictionary<WorldState, HashSet<CbsConstraint>[]> mstarPlanBasesToTheirConstraints; // Most nodes won't have constraints so I don't want to make their memory footprint needlessly large.
        protected List<CbsConflict> mstarBackPropagationConflictList;
        protected Run runner;
        protected Plan solution;
        //// <summary>
        //// For CBS/A*
        //// </summary>
        //protected int minDepth;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public ClassicAStar(IHeuristicCalculator<WorldState> heuristic = null, bool mStar = false, bool mStarShuffle = false)
        {
            this.closedList = new Dictionary<WorldState, WorldState>();
            this.openList = new OpenList<WorldState>(this);
            this.heuristic = heuristic;
            
            this.queryConstraint = new CbsConstraint();
            this.queryConstraint.queryInstance = true;

            this.mstar = mStar;
            this.doMstarShuffle = mStarShuffle;
        }

        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost)
        {
            this.instance = problemInstance;
            this.runner = runner;
            WorldState root = this.CreateSearchRoot(minDepth, minCost);
            root.h = (int)this.heuristic.h(root); // g was already set in the constructor
            this.openList.Add(root);
            this.closedList.Add(root, root);
            this.ClearPrivateStatistics();
            this.generated++; // The root
            this.totalCost = 0;
            this.singleCosts = null;
            this.solution = null;
            this.singlePlans = null;
            this.conflictCounts = null;
            this.conflictTimes = null;
            this.solutionDepth = -1;
            this.numOfAgents = problemInstance.m_vAgents.Length;

            // Store parameters used by IndependenceDetection's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(IndependenceDetection.MAXIMUM_COST_KEY))
                this.maxSolutionCost = (int)problemInstance.parameters[IndependenceDetection.MAXIMUM_COST_KEY];
            else
                this.maxSolutionCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(IndependenceDetection.ILLEGAL_MOVES_KEY) &&
                ((HashSet<TimedMove>)problemInstance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY]).Count != 0)
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS) &&
                ((HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS]).Count != 0)
                 this.constraints = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS];
 
             if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS) &&
                 ((HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS]).Count != 0)
             {
                 var musts = (HashSet_U<CbsConstraint>)problemInstance.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                 this.mustConstraints = new Dictionary<int, TimedMove>[musts.Max<CbsConstraint>(con => con.GetTimeStep()) + 1]; // To have index MAX, array needs MAX + 1 places.
                 foreach (CbsConstraint con in musts)
                 {
                     int timeStep = con.GetTimeStep();
                     if (this.mustConstraints[timeStep] == null)
                         this.mustConstraints[timeStep] = new Dictionary<int, TimedMove>();
                     this.mustConstraints[timeStep][con.agentNum] = con.move;
                 }
             }

             if (this.mstar)
             {
                 root.backPropagationSet = new HashSet<WorldState>();
                 root.collisionSets = new DisjointSets<int>();

                 this.mstarBackPropagationConflictList = new List<CbsConflict>();
             }
        }

        /// <summary>
        /// Factory method. Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1)
        {
            return new WorldState(this.instance.m_vAgents, minDepth, minCost);
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

        public IHeuristicCalculator<WorldState> GetHeuristic()
        {
            return this.heuristic;
        }

        public virtual String GetName()
        {
            if (this.mstar == false)
                return "A*";
            else if (this.doMstarShuffle == false)
                return "rM*";
            else
                return "rM*+shuffle";
        }

        public override string ToString()
        {
            string ret = $"{this.GetName()}/{this.heuristic}";
            if (this.openList.GetType() != typeof(OpenList<WorldState>))
                ret += $" with {this.openList}";
            return ret;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public Dictionary<int, int> GetExternalConflictCounts()
        {
            return this.conflictCounts;
        }


        public Dictionary<int, List<int>> GetConflictTimes()
        {
            return this.conflictTimes;
        }

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
            this.mstarBackprops = 0;
            this.mstarShuffles = 0;
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
            if (this.mstar)
            {
                output.Write(this.ToString() + " Backpropagations");
                output.Write(Run.RESULTS_DELIMITER);
                output.Write(this.ToString() + " Shuffles");
                output.Write(Run.RESULTS_DELIMITER);
            }

            
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
            if (this.mstar)
            {
                Console.WriteLine("Backpropagations: {0}", this.mstarBackprops);
                Console.WriteLine("Shuffles: {0}", this.mstarShuffles);
            }

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write(this.reopened + Run.RESULTS_DELIMITER);
            output.Write(this.bpmxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.reopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.noReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.maxExpansionDelay + Run.RESULTS_DELIMITER);
            if (this.mstar)
            {
                output.Write(this.mstarBackprops + Run.RESULTS_DELIMITER);
                output.Write(this.mstarShuffles + Run.RESULTS_DELIMITER);
            }

            this.heuristic.OutputStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return (this.mstar? 10 : 8) + this.heuristic.NumStatsColumns + this.openList.NumStatsColumns;
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
            this.accMstarBackprops = 0;
            this.accMstarShuffles = 0;

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
            this.accMstarBackprops += this.mstarBackprops;
            this.accMstarShuffles += this.mstarShuffles;

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
            if (this.mstar)
            {
                Console.WriteLine("{0} Accumulated Backpropagations (Low-Level): {1}", this, this.accMstarBackprops);
                Console.WriteLine("{0} Accumulated Shuffles (Low-Level): {1}", this, this.accMstarShuffles);
            }

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accReopened + Run.RESULTS_DELIMITER);
            output.Write(this.accBpmxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.accReopenedWithOldH + Run.RESULTS_DELIMITER);
            output.Write(this.accNoReopenHUpdates + Run.RESULTS_DELIMITER);
            output.Write(this.accMaxExpansionDelay + Run.RESULTS_DELIMITER);
            if (this.mstar)
            {
                output.Write(this.accMstarBackprops + Run.RESULTS_DELIMITER);
                output.Write(this.accMstarShuffles + Run.RESULTS_DELIMITER);
            }

            this.heuristic.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        public bool debug = false;

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public virtual bool Solve()
        {
            int initialEstimate = openList.Peek().h; // g=0 initially

            int lastF = -1;
            WorldState lastNode = null;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.solutionDepth = openList.Peek().g + openList.Peek().h - initialEstimate; // A minimum estimate, assuming h is admissable
                    this.Clear();
                    return false;
                }

                WorldState currentNode = openList.Remove();

                if (currentNode.f > this.maxSolutionCost)  // A late heuristic application may have increased the node's cost
                {
                    continue;
                    // This will exhaust the open list, assuming Fs of nodes chosen for expansions
                    // are monotonically increasing.
                }

                if (debug)
                {
                    Debug.WriteLine("");
                    Debug.WriteLine($"Expanding node: {currentNode}");
                }

                //if (this.instance.m_vAgents.Length > 2)
                //{
                //    int a = 3;
                //    int b = (a + 2) * 2;

                //    int x1, x2, x3, y1, y2, y3, x4, y4;
                //    x1 = 5; y1 = 3;
                //    x2 = 5; y2 = 4;
                //    x3 = 6; y3 = 2;
                //    x4 = 5; y4 = 7;
                //    if (currentNode.allAgentsState[0].lastMove.x == x1 &&
                //        currentNode.allAgentsState[0].lastMove.y == y1 &&
                //        currentNode.allAgentsState[1].lastMove.x == x2 &&
                //        currentNode.allAgentsState[1].lastMove.y == y2 &&
                //        currentNode.allAgentsState[2].lastMove.x == x3 &&
                //        currentNode.allAgentsState[2].lastMove.y == y3 &&
                //        currentNode.allAgentsState[3].lastMove.x == x4 &&
                //        currentNode.allAgentsState[3].lastMove.y == y4)
                //    {
                //        int c = 3;
                //        int d = 3 * c;
                //    }
                //}

                if (this.mstar == false && // Backpropagation can cause the root to be re-expanded after many more expensive nodes were expanded.
                    (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS || this.GetType() != typeof(AStarWithOD)) &&  // A*+OD on makespan can have final nodes with lower F than intermediate nodes because the move cost is
                                                                                                                                 // attributed to the first agent and its gains may show up in a later agent's h
                    (this.openList is DynamicLazyOpenList<WorldState>) == false && // When the open list has just one node, application of the heuristic is skipped altogether.
                    (this.openList is DynamicRationalLazyOpenList) == false        // This can cause decreasing F values.
                    )
                    Debug.Assert(currentNode.f >= lastF,
                                 $"A* node with decreasing F: {currentNode.f} < {lastF}.");
                else
                {
                    // TODO: Record the max F. Assert that the goal's F isn't smaller than it.
                }
                lastF = currentNode.f;
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
                    this.singlePlans = currentNode.GetSinglePlans();
                    this.conflictCounts = currentNode.cbsInternalConflicts; // TODO: Technically, could be IndependenceDetection's count. Merge them.
                    this.conflictTimes = currentNode.conflictTimes;
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.Clear();
                    return true;
                }

                // Expand
                if (this.mstar == false)
                {
                    if (this.debug)
                        Debug.Print("");
                    //Expand(currentNode);
                }
                else
                {
                    this.mstarBackPropagationConflictList.Clear();
                    // TODO: Need to clear individual agent planned moves, in case this node was reopened with an updated collision set.
                    if (this.debug)
                    {
                        Console.Write("with collision sets: {0}", currentNode.collisionSets);
                    }
                    var sets = currentNode.collisionSets.GetSets();
                    foreach (var set in sets)
                    {
                        if (set.Count != 0)
                        {
                        }
                        // Give each agent in the set a planned move
                    }
                }
                Expand(currentNode);
                expanded++;

                if (this.mstar && this.mstarBackPropagationConflictList.Count != 0)
                {
                    ++this.mstarBackprops;
                    foreach (var conflict in this.mstarBackPropagationConflictList)
                    {
                        //currentNode.individualMStarPlans[conflict.agentAIndex] = null;
                        //currentNode.individualMStarPlans[conflict.agentBIndex] = null;
                        this.RMStarCollisionBackPropagation(conflict, currentNode);
                    }
                    this.mstarBackPropagationConflictList.Clear();
                }
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
            var intermediateNodes = new List<WorldState>();
            intermediateNodes.Add(node);

            for (int agentIndex = 0; agentIndex < this.instance.m_vAgents.Length ; ++agentIndex)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;

                //Debug.Print("Moving {0} agent", agentIndex);

                intermediateNodes = ExpandOneAgent(intermediateNodes, agentIndex);
            }
            var finalGeneratedNodes = intermediateNodes;

            foreach (var currentNode in finalGeneratedNodes)
            {
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;
                currentNode.makespan++;
                currentNode.CalculateG();
                currentNode.h = (int)this.heuristic.h(currentNode);

                // Boost h based on minGoalCost
                if (currentNode.g < currentNode.minGoalCost)
                {
                    if (currentNode.h == 0)  // Agent is at the goal, only too early
                        currentNode.h = 2; // Otherwise waiting at goal would expand to waiting at the goal for the same too low cost,
                                           // which would expand to waiting at the goal, etc.
                                           // +2 because you need a step out of the goal and another step into it.
                    currentNode.h = Math.Max(currentNode.h, currentNode.minGoalCost - currentNode.g);
                    // TODO: Add a statistic for when the H was increased thanks to the minGoalCost
                }
            }

            // Path-Max stage:
            if ((this.heuristic.GetType() != typeof(SumIndividualCosts) &&
                (this.heuristic.GetType() != typeof(MaxIndividualCosts))) || (this.openList.GetType() != typeof(OpenList<WorldState>)))  // double CHECK!
            // otherwise if we just use SIC and no lazy heuristic in addition to it,
            // then our heuristic is consistent and Path-Max isn't necessary
            {
                // Reverse Path-Max (operators are invertible) - BPMX (Felner et al. 2005)
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
                    int newParentH = maxChildH - deltaGOfChildWithMaxH;
                    parent.hBonus += newParentH - parent.h;
                    parent.h = newParentH; // Also good for partial expansion algs that reinsert the expanded node into the open list
                                           // (in addition to aiding the forward Path-Max).
                    ++bpmxBoosts;
                    // FIXME: Code duplication with Forward Path-Max
                }
                // Forward Path-Max
                foreach (var child in finalGeneratedNodes)
                {
                    int deltaG = child.g - parent.g; // == (parent.g + c(parent, current)) - parent.g == c(parent, current)
                    if (child.h < parent.h - deltaG)
                    {
                        int newChildH = parent.h - deltaG;
                        child.hBonus += newChildH - child.h;
                        child.h = newChildH;
                        ++bpmxBoosts;
                    }
                }
            }

            // Enter the generated nodes into the open list
            foreach (var child in finalGeneratedNodes)
            {
                ProcessGeneratedNode(child);
            }

            if (this.debug)
                Debug.Print("\n");
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
                    WorldState origNode = agentIndex == 0? currentNode : currentNode.prevStep;
                    bool moveIsValid = true;
                    //moveIsValid = this.IsValid(agentLocation, currentNode.currentMoves, currentNode.makespan + 1, agentIndex, origNode, currentNode);
                    //if (moveIsValid == false)
                    //    continue;

                    //----------------Begin pasting isValid method
                    TimedMove possibleMove = agentLocation;
                    IReadOnlyDictionary<TimedMove, int> currentMoves = currentNode.currentMoves;
                    int makespan = currentNode.makespan + 1;
                    WorldState fromNode = origNode;
                    WorldState intermediateMode = currentNode;
                    int agentNum = fromNode.allAgentsState[agentIndex].agent.agentNum;

                    // Check if the proposed move is reserved in the plan of another agent.
                    // This is used in IndependenceDetection's ImprovedID.
                    if (this.illegalMoves != null)
                    {
                        if (possibleMove.IsColliding(illegalMoves))
                            continue;
                    } // FIXME: Also checked in instance.IsValid later.

                    if (this.constraints != null)
                    {
                        queryConstraint.Init(agentNum, possibleMove);

                        if (this.constraints.Contains(queryConstraint))
                            continue;
                    }

                    if (this.mustConstraints != null && makespan < this.mustConstraints.Length && // There may be a constraint on the timestep of the generated node
                        this.mustConstraints[makespan] != null &&
                        this.mustConstraints[makespan].ContainsKey(agentNum)) // This agent has a must constraint for this time step
                    {
                        if (this.mustConstraints[makespan][agentNum].Equals(possibleMove) == false)
                            continue;
                    }

                    // If the tile is not free (out of the grid or with an obstacle)
                    if (this.instance.IsValid(possibleMove) == false)
                        continue;

                    // Check against all the agents that have already moved to see if current move collides with their move
                    bool collision;

                    if (this.mstar)
                    {
                        bool agentInCollisionSet = fromNode.collisionSets.IsSingle(agentIndex);

                        if (agentInCollisionSet == false) // Only one move allowed
                        {
                            bool hasPlan = true;

                            if (hasPlan)
                            {
                                // If this move isn't its individually optimal one according to its planned route, return false.
                                if (this.instance.GetSingleAgentOptimalMove(fromNode.allAgentsState[agentIndex]).Equals(possibleMove) == false)
                                    continue;
                            }
                        }

                        var collidingWith = possibleMove.GetColliding(currentMoves);
                        collision = collidingWith.Count != 0;

                        if (collision)
                        {
                            // It is possible that possibleMove collides with two moves from currentMoves, even though currentMoves contains no collisions:
                            // Agent 0: 0,0 -> 1,0
                            // Agent 1: 0,1 -> 0,0
                            // Agent 2: 1,0 -> 0,0
                            // Arbitrarily choosing the first colliding agent:
                            int collidingAgentIndex = collidingWith[0];

                            bool otherAgentInColSet = fromNode.collisionSets.IsSingle(collidingAgentIndex);

                            // Check if one of the colliding agents isn't in the collision set yet
                            if (agentInCollisionSet == false ||
                                otherAgentInColSet == false)
                            {
                                if (this.debug)
                                    Debug.Print("Agent planned route collides with another move!");
                                bool success = false;
                                var conflict = new CbsConflict(
                                        agentIndex, collidingAgentIndex, possibleMove,
                                        intermediateMode.allAgentsState[collidingAgentIndex].lastMove, makespan);
                                if (this.debug)
                                    Debug.Print(conflict.ToString());
                                if (success == false)
                                {
                                    this.mstarBackPropagationConflictList.Add(conflict);
                                }
                            }
                        }
                    }
                    else
                    {
                        collision = possibleMove.IsColliding(currentMoves);
                    }

                    if (collision)
                        continue;
                    //----------------end paste from isValid

                    childNode = CreateSearchNode(currentNode);
                    childNode.allAgentsState[agentIndex].MoveTo(agentLocation);

                    childNode.prevStep = currentNode.prevStep; // Skip temporary node objects used during expansion process.
                    if (agentIndex == 0)
                        childNode.prevStep = currentNode;
                    if (agentIndex < currentNode.allAgentsState.Length - 1) // More agents need to move
                        childNode.currentMoves.Add(agentLocation, agentIndex);
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
        /// <param name="currentMoves"></param>
        /// <param name="makespan"></param>
        /// <param name="agentIndex"></param>
        /// <param name="fromNode"></param>
        /// <param name="intermediateMode"></param>
        /// <returns>true, if the move is possible.</returns>
        protected virtual bool IsValid(TimedMove possibleMove,
                                       IReadOnlyDictionary<TimedMove, int> currentMoves, int makespan,
                                       int agentIndex, WorldState fromNode, WorldState intermediateMode)
        {
            int agentNum = fromNode.allAgentsState[agentIndex].agent.agentNum;

            // Check if the proposed move is reserved in the plan of another agent.
            // This is used in IndependenceDetection's ImprovedID.
            if (this.illegalMoves != null)
            {
                if (possibleMove.IsColliding(illegalMoves))
                    return false;
            } // FIXME: Also checked in instance.IsValid later.

            if (this.constraints != null)
            {
                this.queryConstraint.Init(agentNum, possibleMove);

                if (this.constraints.Contains(queryConstraint))
                    return false;
            }

            if (this.mustConstraints != null && makespan < this.mustConstraints.Length && // There may be a constraint on the timestep of the generated node
                this.mustConstraints[makespan] != null &&
                this.mustConstraints[makespan].ContainsKey(agentNum)) // This agent has a must constraint for this time step
            {
                if (this.mustConstraints[makespan][agentNum].Equals(possibleMove) == false)
                    return false;
            }

            // If the tile is not free (out of the grid or with an obstacle)
            if (this.instance.IsValid(possibleMove) == false)
                return false;

            // Check against all the agents that have already moved to see if current move collides with their move
            bool collision;

            if (this.mstar)
            {
                bool agentInCollisionSet = fromNode.collisionSets.IsSingle(agentIndex);
                                            //fromNode.currentCollisionSet.Contains(agentIndex);// ||
                                            //(fromNode.individualMStarPlanBases[agentIndex] != null &&
                                            // this.mstarPlanBasesToTheirPlans[fromNode.individualMStarPlanBases[agentIndex]][agentIndex] == null); // Parent plan was abandoned. Imagine a backpropagation happened.
                
                if (agentInCollisionSet == false) // Only one move allowed
                {
                    bool hasPlan = true;
                    //// if the agent doesn't have a planned route, give it a planned route, 
                    ////if (fromNode.individualMStarPlanBases[agentIndex] == null) // No parent plan ever
                    //if (fromNode.individualMStarPlans[agentIndex] == null) // need to give this agent a plan
                    //{
                    //    //fromNode.individualMStarPlanBases[agentIndex] = fromNode;
                    //    fromNode.individualMStarBookmarks[agentIndex] = 0;
                    //    //if (this.mstarPlanBasesToTheirPlans.ContainsKey(fromNode) == false)
                    //    //    this.mstarPlanBasesToTheirPlans[fromNode] = new SinglePlan[this.instance.GetNumOfAgents()];
                    //    if (fromNode.individualMStarPlans == null)
                    //        fromNode.individualMStarPlans = new SinglePlan[this.instance.GetNumOfAgents()];

                    //    hasPlan = this.solveOneAgentForMstar(fromNode, agentIndex);

                    //    //if (hasPlan == false)
                    //    //    this.mstarPlanBasesToTheirPlans[fromNode][agentIndex] = null;

                    //    if (this.debug)
                    //    {
                    //        Debug.Print("Agent {0} plan:", agentIndex);
                    //        //Debug.Print(this.mstarPlanBasesToTheirPlans[fromNode.individualMStarPlanBases[agentIndex]][agentIndex].ToString());
                    //        Debug.Print(fromNode.individualMStarPlans[agentIndex].ToString());
                    //    }
                    //}

                    if (hasPlan)
                    {
                        // If this move isn't its individually optimal one according to its planned route, return false.
                        ////var planBase = fromNode.individualMStarPlanBases[agentIndex];
                        ////var plan = this.mstarPlanBasesToTheirPlans[planBase][agentIndex];
                        //var plan = fromNode.individualMStarPlans[agentIndex];
                        //Move allowed = plan.GetLocationAt(fromNode.individualMStarBookmarks[agentIndex] + 1);
                        //if (possibleMove.Equals(allowed) == false)
                        //    return false;
                        if (this.instance.GetSingleAgentOptimalMove(fromNode.allAgentsState[agentIndex]).Equals(possibleMove) == false)
                            return false;
                    }
                }

                var collidingWith = possibleMove.GetColliding(currentMoves);
                collision = collidingWith.Count != 0;

                if (collision)
                {
                    // It is possible that possibleMove collides with two moves from currentMoves, even though currentMoves contains no collisions:
                    // Agent 0: 0,0 -> 1,0
                    // Agent 1: 0,1 -> 0,0
                    // Agent 2: 1,0 -> 0,0
                    // Arbitrarily choosing the first colliding agent:
                    int collidingAgentIndex = collidingWith[0];

                    bool otherAgentInColSet = fromNode.collisionSets.IsSingle(collidingAgentIndex);
                                                //fromNode.currentCollisionSet.Contains(collidingAgentIndex);// ||
                                              //(fromNode.individualMStarPlanBases[collidingAgentIndex] != null &&
                                               //this.mstarPlanBasesToTheirPlans[fromNode.individualMStarPlanBases[collidingAgentIndex]][collidingAgentIndex] == null); // Parent plan was abandoned;

                    // Check if one of the colliding agents isn't in the collision set yet
                    if (agentInCollisionSet == false || 
                        otherAgentInColSet == false)
                    {
                        if (this.debug)
                            Debug.Print("Agent planned route collides with another move!");
                        bool success = false;
                        var conflict = new CbsConflict(
                                agentIndex, collidingAgentIndex, possibleMove,
                                intermediateMode.allAgentsState[collidingAgentIndex].lastMove, makespan);
                        if (this.debug)
                            Debug.Print(conflict.ToString());
                        //if (this.doMstarShuffle && agentInCollisionSet == false)
                        //{
                        //    ++this.mstarShuffles;
                        //    WorldState planStart = fromNode.GetPlanStart(agentIndex);
                        //    success = this.RMStarShuffleIndividualPath(conflict, true, planStart);
                        //    if (success)
                        //    {
                        //        //this.reinsertIntoOpenList(fromNode.individualMStarPlanBases[agentIndex]);
                        //        this.reinsertIntoOpenList(planStart);
                        //        if (this.debug)
                        //        {
                        //            Debug.Print("Agent {0} new plan:", agentIndex);
                        //            //Debug.Print(this.mstarPlanBasesToTheirPlans[fromNode.individualMStarPlanBases[agentIndex]][agentIndex].ToString());
                        //            Debug.Print(fromNode.individualMStarPlans[agentIndex].ToString());
                        //        }
                        //    }
                        //    else
                        //    {
                        //        if (this.debug)
                        //            Debug.Print("Replanning Agent {0} for the same cost failed", agentIndex);
                        //    }
                        //}
                        //if (this.doMstarShuffle && success == false && otherAgentInColSet == false)
                        //{
                        //    ++this.mstarShuffles;
                        //    WorldState planStart = fromNode.GetPlanStart(collidingAgentIndex);
                        //    success = this.RMStarShuffleIndividualPath(conflict, false, planStart);
                        //    if (success)
                        //    {
                        //        //this.reinsertIntoOpenList(fromNode.individualMStarPlanBases[collidingAgentIndex]);
                        //        this.reinsertIntoOpenList(planStart);
                        //        if (this.debug)
                        //        {
                        //            Debug.Print("Agent {0} new plan:", collidingAgentIndex);
                        //            //Debug.Print(this.mstarPlanBasesToTheirPlans[fromNode.individualMStarPlanBases[collidingAgentIndex]][collidingAgentIndex].ToString());
                        //            Debug.Print(fromNode.individualMStarPlans[collidingAgentIndex].ToString());
                        //        }
                        //    }
                        //    else
                        //    {
                        //        if (this.debug)
                        //            Debug.Print("Replanning Agent {0} for the same cost failed", collidingAgentIndex);
                        //    }
                        //}
                        if (success == false)
                        {
                            this.mstarBackPropagationConflictList.Add(conflict);
                        }
                    }
                }
            }
            else
            {
                collision = possibleMove.IsColliding(currentMoves);
            }
                
            return collision == false;
        }

        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        protected SinglePlan[] singlePlans;

        public virtual SinglePlan[] GetSinglePlans()
        {
            return this.singlePlans;
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
            this.Setup(problemInstance, -1, runner, -1);
        }

        /// <summary>
        /// Checks for closed list hits and handles them, check the max cost, does book-keeping
        /// of conflict counts, etc.
        /// </summary>
        /// <param name="currentNode"></param>
        /// <returns>Returns whether the node was inserted into the open list.</returns>
        protected virtual bool ProcessGeneratedNode(WorldState currentNode)
        {
            if (currentNode.f <= this.maxSolutionCost)
            // Assuming h is an admissable heuristic, no need to generate nodes that won't get us to the goal
            // within the budget
            {
                if (instance.parameters.ContainsKey(IndependenceDetection.CONFLICT_AVOIDANCE))
                {
                    // Accumulating the conflicts count from parent to child
                    // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
                    currentNode.cbsInternalConflicts = new Dictionary<int, int>(currentNode.prevStep.cbsInternalConflicts);
                    currentNode.conflictTimes = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimes)
                        currentNode.conflictTimes[kvp.Key] = new List<int>(kvp.Value);

                    currentNode.UpdateConflictCounts(
                        (IReadOnlyDictionary<TimedMove, List<int>>)instance.parameters[IndependenceDetection.CONFLICT_AVOIDANCE]);
                    // We're counting conflicts along the entire path, so the parent's conflicts count
                    // is added to the child's.

                    currentNode.potentialConflictsCount = currentNode.cbsInternalConflicts.Count;

                    // FIXME: The above code duplication with the CBS CAT. Some of the vars above are actually from CBS now.
                }

                if (instance.parameters.ContainsKey(CBS_LocalConflicts.CAT))
                {
                    // Accumulating the conflicts count from parent to child.
                    // We're counting conflicts along the entire path, so the parent's conflicts count is added to the child's:
                    currentNode.cbsInternalConflicts = new Dictionary<int,int>(currentNode.prevStep.cbsInternalConflicts);
                    currentNode.conflictTimes = new Dictionary<int, List<int>>();
                    foreach (var kvp in currentNode.prevStep.conflictTimes)
                        currentNode.conflictTimes[kvp.Key] = new List<int>(kvp.Value);

                    currentNode.UpdateConflictCounts(
                        ((IReadOnlyDictionary<TimedMove, List<int>>)instance.parameters[CBS_LocalConflicts.CAT]));

                    // Count one for every agent the path conflicts with any number of times:
                    currentNode.cbsInternalConflictsCount = currentNode.cbsInternalConflicts.Count;
                }

                if (this.mstar)
                {
                    //currentNode.currentCollisionSet = null; // Clear expansion data

                    currentNode.backPropagationSet = new HashSet<WorldState>();
                    currentNode.backPropagationSet.Add(currentNode.prevStep);

                    currentNode.collisionSets = new DisjointSets<int>();

                    // Copy parent's individual agent plans
                    //currentNode.individualMStarPlans = currentNode.prevStep.individualMStarPlans.ToArray<SinglePlan>();
                    //currentNode.individualMStarBookmarks = currentNode.prevStep.individualMStarBookmarks.ToArray<int>();
                }

                // If in closed list - only reopen if F is lower or node is otherwise preferred
                if (this.closedList.ContainsKey(currentNode) == true)
                {
                    ++this.closedListHits;
                    WorldState inClosedList = this.closedList[currentNode];
                    // Notice the agents may have gotten to their location from a different direction in this node.

                    if (this.mstar)
                    {
                        if (currentNode.g == inClosedList.g)
                        {
                            // Unite backpropagation sets and collision sets of inClosedList and currentNode.
                            // Notice only only of them is going to survive this method.
                            foreach (WorldState parent in currentNode.backPropagationSet) // Only one node expected on the backprop list - currentNode's parent. TODO: Assert this?
                                inClosedList.backPropagationSet.Add(parent);
                            currentNode.backPropagationSet = inClosedList.backPropagationSet;

                            //// Copy relavant individual paths
                            //for (int i = 0; i < this.instance.GetNumOfAgents(); i++)
                            //{
                            //    if (inClosedList.individualMStarPlanBases[i] != null &&

                            //        currentNode.individualMStarPlanBases[i] == null) // In the case where both nodes have a plan for an agent, arbitrarily choose currentNode's
                            //    {
                            //        currentNode.individualMStarPlanBases[i] = inClosedList.individualMStarPlanBases[i];
                            //        currentNode.individualMStarBookmarks[i] = inClosedList.individualMStarBookmarks[i];
                            //    }
                            //}
                        }

                        //inClosedList.collisionSets.CopyUnions(currentNode.collisionSets); // Not necessary - currentNode has no unions yet
                        currentNode.collisionSets = inClosedList.collisionSets;
                        RMStarCollisionBackPropagation(currentNode.collisionSets, currentNode.prevStep); // The collision sets are information about the future,
                                                                                                         // not the past, so they should 
                    }

                    // Since the nodes are equal, give them both the max of their H
                    bool improvedHOfThisNode = false;
                    bool improvedHOfOldNode = false;
                    if (currentNode.h < inClosedList.h)
                    {
                        currentNode.hBonus += inClosedList.h - currentNode.h;
                        currentNode.h = inClosedList.h;
                        improvedHOfThisNode = true;
                    }
                    if (inClosedList.h < currentNode.h)
                    {
                        inClosedList.hBonus += currentNode.h - inClosedList.h;
                        inClosedList.h = currentNode.h;
                        improvedHOfOldNode = true;
                    }

                    // LOAD OPTIMIZATION HACK:
                    // Work around the fact that a partially-expanded node implicitly has higher h, which isn't
                    // copied to currentNode. We don't want currentNode.CompareTo(inClosedList) to be -1 just for this reason.
                    ushort inClosedTargetDeltaFBackup = 0;
                    if (inClosedList.GetType() == typeof(WorldStateForPartialExpansion))
                    {
                        WorldStateForPartialExpansion inClosed = (WorldStateForPartialExpansion)inClosedList;
                        inClosedTargetDeltaFBackup = inClosed.targetDeltaF;
                        inClosed.targetDeltaF = 0;
                    }

                    int compareVal = currentNode.CompareTo(inClosedList);

                    // UNLOAD OPTIMIZATION HACK:
                    if (inClosedList.GetType() == typeof(WorldStateForPartialExpansion))
                    {
                        WorldStateForPartialExpansion inClosed = (WorldStateForPartialExpansion)inClosedList;
                        inClosed.targetDeltaF = inClosedTargetDeltaFBackup;
                    }

                    if (compareVal == -1 || // This node has smaller f, or preferred due to other consideration.
                        // Since we equalised their h, a smaller f means smaller g.
                        (this.mstar && this.doMstarShuffle && currentNode.g == inClosedList.g)) // Enables re-trying a node with different paths for the agents
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
                        // Reinsert old node with new higher F, if it's still in the open list.
                        // This pushes it further back in the open list so it certainly won't be smaller than the currently expanded node, so monotonicity is maintained.
                        if (this.openList.Remove(inClosedList)) // Cheap if it isn't there
                        {
                            inClosedList.Clear();
                            this.openList.Add(inClosedList);
                            ++noReopenHUpdates;
                        }
                    }
                }

                if (this.closedList.ContainsKey(currentNode) == false)
                {
                    //if (this.instance.m_vAgents.Length > 2)
                    //{
                    //    int a = 3;
                    //    int b = (a + 2) * 2;

                    //    int x1, x2, x3, y1, y2, y3;
                    //    x1 = 5; y1 = 3;
                    //    x2 = 2; y2 = 4;
                    //    x3 = 2; y3 = 2;
                    //    if (currentNode.allAgentsState[0].lastMove.x == x1 &&
                    //        currentNode.allAgentsState[0].lastMove.y == y1 &&
                    //        currentNode.allAgentsState[1].lastMove.x == x2 &&
                    //        currentNode.allAgentsState[1].lastMove.y == y2 &&
                    //        currentNode.allAgentsState[2].lastMove.x == x3 &&
                    //        currentNode.allAgentsState[2].lastMove.y == y3)
                    //    {
                    //        int c = 3;
                    //        int d = 3 * c;
                    //    }
                    //}

                    this.closedList.Add(currentNode, currentNode);
                    this.generated++; // Reopened nodes are also recounted here.
                    this.openList.Add(currentNode);
                    currentNode.expandedCountWhenGenerated = this.expanded;
                    if (this.debug)
                        Debug.Print($"Generated node {currentNode}");
                    return true;
                }
                else
                {
                    if (this.debug)
                        Debug.Print($"NOT generating node {currentNode}. It already exists.");
                }

                // What if in open list? This implementation immediately puts _generated_ nodes in the closed list,
                // so it only needs to check it and not the open list.
                // That actually makes a lot of sense: membership tests in heaps are expensive, and in hashtables are cheap.
                // This way we only need to _search_ the open list if we encounter a node that was already visited.
            }
            return false;
        }

        ///// <summary>
        ///// 
        ///// </summary>
        ///// <param name="agentIndex"></param>
        ///// <param name="fromNode"></param>
        ///// <returns>Whether the shuffle succeeded</returns>
        //bool RMStarShuffleIndividualPath(CbsConflict conflict, bool agentA, WorldState fromNode)
        //{
        //    int agentIndex = agentA ? conflict.agentAIndex : conflict.agentBIndex;
        //    //WorldState node = fromNode.individualMStarPlanBases[agentIndex];
        //    WorldState node = fromNode;

        //    if (this.mstarPlanBasesToTheirConstraints.ContainsKey(node) == false)
        //        this.mstarPlanBasesToTheirConstraints[node] = new HashSet<CbsConstraint>[this.instance.GetNumOfAgents()];
        //    if (this.mstarPlanBasesToTheirConstraints[node][agentIndex] == null)
        //        this.mstarPlanBasesToTheirConstraints[node][agentIndex] = new HashSet<CbsConstraint>();

        //    return solveOneAgentForMstar(node, conflict, agentA);
        //}

        //protected bool solveOneAgentForMstar(WorldState node, CbsConflict conflict, bool agentA)
        //{
        //    int agentIndex = agentA ? conflict.agentAIndex : conflict.agentBIndex;
        //    HashSet_U<CbsConstraint> constraints = null;
        //    HashSet<CbsConstraint> newConstraints = null;
        //    int oldMaxCost = int.MaxValue;
        //    if (this.instance.parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
        //        constraints = (HashSet_U<CbsConstraint>)this.instance.parameters[CBS_LocalConflicts.CONSTRAINTS];
        //    else
        //    {
        //        constraints = new HashSet_U<CbsConstraint>();
        //        this.instance.parameters[CBS_LocalConflicts.CONSTRAINTS] = constraints;
        //    }

        //    if (this.instance.parameters.ContainsKey(CBS_LocalConflicts.CAT) == false)
        //        this.instance.parameters[CBS_LocalConflicts.CAT] = new Dictionary_U<TimedMove, int>(); // Indicate TO CBS that another level is running above it

        //    if (this.debug)
        //    {
        //        Debug.Print("Planning for agent index: " + agentIndex + " in node: " + node);
        //    }
            
        //    newConstraints = this.mstarPlanBasesToTheirConstraints[node][agentIndex];
        //    CbsConstraint newConstraint = new CbsConstraint(conflict, this.instance, agentA);
        //    newConstraints.Add(newConstraint);
        //    constraints.Join(newConstraints);

        //    if (this.debug)
        //    {
        //        Debug.Print("Constraints: ");
        //        foreach (var constraint in newConstraints)
        //        {
        //            Debug.Print(constraint.ToString());
        //        }
        //    }
            
        //    oldMaxCost = this.maxCost;
        //    //this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = this.mstarPlanBasesToTheirPlans[node][agentIndex].GetSize() - 1;
        //    this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = node.individualMStarPlans[agentIndex].GetSize() - 1;

        //    bool success = solveOneAgentForMstar(node, agentIndex);

        //    constraints.Separate(newConstraints);
        //    this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = oldMaxCost;
        //    this.instance.parameters.Remove(CBS_LocalConflicts.CAT);

        //    return success;
        //}

        //protected bool solveOneAgentForMstar(WorldState node, int agentIndex)
        //{
        //    AgentState[] thisAgentOnly = new AgentState[1];
        //    thisAgentOnly[0] = node.allAgentsState[agentIndex];
        //    var subProblem = this.instance.Subproblem(thisAgentOnly);

        //    ClassicAStar astar = new ClassicAStar(this.heuristic);
        //    ICbsSolver solver = new CBS_LocalConflicts(astar, astar); // Uses a precomputed solution if possible
        //    solver.Setup(subProblem, this.runner);
        //    bool success = solver.Solve();

        //    if (success)
        //    {
        //        //this.mstarPlanBasesToTheirPlans[node][agentIndex] = solver.GetSinglePlans()[0];
        //        node.individualMStarPlans[agentIndex] = solver.GetSinglePlans()[0];
        //    }
        //    // else nothing. Don't null the old plan yet - it might be saved by a successful replan of the other agent

        //    return success;
        //}

        /// <summary>
        /// NOT the algorithm in the M* journal paper.
        /// They want each node to propagate its entire collision set, not just the new conflict that began the process.
        /// This implementation may be suitable for the M* algorithm as appears in the paper,
        /// but it isn't suitable when we want to backpropagate from a closed list hit,
        /// because then we don't have a specific conflict to propagate.
        /// </summary>
        /// <param name="conflict"></param>
        /// <param name="fromNode">
        /// Not the node where the collision happened, because it was never generated.
        /// The node from where the colliding moves were made.
        /// </param>
        void RMStarCollisionBackPropagation(CbsConflict conflict, WorldState fromNode)
        {
            if (this.debug)
                Debug.Print("Back prop!!");
            var queue = new Queue<WorldState>();
            queue.Enqueue(fromNode);

            while (queue.Count != 0)
            {
                var node = queue.Dequeue();

                bool onlyUnitedNow = node.collisionSets.Union(conflict.agentAIndex, conflict.agentBIndex);

                if (onlyUnitedNow)
                {
                    if (this.debug)
                        Console.WriteLine("Re-opening node {0} with an updated collision set", node);
                    this.reinsertIntoOpenList(node);

                    foreach (var next in node.backPropagationSet)
                        queue.Enqueue(next);
                }
            }
        }

        /// <summary>
        /// Not the paper's version either, since it always propagates the same collision sets.
        /// </summary>
        /// <param name="colSets">The collision sets of the _child_ of fromNode</param>
        /// <param name="fromNode"></param>
        void RMStarCollisionBackPropagation(DisjointSets<int> colSets, WorldState fromNode)
        {
            if (this.debug)
                Debug.Print("Back prop!!");
            var queue = new Queue<WorldState>();
            queue.Enqueue(fromNode);

            while (queue.Count != 0)
            {
                var node = queue.Dequeue();

                bool onlyUnitedNow = node.collisionSets.CopyUnions(colSets);

                if (onlyUnitedNow)
                {
                    if (this.debug)
                        Console.WriteLine("Re-opening node {0} with an updated collision set", node);
                    this.reinsertIntoOpenList(node);

                    foreach (var next in node.backPropagationSet)
                        queue.Enqueue(next);
                }
            }
        }

        void reinsertIntoOpenList(WorldState node)
        {
            this.openList.Remove(node);
            node.Clear();
            this.openList.Add(node); // Re-insert into open list
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
