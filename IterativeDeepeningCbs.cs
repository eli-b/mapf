using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;


namespace mapf
{
    /// <summary>
    /// Is the CBS open list ever large enough to warrant this?
    /// Consider foregoing this implementation. It needs maintainence anyway.
    /// </summary>
    class IterativeDeepeningCBS : ISolver, IHeuristicSolver<CbsNode>
    {
        protected ProblemInstance instance;
        public int highLevelExpanded;
        public int highLevelGenerated;
        public int totalCost;
        protected Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        protected int maxCost;
        protected ICbsSolver solver;
        protected ICbsSolver singleAgentSolver;
        protected int mergeThreshold, nextF, fBound;
        protected int minDepth;
        protected int maxThreshold;
        protected int maxSizeGroup;
        protected IHeuristicCalculator<CbsNode> heuristic;
        int[][] globalConflictsCounter;
        CbsNode root;
        bool topMost;

        public IterativeDeepeningCBS(ICbsSolver singleAgentSolver, ICbsSolver generalSolver, int maxThreshold = -1,
            int currentThreshold = -1, IHeuristicCalculator<CbsNode> heuristic = null)
        {
            this.mergeThreshold = currentThreshold;
            this.solver = generalSolver;
            this.singleAgentSolver = singleAgentSolver;
            this.maxThreshold = maxThreshold;
            this.heuristic = heuristic;
            if (currentThreshold < maxThreshold)
            {
                //this.solver = new MACBS_WholeTreeThreshold(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public void Clear()
        {
            this.instance = null;
            this.solver.Clear();
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);

            this.solver.OutputStatisticsHeader(output);
        }

        public void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 2 + this.solver.NumStatsColumns;
            }
        }

        public void ClearStatistics()
        {
            // Own statistics cleared on Setup.

            if (this.topMost)
                this.solver.ClearAccumulatedStatistics();
        }

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);

            if (root.Solve(minDepth) == false)
            {
                AgentState.EquivalenceOverDifferentTimes = equivalenceWasOn;
                return false;
            }

            fBound = root.f;

            // fBound must be <= maxCost
            while (fBound < maxCost)
            {
                nextF = int.MaxValue;
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    AgentState.EquivalenceOverDifferentTimes = equivalenceWasOn;
                    return false;
                }
                if (Expand(root, root.GetConflict()))
                {
                    AgentState.EquivalenceOverDifferentTimes = equivalenceWasOn;
                    return true;
                }
                fBound = nextF;
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            AgentState.EquivalenceOverDifferentTimes = equivalenceWasOn;
            return false;
        }

        public virtual bool Expand(CbsNode node, CbsConflict conflict)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                return false;
            highLevelExpanded++;
            if (conflict == null)
            {
                this.totalCost = node.g;
                this.goalNode = node;
                this.solution = node.CalculateJointPlan();
                this.Clear();
                return true;
            }
            CbsNode toAdd;
            CbsConstraint con2 = new CbsConstraint(conflict, instance, false);
            CbsConstraint con1 = new CbsConstraint(conflict, instance, true);
            byte stepLength=0;
            if (conflict.isVertexConflict)
                stepLength = 1;
            bool ok1 = false, ok2 = false;

            if (node.f + conflict.timeStep + stepLength - node.PathLength(conflict.agentAIndex) <= fBound)
            {
                ok1 = true;
                if (node.DoesMustConstraintAllow(con1))
                {
                    toAdd = new CbsNode(node, con1, conflict.agentAIndex);
                    toAdd.SetMustConstraint(con2);

                    if (toAdd.Replan3b(conflict.agentAIndex, Math.Max(minDepth, conflict.timeStep)))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.f <= fBound)
                        {
                            if (Expand(toAdd, toAdd.GetConflict()))
                                return true;
                        }
                        else if (toAdd.f < nextF)
                            nextF = toAdd.f;
                    }
                }
            }

            if (node.f + conflict.timeStep + stepLength - node.PathLength(conflict.agentBIndex) <= fBound)
            {
                ok2 = true;
                if (node.DoesMustConstraintAllow(con2))
                {
                    toAdd = new CbsNode(node, con2, conflict.agentBIndex);
                    toAdd.SetMustConstraint(con1);

                    if (toAdd.Replan3b(conflict.agentBIndex, Math.Max(minDepth, conflict.timeStep)))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.f <= fBound)
                        {
                            if (Expand(toAdd, toAdd.GetConflict()))
                                return true;
                        }
                        else if (toAdd.f < nextF)
                            nextF = toAdd.f;
                    }
                }
            }

            if (ok1 && ok2)
            {
                toAdd = new CbsNode(node, con1, conflict.agentAIndex);
                if (toAdd.Replan3b(conflict.agentAIndex, Math.Max(minDepth, conflict.timeStep)))
                {
                    if (toAdd.f <= fBound)
                    {
                        toAdd = new CbsNode(toAdd, con2, conflict.agentBIndex);
                        if (toAdd.Replan(conflict.agentBIndex, Math.Max(minDepth, conflict.timeStep)))
                            // FIXME: Should this really use the regular Replan() or was this a typo?
                        {
                            this.highLevelGenerated++;
                            if (toAdd.f <= fBound)
                            {
                                if (Expand(toAdd, toAdd.GetConflict()))
                                    return true;
                            }
                            else if (toAdd.f < nextF)
                                nextF = toAdd.f;
                        }
                    }
                }
            }
            return false;
        }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end)
        {
        }

        public int GetSolutionDepth() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public SinglePlan[] getSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }
        public int GetHighLevelExpanded() { return highLevelExpanded; }
        public int GetHighLevelGenerated() { return highLevelGenerated; }
        public int GetLowLevelExpanded() { return this.solver.GetAccumulatedExpanded(); }
        public int GetLowLevelGenerated() { return this.solver.GetAccumulatedGenerated(); }
        public int GetExpanded() { return highLevelExpanded; }
        public int GetGenerated() { return highLevelGenerated; }
        public int GetMaxGroupSize()
        {
            return this.maxSizeGroup;
        }

        bool equivalenceWasOn;

        public void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.equivalenceWasOn = AgentState.EquivalenceOverDifferentTimes == true;
            AgentState.EquivalenceOverDifferentTimes = false;
            globalConflictsCounter = new int[problemInstance.agents.Length][];
            for (int i = 0; i < globalConflictsCounter.Length; i++)
            {
                globalConflictsCounter[i] = new int[i];
                for (int j = 0; j < i; j++)
                {
                    globalConflictsCounter[i][j] = 0;
                }
            }

            this.instance = problemInstance;
            this.runner = runner;
            //root = new CbsNode(instance.agents.Length, problemInstance, this.solver, this.lowLevelSolver, this); // FIXME!
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            maxSizeGroup = 1;
            this.totalCost = 0;

            if (problemInstance.parameters.ContainsKey(IndependenceDetection.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[IndependenceDetection.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(CBS.CAT) == false) // Top-most CBS only
            {
                problemInstance.parameters[CBS.CAT] = new HashSet_U<TimedMove>();
                problemInstance.parameters[CBS.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                problemInstance.parameters[CBS.MUST_CONSTRAINTS] = new List<CbsConstraint>();
                this.topMost = true;
            }
            else
                this.topMost = false;

            minDepth = 0;
        }

        public IHeuristicCalculator<CbsNode> GetHeuristic()
        {
            return this.heuristic;
        }

        protected bool MergeConflicting(CbsNode node)
        {
            return node.ShouldMerge(mergeThreshold, globalConflictsCounter);
        }

        protected void addToGlobalConflictCount(CbsConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentAIndex, conflict.agentBIndex)][Math.Min(conflict.agentAIndex, conflict.agentBIndex)]++;
        }

        public string GetName()
        {
            if (mergeThreshold == -1)
                return "DFID-CBS3b";
            return "Bad Input As Solver CBS_IDA*";
        }
    }
}
