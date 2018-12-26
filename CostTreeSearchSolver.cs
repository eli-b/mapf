using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// This class solves an instance of the MAPF problem using the increasing costs tree search algorithm.
    /// </summary>
    class CostTreeSearchSolver : ICbsSolver
    {
        protected Queue<CostTreeNode> openList;
        protected HashSet<CostTreeNode> closedList;
        public int totalCost;
        public int generatedHL;
        public int expandedHL;
        public int expandedLL;
        public int generatedLL;
        int accExpandedHL;
        int accGeneratedHL;
        int accExpandedLL;
        int accGeneratedLL;
        protected ProblemInstance problem;
        protected Run runner;
        public CostTreeNode costTreeNode;
        protected int costA;
        protected int costB;
        protected int sizeOfA;
        protected int maxCost;
        protected SinglePlan[] solution;
        protected int initialEstimate;
        protected int solutionDepth;
        public static int passed; // FIXME: Why static?
        protected Dictionary<TimedMove, List<int>> ID_CAT;
        protected Dictionary<TimedMove, List<int>> CBS_CAT;
        protected int minCAViolations;

        // These variables are for matching and pruning MDDs
        public static int[,,] edgesMatrix; // K(agent), V(from), V(to)
        public static int edgesMatrixCounter;
        public static int maxY; 

        public CostTreeSearchSolver()
        {
        }

        /// <summary>
        /// Return the name of the solver, useful for outputting results.
        /// </summary>
        /// <returns>The name of the solver</returns>
        public virtual String GetName()
        {
            return "CostTreeSearch+pairsMatch";
        }

        public override string ToString()
        {
            return GetName();
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }

        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="minTimeStep"></param>
        /// <param name="runner"></param>
        /// <param name="minCost">FIXME: Not taken into account, just added to comply with ICbsSolver</param>
        public virtual void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner, int minCost = -1)
        {
            this.minCAViolations = int.MaxValue;
            CostTreeSearchSolver.passed = 0;
            this.generatedHL = 1;
            this.expandedHL = 1;
            this.generatedLL = 0;
            this.expandedLL = 0;
            this.totalCost = Constants.TIMEOUT_COST;
            
            // If there exists relevant previously solved subproblems - use their solution as a lower bound
            //if (problemInstance.parameters.ContainsKey(CostTreeSearch.PARENT_GROUP1_KEY))
            //{
            //    costA = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP1_KEY])).solutionCost;
            //    costB = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP2_KEY])).solutionCost;
            //    sizeOfA = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP1_KEY])).Size();
            //}
            //else
            {
                costA = problemInstance.GetSingleAgentOptimalCost(problemInstance.m_vAgents[0]);
                costB = 0;
                sizeOfA = 1;
            }

            this.problem = problemInstance;
            this.runner = runner;

            closedList = new HashSet<CostTreeNode>();
            openList = new Queue<CostTreeNode>();
            int[] costs = new int[problem.GetNumOfAgents()];
            for (int i = 0; i < problem.GetNumOfAgents(); i++)
            {
                costs[i] = Math.Max(problem.GetSingleAgentOptimalCost(problem.m_vAgents[i]), minTimeStep);
            }

            openList.Enqueue(new CostTreeNode(costs)); // The root
            this.initialEstimate = openList.Peek().costs.Sum();

            // Store parameters used by IndependenceDetection's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(IndependenceDetection.MAXIMUM_COST_KEY))
                this.maxCost = (int)problemInstance.parameters[IndependenceDetection.MAXIMUM_COST_KEY];
            else
                this.maxCost = -1;

            if (problemInstance.parameters.ContainsKey(IndependenceDetection.CONFLICT_AVOIDANCE))
            {
                ID_CAT = (Dictionary<TimedMove, List<int>>)problemInstance.parameters[IndependenceDetection.CONFLICT_AVOIDANCE];
            }

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.CAT))
            {
                CBS_CAT = (Dictionary<TimedMove, List<int>>)problemInstance.parameters[CBS_LocalConflicts.CAT];
            }
        }

        /// <summary>
        /// Clears the relevant data structures and variables to free memory usage.
        /// </summary>
        public void Clear()
        {
            this.problem = null;
            this.closedList.Clear();
            this.openList.Clear();
            this.ID_CAT = null;
        }

        public Plan GetPlan() { return new Plan(solution); }

        /// <summary>
        /// Returns the cost of the solution found, or error codes otherwise.
        /// </summary>
        public int GetSolutionCost()
        {
            return this.totalCost;
        }

        public Dictionary<int, int> GetExternalConflictCounts()
        {
            throw new NotImplementedException();
        }

        public Dictionary<int, List<int>> GetConflictTimes()
        {
            throw new NotImplementedException();
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Expanded (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Passed");
            output.Write(Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Total Expanded Nodes (Low-Level): {0}", this.GetLowLevelExpanded());
            Console.WriteLine("Total Generated Nodes (Low-Level): {0}", this.GetLowLevelGenerated());
            Console.WriteLine("\"passed\": {0}", CostTreeSearchSolver.passed);

            output.Write(this.expandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedHL + Run.RESULTS_DELIMITER);
            output.Write(this.expandedLL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedLL + Run.RESULTS_DELIMITER);
            output.Write(CostTreeSearchSolver.passed + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 5;
            }
        }

        public void ClearStatistics()
        {
            // Own statistics cleared on Setup.
        }

        public void ClearAccumulatedStatistics()
        {
            this.accExpandedHL = 0;
            this.accGeneratedHL = 0;
            this.accExpandedLL = 0;
            this.accGeneratedLL = 0;
        }

        public void AccumulateStatistics()
        {
            this.accExpandedHL += this.expandedHL;
            this.accGeneratedHL += this.generatedHL;
            this.accExpandedLL += this.expandedLL;
            this.accGeneratedLL += this.generatedLL;
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, this.accExpandedHL);
            Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, this.accGeneratedHL);
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpandedLL);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGeneratedLL);
            Console.WriteLine("{0} Accumulated \"passed\": ?", this);

            output.Write(this.accExpandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.accGeneratedHL + Run.RESULTS_DELIMITER);
            output.Write(this.accExpandedLL + Run.RESULTS_DELIMITER);
            output.Write(this.accGeneratedLL + Run.RESULTS_DELIMITER);

            output.Write("" + Run.RESULTS_DELIMITER); // TODO: Can't accumulate what I don't yet understand...
        }

        /// <summary>
        /// Solves the instance that was set by a call to Setup()
        /// </summary>
        /// <returns></returns>
        public virtual bool Solve()
        {
            // TODO: Remove dead code.
            //int currentLevelCost = -1;
            //CostTreeNodeSolver next;
            //LinkedList<Move>[] ans = null;
            //int sumSubGroupA;
            //int sumSubGroupB;
            ////TODO if no solution found the algorithm will never stop
            //while (runner.ElapsedMilliseconds() <= Constants.MAX_TIME)
            //{
            //    costTreeNode = openList.First.Value;
            //    if (costTreeNode.costs.Sum() > currentLevelCost)
            //    {
            //        costTreeDepth++;
            //        currentLevelCost = costTreeNode.costs.Sum();
            //    }
            //    sumSubGroupA = costTreeNode.Sum(0, sizeOfA);
            //    sumSubGroupB = costTreeNode.Sum(sizeOfA, costTreeNode.costs.Length);

            //    //if maxValue is set i.e. were loking for a given cost solution
            //    if (maxCost != -1)
            //    {
            //        //if we are above the given solution return no solution found
            //        if (sumSubGroupA + sumSubGroupB > maxCost)
            //            return false;
            //        //if we are below the given solution no need to do goal test just expand node
            //        if (sumSubGroupA + sumSubGroupB < maxCost)
            //        {
            //            costTreeNode.Expand(openList, closedList);
            //            openList.RemoveFirst();
            //            continue;
            //        }
            //    }
            //    // Reuse optimal solutions to previously solved subproblems
            //    if (sumSubGroupA >= costA && sumSubGroupB >= costB)
            //    {
            //        next = new CostTreeNodeSolverDDBF(problem, costTreeNode, runner);
            //        generated++;
            //        ans = next.Solve(conflictTable);
            //        expanded += next.GetExpanded();
            //        if (ans != null)
            //        {
            //            if (ans[0] != null)
            //            {
            //                totalCost = 0;
            //                for (int i = 0; i < next.costs.Length; i++)
            //                {
            //                    totalCost += next.costs[i];
            //                }
            //                solution = new Plan(ans);
            //                return true;
            //            }
            //        }
            //    }
            //    costTreeNode.Expand(openList, closedList);
            //    openList.RemoveFirst();
            //}
            return false;
        }

        public int GetHighLevelExpanded() { return this.expandedHL; }
        public int GetHighLevelGenerated() { return this.generatedHL; }
        public int GetLowLevelExpanded() { return this.expandedLL; }
        public int GetLowLevelGenerated() { return this.generatedLL; }
        public int GetExpanded() { return this.expandedHL; }
        public int GetGenerated() { return this.generatedHL; }
        public int GetAccumulatedExpanded() { return this.accExpandedHL; }
        public int GetAccumulatedGenerated() { return this.accGeneratedHL; }
        public int GetSolutionDepth() { return this.solutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public int GetMaxGroupSize() { return problem.m_vAgents.Length; }
        public SinglePlan[] GetSinglePlans() { return solution; }

        public virtual int[] GetSingleCosts()
        {
            return null;
        }
    }

    class CostTreeSearchSolverOldMatching : CostTreeSearchSolver
    {
        int syncSize;

        public CostTreeSearchSolverOldMatching(int syncSize) : base() { this.syncSize = syncSize; }

        public override bool Solve()
        {
            CostTreeNodeSolverOldMatching nodeSolver = new CostTreeNodeSolverOldMatching(problem, runner); // TODO: This begs a factory.
            SinglePlan[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;

            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, this.sizeOfA);
                sumSubGroupB = costTreeNode.Sum(this.sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)  // TODO: For makespan, using Max of the group "sums"
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                // Eli: Does this actually happen? Add a statistic
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    nodeSolver.Setup(costTreeNode, syncSize);
                    expandedHL++;
                    ans = nodeSolver.Solve(ID_CAT, CBS_CAT);
                    generatedLL += nodeSolver.generated;
                    expandedLL += nodeSolver.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.EXHAUSTIVE_ICTS == false)
                            {
                                this.totalCost = nodeSolver.totalCost;
                                this.solutionDepth = nodeSolver.totalCost - this.initialEstimate;
                                this.solution = ans;
                                CostTreeSearchSolver.passed--;
                                return true;
                            }

                            if (nodeSolver.caViolations < this.minCAViolations)
                            {
                                this.totalCost = nodeSolver.totalCost;
                                this.solutionDepth = nodeSolver.totalCost - this.initialEstimate;
                                this.solution = ans;
                                CostTreeSearchSolver.passed--;
                                this.minCAViolations = nodeSolver.caViolations;
                                this.maxCost = totalCost;
                                if (nodeSolver.caViolations == 0)
                                    return true;
                            }
                        }
                    }
                }

                costTreeNode.Expand(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.Dequeue();
            }

            this.totalCost = Constants.TIMEOUT_COST;
            this.solutionDepth = nodeSolver.totalCost - this.initialEstimate; // A lower bound
            Console.WriteLine("Out of time");
            return false; 
        }

        public override String GetName() 
        {
            return "ICTS " + syncSize + "E ";
        }
    }

    class CostTreeSearchSolverNoPruning : CostTreeSearchSolver
    {
        public override bool Solve()
        {
            CostTreeNodeSolverDDBF next = new CostTreeNodeSolverDDBF(problem, this.runner);
            SinglePlan[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.Sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    next.Setup(costTreeNode);
                    expandedHL++;
                    ans = next.Solve(ID_CAT,CBS_CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.EXHAUSTIVE_ICTS == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                this.minCAViolations = next.caViolations;
                                maxCost = totalCost;
                                if (next.caViolations == 0)
                                    return true;
                                passed--;
                            }
                        }
                    }
                }
                passed++;
                costTreeNode.Expand(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.Dequeue();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() { return "ICTS "; }
    }

    class CostTreeSearchSolverKMatch : CostTreeSearchSolver
    {
        int maxGroupChecked;
        public CostTreeSearchSolverKMatch(int maxGroupChecked) : base() { this.maxGroupChecked = maxGroupChecked; }
        public override void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }
        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost = -1)
        {
            edgesMatrix = new int[problemInstance.m_vAgents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            maxY = problemInstance.GetMaxY();
            base.Setup(problemInstance, minDepth, runner, minCost);
        }
        public override bool Solve()
        {
            CostTreeNodeSolverKSimpleMatching next = new CostTreeNodeSolverKSimpleMatching(problem, runner);
            SinglePlan[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.Sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    next.setup(costTreeNode, maxGroupChecked);
                    expandedHL++;
                    ans = next.Solve(ID_CAT,CBS_CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.EXHAUSTIVE_ICTS == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                passed--;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                passed--;
                                this.minCAViolations = next.caViolations;
                                maxCost = totalCost;
                                if (next.caViolations == 0)
                                    return true;
                            }
                        }
                    }
                }
                costTreeNode.Expand(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.Dequeue();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() { return "ICTS+" + maxGroupChecked + "S "; }
    }

    class CostTreeSearchSolverRepeatedMatch : CostTreeSearchSolver
    {
        int syncSize;
        public CostTreeSearchSolverRepeatedMatch(int syncSize) : base() { this.syncSize = syncSize; }
        public override void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }
        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost = -1)
        {
            edgesMatrix = new int[problemInstance.m_vAgents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            maxY = problemInstance.GetMaxY();
            base.Setup(problemInstance, minDepth, runner, minCost);
        }
        public override bool Solve()
        {
            //int time = 0;
            CostTreeNodeSolverRepeatedMatching next = new CostTreeNodeSolverRepeatedMatching(problem, runner);
            SinglePlan[] ans = null;
            Stopwatch sw = new Stopwatch();
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                sw.Reset();
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.Sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    next.setup(costTreeNode, syncSize);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    sw.Start();
                    ans = next.Solve(ID_CAT, CBS_CAT);
                    sw.Stop();
                    Console.WriteLine(sw.ElapsedMilliseconds);
                    if (sw.ElapsedMilliseconds > 0)
                        Console.ReadLine();
                    generatedLL += next.getGenerated();
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.EXHAUSTIVE_ICTS == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                passed--;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                passed--;
                                this.minCAViolations = next.caViolations;
                                maxCost = totalCost;
                                if (next.caViolations == 0)
                                    return true;
                            }
                        }
                    }
                }
                costTreeNode.Expand(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.Dequeue();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() { return "ICTS " + syncSize + "RE"; }
    }
}
