using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;//////////////////////
using System.IO;
namespace CPF_experiment
{
    /// <summary>
    /// This class solves an instance of the MAPF problem using the cost tree search algorithm.
    /// </summary>
    class CostTreeSearchSolver : ICbsSolver
    {
        protected LinkedList<CostTreeNode> openList;
        protected HashSet<CostTreeNode> closedList;
        public int totalCost;
        public int generatedHL;
        public int expandedHL;
        public int expandedLL;
        public int generatedLL;
        protected ProblemInstance problem;
        protected Run runner;
        public CostTreeNode costTreeNode;
        protected int costA;
        protected int costB;
        protected int sizeOfA;
        protected int maxCost;
        protected LinkedList<Move>[] solution;
        protected int initialHeuristics;
        public static int passed;
        protected HashSet<TimedMove> ID_CAT;
        protected HashSet_U<TimedMove> CBS_CAT;
        protected int minCAViolations;

        // These variables are for matching and pruning MDDs
        public static int[,,] edgesMatrix; // K(agent), V(from), V(to)
        public static int edgesMatrixCounter;
        public static int maxY; 
        /// /////////////////////////////


        public CostTreeSearchSolver()
        {
            closedList = new HashSet<CostTreeNode>();
            openList = new LinkedList<CostTreeNode>();
        }

        /// <summary>
        /// Return the name of the solver, usefull for outputing results.
        /// </summary>
        /// <returns>The name of the solver</returns>
        public virtual String GetName() { return "CostTreeSearch+pairsMatch"; }

        public virtual void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }

        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            minCAViolations = int.MaxValue;
            passed = 0;
            this.generatedHL = 1;
            this.expandedHL = 1;
            this.generatedLL = 0;
            this.expandedLL = 0;
            this.totalCost = Constants.TIMEOUT_COST;
            
            // If there exists relevant previously solved subproblems - use their solution as a lower bound
            if (problemInstance.parameters.ContainsKey(CostTreeSearch.PARENT_GROUP1_KEY))
            {
                costA = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP1_KEY])).solutionCost;
                costB = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP2_KEY])).solutionCost;
                sizeOfA = ((AgentsGroup)(problemInstance.parameters[CostTreeSearch.PARENT_GROUP1_KEY])).Size();
            }
            else
            {
                costA = problemInstance.m_vAgents[0].h;
                costB = 0;
                sizeOfA = 1;
            }
            this.problem = problemInstance;
            this.runner = runner;

            closedList = new HashSet<CostTreeNode>();
            openList = new LinkedList<CostTreeNode>();
            int[] costs = new int[problem.GetNumOfAgents()];
            AgentState temp;
            for (int i = 0; i < problem.GetNumOfAgents(); i++)
            {
                temp=problem.m_vAgents[i];
                costs[i] = Math.Max(problem.GetSingleAgentShortestPath(temp), minDepth);
            }

            openList.AddFirst(new CostTreeNode(costs));
            this.initialHeuristics = openList.First.Value.costs.Sum();

            // Store parameters used by Trevor's Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = -1;

            if (problemInstance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
            {
                ID_CAT = ((HashSet<TimedMove>)problemInstance.parameters[Trevor.CONFLICT_AVOIDANCE]);
            }
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
            {
                CBS_CAT = ((HashSet_U<TimedMove>)problemInstance.parameters[CBS_LocalConflicts.INTERNAL_CAT]);
            }
        }

        public void SetHeuristic(HeuristicCalculator heuristic) {}

        public HeuristicCalculator GetHeuristic()
        {
            return null;
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
            output.Write(this.expandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedHL + Run.RESULTS_DELIMITER);
            output.Write(this.expandedLL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedLL + Run.RESULTS_DELIMITER);
            output.Write(passed + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Solves the instance that was set by a call to Setup()
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public virtual bool Solve()
        {
            //int currentLevelCost = -1;
            //CostTreeNodeSolver next;            
            //LinkedList<Move>[] ans = null;
            //int sumSubGroupA;
            //int sumSubGroupB;
            ////TODO if no solution found the algorithm will never stop
            //while (runner.ElapsedMilliseconds < Constants.MAX_TIME)
            //{
            //    costTreeNode = openList.First.Value;
            //    if (costTreeNode.costs.Sum() > currentLevelCost)
            //    {
            //        costTreeDepth++;
            //        currentLevelCost = costTreeNode.costs.Sum();
            //    }
            //    sumSubGroupA = costTreeNode.sum(0, sizeOfA);
            //    sumSubGroupB = costTreeNode.sum(sizeOfA, costTreeNode.costs.Length);

            //    //if maxValue is set i.e. were loking for a given cost solution
            //    if (maxCost != -1)
            //    {
            //        //if we are above the given solution return no solution found
            //        if (sumSubGroupA + sumSubGroupB > maxCost)
            //            return false;
            //        //if we are below the given solution no need to do goal test just expand node
            //        if (sumSubGroupA + sumSubGroupB < maxCost)
            //        {
            //            costTreeNode.expandNode(openList, closedList);
            //            openList.RemoveFirst();
            //            continue;
            //        }
            //    }
            //    // Reuse optimal solutions to previously solved subproblems
            //    if (sumSubGroupA >= costA && sumSubGroupB >= costB)
            //    {
            //        next = new CostTreeNodeSolverDDBF(problem, costTreeNode,runner);
            //        generated++;
            //        ans = next.Solve(conflictTable);
            //        expanded += next.getExpanded();
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
            //    costTreeNode.expandNode(openList, closedList);
            //    openList.RemoveFirst();
            //}
            return false;
        }

        public int GetHighLevelExpanded() { return this.expandedHL; }
        public int GetHighLevelGenerated() { return this.generatedHL; }
        public int GetLowLevelExpanded() { return this.expandedLL; }
        public int GetLowLevelGenerated() { return this.generatedLL; }
        public int GetSolutionDepth() { return this.totalCost - initialHeuristics; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public int GetMaxGroupSize() { return problem.m_vAgents.Length; }
        public SinglePlan[] GetSinglePlans() { return SinglePlan.getSinglePlans(solution); }
    }

    class CostTreeSearchSolverOldMatching : CostTreeSearchSolver
    {
        int syncSize;
        public CostTreeSearchSolverOldMatching(int syncSize) : base() { this.syncSize = syncSize; }
        public override bool Solve()
        {
            //long time=0;
            CostTreeNodeSolverOldMatching next = new CostTreeNodeSolverOldMatching(problem,runner);
            LinkedList<Move>[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.First.Value;
                sumSubGroupA = costTreeNode.sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.expandNode(openList, closedList);
                        openList.RemoveFirst();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    next.setup(costTreeNode,syncSize);
                    expandedHL++;
                    ans = next.Solve(ID_CAT,CBS_CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.exhaustiveIcts == false)
                            {
                                totalCost = next.costs.Sum();
                                solution = ans;
                                passed--;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.costs.Sum();
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
                costTreeNode.expandNode(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.RemoveFirst();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() 
        { return "ICTS " + syncSize + "E "; }
    }

    class CostTreeSearchSolverNoPruning : CostTreeSearchSolver
    {
        public override bool Solve()
        {
            CostTreeNodeSolverDDBF next = new CostTreeNodeSolverDDBF(problem, this.runner);
            LinkedList<Move>[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.First.Value;
                sumSubGroupA = costTreeNode.sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.expandNode(openList, closedList);
                        openList.RemoveFirst();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costA && sumSubGroupB >= costB)
                {
                    next.setup(costTreeNode);
                    expandedHL++;
                    ans = next.Solve(ID_CAT,CBS_CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (Constants.exhaustiveIcts == false)
                            {
                                totalCost = next.costs.Sum();
                                solution = ans;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.costs.Sum();
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
                costTreeNode.expandNode(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.RemoveFirst();
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
        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            edgesMatrix = new int[problemInstance.m_vAgents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            maxY = problemInstance.GetMaxY();
            base.Setup(problemInstance, minDepth, runner);
        }
        public override bool Solve()
        {
            CostTreeNodeSolverKSimpaleMatching next = new CostTreeNodeSolverKSimpaleMatching(problem, runner);
            LinkedList<Move>[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.First.Value;
                sumSubGroupA = costTreeNode.sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.expandNode(openList, closedList);
                        openList.RemoveFirst();
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
                            if (Constants.exhaustiveIcts == false)
                            {
                                totalCost = next.costs.Sum();
                                solution = ans;
                                passed--;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.costs.Sum();
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
                costTreeNode.expandNode(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.RemoveFirst();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() { return "ICTS+" + maxGroupChecked + "S "; }
    }

    class CostTreeSearchSolverRepatedMatch : CostTreeSearchSolver
    {
        int syncSize;
        public CostTreeSearchSolverRepatedMatch(int syncSize) : base() { this.syncSize = syncSize; }
        public override void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }
        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            edgesMatrix = new int[problemInstance.m_vAgents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            maxY = problemInstance.GetMaxY();
            base.Setup(problemInstance, minDepth, runner);
        }
        public override bool Solve()
        {
            //int time = 0;
            CostTreeNodeSolverRepatedMatching next = new CostTreeNodeSolverRepatedMatching(problem, runner);
            LinkedList<Move>[] ans = null;
            Stopwatch sw = new Stopwatch();
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                sw.Reset();
                costTreeNode = openList.First.Value;
                sumSubGroupA = costTreeNode.sum(0, sizeOfA);
                sumSubGroupB = costTreeNode.sum(sizeOfA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minCAViolations != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.expandNode(openList, closedList);
                        openList.RemoveFirst();
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
                            if (Constants.exhaustiveIcts == false)
                            {
                                totalCost = next.costs.Sum();
                                solution = ans;
                                passed--;
                                return true;
                            }

                            if (next.caViolations < this.minCAViolations)
                            {
                                totalCost = next.costs.Sum();
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
                costTreeNode.expandNode(openList, closedList);
                generatedHL += costTreeNode.costs.Length;
                openList.RemoveFirst();
            }
            totalCost = Constants.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            return false; 
        }
        public override String GetName() { return "ICTS " + syncSize + "RE"; }
    }
}
