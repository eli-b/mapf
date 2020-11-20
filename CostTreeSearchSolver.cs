using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using System.Collections.Immutable;

namespace mapf
{
    /// <summary>
    /// This class solves an instance of the MAPF problem using the increasing costs tree search algorithm.
    /// </summary>
    abstract class CostTreeSearchSolver : ICbsSolver, IIndependenceDetectionSolver
    {
        /// <summary>
        /// This determines whether ICTS should search for a solution with lowest conflicts for the ID framework.
        /// This may require going over all goal nodes with the same cost until all are exhausted or a plan with zero conflicts is found.
        /// Even if this is set to false, ICTS will try to minimize the number of conflicts it creates.
        /// </summary>
        public bool exhaustive = false;
        protected Queue<CostTreeNode> openList;
        protected HashSet<CostTreeNode> closedList;
        public int totalCost;
        public int generatedHL;
        public int expandedHL;
        public int expandedLL;
        public int generatedLL;
        public int survivedPruningHL;
        public int goalTestSkipped;
        int accExpandedHL;
        int accGeneratedHL;
        int accExpandedLL;
        int accGeneratedLL;
        int accSurvivedPruningHL;
        int accGoalTestSkipped;
        protected ProblemInstance problem;
        protected Run runner;
        public CostTreeNode costTreeNode;
        protected int costParentGroupA;
        protected int costParentGroupB;
        protected int sizeParentGroupA;
        protected int minCost;
        protected int maxCost;
        protected SinglePlan[] solution;
        protected int[] costs;
        protected int initialEstimate;
        protected int solutionDepth;
        protected ConflictAvoidanceTable CAT;
        protected ISet<TimedMove> reserved;
        protected int minConflictsNotAvoided;

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

        public virtual void Setup(ProblemInstance problemInstance, Run runner) {
            
            
            Setup(problemInstance, 0, runner, null);
        }

        /// <summary>
        /// For new groups under Independence Detection
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="runner"></param>
        /// <param name="CAT"></param>
        /// <param name="parentGroup1Cost"></param>
        /// <param name="parentGroup2Cost"></param>
        /// <param name="parentGroup1Size"></param>
        public virtual void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                   int parentGroup1Cost, int parentGroup2Cost, int parentGroup1Size)
        {
            // Use the solutions of previously solved subproblems as a lower bound
            this.costParentGroupA = parentGroup1Cost;
            this.costParentGroupB = parentGroup2Cost;
            this.sizeParentGroupA = parentGroup1Size;

            Setup(problemInstance, 0, runner, CAT, minCost: parentGroup1Cost + parentGroup2Cost, maxCost: int.MaxValue);
        }

        /// <summary>
        /// For replanning groups to resolve a conflict under independence Detection
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="runner"></param>
        /// <param name="CAT"></param>
        /// <param name="targetCost">/// </param>
        /// <param name="illegalMoves"></param>
        public virtual void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                   int targetCost, ISet<TimedMove> reserved)
        {
            this.reserved = reserved;
            Setup(problemInstance, 0, runner, CAT, minCost: targetCost, maxCost: targetCost);
        }

        /// <summary>
        /// Setup the relevant data structures for a run (possibly under CBS).
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="minTimeStep"></param>
        /// <param name="runner"></param>
        /// <param name="CAT"></param>
        /// <param name="constraints"></param>
        /// <param name="positiveConstraints"></param>
        /// <param name="minCost"></param>
        /// <param name="maxCost"></param>
        /// <param name="mdd">FIXME: Not taken into account, just added to comply with ICbsSolver</param>
        public virtual void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner,
                                  ConflictAvoidanceTable CAT = null,
                                  ISet<CbsConstraint> constraints = null, ISet<CbsConstraint> positiveConstraints = null,
                                  int minCost = -1, int maxCost = int.MaxValue, MDD mdd = null)
        {
            this.minConflictsNotAvoided = int.MaxValue;
            this.survivedPruningHL = 0;
            this.goalTestSkipped = 0;
            this.generatedHL = 1;
            this.expandedHL = 1;
            this.generatedLL = 0;
            this.expandedLL = 0;
            this.totalCost = Constants.TIMEOUT_COST;
            
            this.problem = problemInstance;
            this.runner = runner;

            closedList = new HashSet<CostTreeNode>();
            openList = new Queue<CostTreeNode>();
            int[] costs = new int[problem.GetNumOfAgents()];
            for (int i = 0; i < problem.GetNumOfAgents(); i++)
            {
                costs[i] = Math.Max(problem.GetSingleAgentOptimalCost(problem.agents[i]), minTimeStep);  // TODO: Use the time of the latest constraint on each agent!
            }

            openList.Enqueue(new CostTreeNode(costs)); // The root
            this.initialEstimate = openList.Peek().costs.Sum();  // TODO: Support other cost functions

            // Store parameters used by the Independence Detection algorithm
            this.maxCost = maxCost;
            this.minCost = minCost;

            this.CAT = CAT;
        }

        /// <summary>
        /// Clears the relevant data structures and variables to free memory usage.
        /// </summary>
        public void Clear()
        {
            this.problem = null;
            this.closedList.Clear();
            this.openList.Clear();
            this.CAT = null;
            this.reserved = null;
            // Set trivial values for subproblem data
            this.costParentGroupA = 0;
            this.costParentGroupB = 0;
            this.sizeParentGroupA = 1;
        }

        public Plan GetPlan() { return new Plan(solution); }

        /// <summary>
        /// Returns the cost of the solution found, or error codes otherwise.
        /// </summary>
        public int GetSolutionCost()
        {
            return this.totalCost;
        }

        protected Dictionary<int, int> conflictCounts;
        protected Dictionary<int, List<int>> conflictTimes;

        public Dictionary<int, int> GetExternalConflictCounts()
        {
            return conflictCounts;
        }

        public Dictionary<int, List<int>> GetConflictTimes()
        {
            return conflictTimes;
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
            output.Write(this.ToString() + " HL Survived Pruning");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " HL Goal Tests Skipped");
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
            Console.WriteLine("Total Survived Pruning: {0}", this.survivedPruningHL);
            Console.WriteLine("Total High-Level Goal Tests Skipped: {0}", this.goalTestSkipped);

            output.Write(this.expandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedHL + Run.RESULTS_DELIMITER);
            output.Write(this.expandedLL + Run.RESULTS_DELIMITER);
            output.Write(this.generatedLL + Run.RESULTS_DELIMITER);
            output.Write(this.survivedPruningHL + Run.RESULTS_DELIMITER);
            output.Write(this.goalTestSkipped + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 6;
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
            this.accSurvivedPruningHL = 0;
            this.accGoalTestSkipped = 0;
        }

        public void AccumulateStatistics()
        {
            this.accExpandedHL += this.expandedHL;
            this.accGeneratedHL += this.generatedHL;
            this.accExpandedLL += this.expandedLL;
            this.accGeneratedLL += this.generatedLL;
            this.accSurvivedPruningHL += this.survivedPruningHL;
            this.accGoalTestSkipped += this.goalTestSkipped;
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, this.accExpandedHL);
            Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, this.accGeneratedHL);
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpandedLL);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGeneratedLL);
            Console.WriteLine("{0} Accumulated High-Level Nodes Survived Pruning: {1}", this, this.accSurvivedPruningHL);
            Console.WriteLine("{0} Accumulated High-Level Goal Tests Skipped: {1}", this, this.accGoalTestSkipped);

            output.Write(this.accExpandedHL + Run.RESULTS_DELIMITER);
            output.Write(this.accGeneratedHL + Run.RESULTS_DELIMITER);
            output.Write(this.accExpandedLL + Run.RESULTS_DELIMITER);
            output.Write(this.accGeneratedLL + Run.RESULTS_DELIMITER);
            output.Write(this.accSurvivedPruningHL + Run.RESULTS_DELIMITER);
            output.Write(this.accGoalTestSkipped + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Solves the instance that was set by a call to Setup()
        /// </summary>
        /// <returns></returns>
        public abstract bool Solve();

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
        public int GetMaxGroupSize() { return problem.agents.Length; }
        public SinglePlan[] GetSinglePlans() { return solution; }

        public virtual int[] GetSingleCosts()
        {
            return costs;
        }

        public abstract CostTreeNodeSolver CreateNodeSolver(ProblemInstance instance, Run runner);
    }

    class CostTreeSearchSolverOldMatching : CostTreeSearchSolver
    {
        int syncSize;

        public CostTreeSearchSolverOldMatching(int syncSize) : base() { this.syncSize = syncSize; }

        public override CostTreeNodeSolver CreateNodeSolver(ProblemInstance instance, Run runner)
        {
            return new CostTreeNodeSolverOldMatching(problem, runner, this);
        }

        public override bool Solve()
        {
            CostTreeNodeSolver nodeSolver = CreateNodeSolver(this.problem, this.runner);
            SinglePlan[] ans = null;

            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                int sumSubGroupA = costTreeNode.Sum(0, this.sizeParentGroupA);
                int sumSubGroupB = costTreeNode.Sum(this.sizeParentGroupA, costTreeNode.costs.Length);

                //if we are above the given solution return no solution found
                if (sumSubGroupA + sumSubGroupB > maxCost)  // TODO: For makespan, using Max of the group "sums"
                    return this.minConflictsNotAvoided != int.MaxValue;  // If we're running exhaustive ICTS and have a solution and
                                                                          // reached a node with a higher cost, it means we've exhausted
                                                                          // all goal nodes and can return that a solution was found
                                                                          // (with some external conflicts)
                
                // Goal test, unless any subproblem hasn't reached its previous optimal cost or we're below the minimum total cost
                if (sumSubGroupA >= costParentGroupA && sumSubGroupB >= costParentGroupB &&
                    sumSubGroupA + sumSubGroupB >= minCost  // TODO: Support other cost functions
                    )
                {
                    ((CostTreeNodeSolverOldMatching)nodeSolver).Setup(costTreeNode, syncSize, this.reserved);
                    expandedHL++;
                    ans = nodeSolver.Solve(CAT);
                    generatedLL += nodeSolver.generated;
                    expandedLL += nodeSolver.expanded;
                    this.survivedPruningHL--;
                    if (ans != null && ans[0] != null)  // A solution was found!
                    {
                        if (this.exhaustive == false)
                        {
                            this.totalCost = nodeSolver.totalCost;
                            this.solutionDepth = nodeSolver.totalCost - this.initialEstimate;
                            this.solution = ans;
                            this.costs = costTreeNode.costs;
                            this.conflictCounts = nodeSolver.GetExternalConflictCounts();
                            this.conflictTimes = nodeSolver.GetConflictTimes();
                            return true;
                        }

                        if (nodeSolver.conflictsNotAvoided < this.minConflictsNotAvoided)
                        {
                            this.totalCost = nodeSolver.totalCost;
                            this.solutionDepth = nodeSolver.totalCost - this.initialEstimate;
                            this.solution = ans;
                            this.costs = costTreeNode.costs;
                            this.conflictCounts = nodeSolver.GetExternalConflictCounts();
                            this.conflictTimes = nodeSolver.GetConflictTimes();
                            this.minConflictsNotAvoided = nodeSolver.conflictsNotAvoided;
                            this.maxCost = totalCost;
                            if (nodeSolver.conflictsNotAvoided == 0)
                                return true;
                        }
                    }
                    else
                    {
                        // This would allow us to also prune nodes where a subproblem has the same cost as its optimal cost,
                        // but in a bad configuration
                        // make ID pass the group solver a persistance object it got from the previous unconstrained execution
                        //if (this.instance.parameters.ContainsKey(IndependenceDetection.ILLEGAL_MOVES_KEY) == false &&
                        //    this.instance.parameters.ContainsKey(CBS.CONSTRAINTS) == false)  // technically we could cache by the constraints etc. but nah
                        // persistance.Add(costTreeNode.costs.ToImmutableArray())
                    }
                }
                else
                    ++goalTestSkipped;

                costTreeNode.Expand(openList, closedList);
                generatedHL += costTreeNode.costs.Length;  // We generated a child per each individual cost, with that cost increased by 1
                openList.Dequeue();
            }

            this.totalCost = Constants.TIMEOUT_COST;
            this.solutionDepth = nodeSolver.totalCost - this.initialEstimate; // A lower bound
            Console.WriteLine("Out of time");
            return false; 
        }

        public override String GetName() 
        {
            if (exhaustive)
                return $"Exhaustive ICTS {syncSize}E";  // 3E is the variant from the ICTS journal paper
            else
                return $"ICTS {syncSize}E";  // 3E is the variant from the ICTS journal paper
        }
    }

    class CostTreeSearchSolverNoPruning : CostTreeSearchSolver
    {
        public override CostTreeNodeSolver CreateNodeSolver(ProblemInstance instance, Run runner)
        {
            return new CostTreeNodeSolverDDBF(problem, runner, this);
        }

        public override bool Solve()
        {
            CostTreeNodeSolver next = CreateNodeSolver(this.problem, this.runner);
            SinglePlan[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeParentGroupA);
                sumSubGroupB = costTreeNode.Sum(sizeParentGroupA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return this.minConflictsNotAvoided != int.MaxValue;
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costParentGroupA && sumSubGroupB >= costParentGroupB)
                {
                    next.Setup(costTreeNode, reserved);
                    expandedHL++;
                    ans = next.Solve(CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (this.exhaustive == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                return true;
                            }

                            if (next.conflictsNotAvoided < this.minConflictsNotAvoided)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                this.minConflictsNotAvoided = next.conflictsNotAvoided;
                                maxCost = totalCost;
                                if (next.conflictsNotAvoided == 0)
                                    return true;
                                survivedPruningHL--;
                            }
                        }
                    }
                }
                survivedPruningHL++;
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

    abstract class CostTreeSearchWithEdgesMatrix : CostTreeSearchSolver
    {
        // These variables are for matching and pruning MDDs
        public int[,,] edgesMatrix; // K(agent), V(from), V(to)
        public int edgesMatrixCounter;
    }

    class CostTreeSearchSolverKMatch : CostTreeSearchWithEdgesMatrix
    {
        int maxGroupChecked;

        public CostTreeSearchSolverKMatch(int maxGroupChecked) : base() { this.maxGroupChecked = maxGroupChecked; }
        public override void Setup(ProblemInstance problemInstance, Run runner) {
            base.Setup(problemInstance, 0, runner);
        }
        public override void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner,
                                  ConflictAvoidanceTable CAT,
                                  ISet<CbsConstraint> constraints = null, ISet<CbsConstraint> positiveConstraints = null,
                                  int minCost = -1, int maxCost = int.MaxValue, MDD mdd = null)
        {
            edgesMatrix = new int[problemInstance.agents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            base.Setup(problemInstance, minTimeStep, runner, CAT, constraints, positiveConstraints, minCost, maxCost, mdd);
        }

        public override CostTreeNodeSolver CreateNodeSolver(ProblemInstance instance, Run runner)
        {
            return new CostTreeNodeSolverKSimpleMatching(problem, runner, this);
        }

        public override bool Solve()
        {
            CostTreeNodeSolver next = CreateNodeSolver(problem, runner);
            SinglePlan[] ans = null;
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeParentGroupA);
                sumSubGroupB = costTreeNode.Sum(sizeParentGroupA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minConflictsNotAvoided != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costParentGroupA && sumSubGroupB >= costParentGroupB)
                {
                    ((CostTreeNodeSolverKSimpleMatching)next).Setup(costTreeNode, maxGroupChecked, reserved);
                    expandedHL++;
                    ans = next.Solve(CAT);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (this.exhaustive == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                survivedPruningHL--;
                                return true;
                            }

                            if (next.conflictsNotAvoided < this.minConflictsNotAvoided)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                survivedPruningHL--;
                                this.minConflictsNotAvoided = next.conflictsNotAvoided;
                                maxCost = totalCost;
                                if (next.conflictsNotAvoided == 0)
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

    class CostTreeSearchSolverRepeatedMatch : CostTreeSearchWithEdgesMatrix
    {
        int syncSize;
        public CostTreeSearchSolverRepeatedMatch(int syncSize) : base() { this.syncSize = syncSize; }
        public override void Setup(ProblemInstance problemInstance, Run runner) { Setup(problemInstance, 0, runner); }
        public override void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner,
                                   ConflictAvoidanceTable CAT = null,
                                   ISet<CbsConstraint> constraints = null, ISet<CbsConstraint> positiveConstraints = null,
                                   int minCost = -1, int maxCost = int.MaxValue, MDD mdd = null)
        {
            edgesMatrix = new int[problemInstance.agents.Length, problemInstance.GetMaxX() * problemInstance.GetMaxY() + problemInstance.GetMaxY(), Move.NUM_NON_DIAG_MOVES];
            edgesMatrixCounter = 0;
            base.Setup(problemInstance, minTimeStep, runner, CAT, constraints, positiveConstraints, minCost, maxCost, mdd);
        }

        public override CostTreeNodeSolver CreateNodeSolver(ProblemInstance instance, Run runner)
        {
            return new CostTreeNodeSolverRepeatedMatching(problem, runner, this);
        }

        public override bool Solve()
        {
            //int time = 0;
            CostTreeNodeSolver next = CreateNodeSolver(problem, runner);
            SinglePlan[] ans = null;
            Stopwatch sw = new Stopwatch();
            int sumSubGroupA;
            int sumSubGroupB;
            //TODO if no solution found the algorithm will never stop
            while (runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                sw.Reset();
                costTreeNode = openList.Peek();
                sumSubGroupA = costTreeNode.Sum(0, sizeParentGroupA);
                sumSubGroupB = costTreeNode.Sum(sizeParentGroupA, costTreeNode.costs.Length);

                if (maxCost != -1)
                {
                    //if we are above the given solution return no solution found
                    if (sumSubGroupA + sumSubGroupB > maxCost)
                        return (this.minConflictsNotAvoided != int.MaxValue);
                    //if we are below the given solution no need to do goal test just expand node
                    if (sumSubGroupA + sumSubGroupB < maxCost)
                    {
                        costTreeNode.Expand(openList, closedList);
                        openList.Dequeue();
                        continue;
                    }
                }

                // Reuse optimal solutions to previously solved subproblems
                if (sumSubGroupA >= costParentGroupA && sumSubGroupB >= costParentGroupB)
                {
                    ((CostTreeNodeSolverRepeatedMatching)next).setup(costTreeNode, syncSize, reserved);
                    generatedLL += next.generated;
                    expandedLL += next.expanded;
                    sw.Start();
                    ans = next.Solve(CAT);
                    sw.Stop();
                    Console.WriteLine(sw.ElapsedMilliseconds);
                    if (sw.ElapsedMilliseconds > 0)
                        Console.ReadLine();
                    generatedLL += next.getGenerated();
                    if (ans != null)
                    {
                        if (ans[0] != null)
                        {
                            if (this.exhaustive == false)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                survivedPruningHL--;
                                return true;
                            }

                            if (next.conflictsNotAvoided < this.minConflictsNotAvoided)
                            {
                                totalCost = next.totalCost;
                                solution = ans;
                                costs = costTreeNode.costs;
                                survivedPruningHL--;
                                this.minConflictsNotAvoided = next.conflictsNotAvoided;
                                maxCost = totalCost;
                                if (next.conflictsNotAvoided == 0)
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
