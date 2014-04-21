using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// This class is responsible for running the experiments.
    /// </summary>
    public class Run : IDisposable
    {
        ////////debug
        // public static TextWriter resultsWriterdd;
        /////////////

        /// <summary>
        /// Delimiter character used when writing the results of the runs to the output file.
        /// </summary>
        public static string RESULTS_DELIMITER = ",";

        /// <summary>
        /// Number of random steps performed when generating a new problem instance for choosing a start-goal pair.
        /// </summary>
        public static int RANDOM_WALK_STEPS = 100000;

        /// <summary>
        /// Indicates the starting time in ms for timing the different algorithms.
        /// </summary>
        private double startTime;

        /// <summary>
        /// This hold an open stream to the results file.
        /// </summary>
        private TextWriter resultsWriter;

        /// <summary>
        /// EH: I introduced this variable so that debugging and experiments
        /// can have deterministic results.
        /// </summary>
        static public Random rand = new Random();

        /// <summary>
        /// Calls resultsWriter.Dispose()
        /// </summary>
        protected virtual void Dispose(bool dispose_managed)
        {
            if (dispose_managed)
            {
                if (this.resultsWriter != null)
                {
                    this.resultsWriter.Dispose();
                    this.resultsWriter = null;
                }
            }
        }

        public void Dispose() {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Open the results file for output. Currently the file is opened in append mode.
        /// </summary>
        /// <param name="fileName">The name of the results file</param>
        public void openResultsFile(string fileName)
        {
            this.resultsWriter = new StreamWriter(fileName, true); // 2nd arguments indicate the "append" mode
        }

        /// <summary>
        /// Closes the results file.
        /// </summary>
        public void closeResultsFile()
        {
            this.resultsWriter.Close();
        }

        /// <summary>
        /// all types of algorithms to be run
        /// </summary>
        List<ISolver> solvers;

        /// <summary>
        /// Counts the number of times each algorithm went out of time consecutively
        /// </summary>
        public int[] outOfTimeCounter;

        /// <summary>
        /// Construct with chosen algorithms.
        /// </summary>
        public Run()
        {
            // Preparing the heuristics:
            var sic = new SumIndividualCosts();
            
            // Preparing the solvers:
            solvers = new List<ISolver>();
            solvers.Add(new ClassicAStar(sic)); // Assumed to be the first solver. Works
            solvers.Add(new CBS_LocalConflicts(new ClassicAStar(sic), -1, -1, sic)); // Works

         //   solvers.Add(new CostTreeSearchSolverNoPruning());
            //solvers.Add(new CostTreeSearchSolverKMatch(2));
            //solvers.Add(new CostTreeSearchSolverOldMatching(2));
            //solvers.Add(new CostTreeSearchSolverRepatedMatch(2));
            //solvers.Add(new CostTreeSearchSolverKMatch(3));
           // solvers.Add(new CostTreeSearchSolverOldMatching(3));
            //solvers.Add(new CostTreeSearchSolverRepatedMatch(3));

           // solvers.Add(new CostTreeSearchNoPruning());
            //solvers.Add(new CostTreeSearchKMatch(2));
            //solvers.Add(new CostTreeSearchOldMatching(2));
            //solvers.Add(new CostTreeSearchRepatedMatch(2));
            //solvers.Add(new CostTreeSearchKMatch(3));
    //        solvers.Add(new CostTreeSearchOldMatching(3));
            //solvers.Add(new CostTreeSearchRepatedMatch(3));


           //solvers.Add(new AStarWithPartialExpansionBasic());
           // solvers.Add(new AStarWithOD());
           // solvers.Add(new AStarWithPartialExpansion());

   //         solvers.Add(new Trevor(new AStarWithPartialExpansion()));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 1, 1)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 5, 5)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 10, 10)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 100, 100)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 500, 500)));
   //         solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar())));
            //solvers.Add(new CostTreeSearchOldMatching(3));

            //solvers.Add(new CBS_IDA(new ClassicAStar()));
           // solvers.Add(new AStarWithPartialExpansion());
          //  solvers.Add(new CBS_GlobalConflicts(new ClassicAStar()));

           // solvers.Add(new CBS_NoDD(new ClassicAStar()));
            //solvers.Add(new CBS_NoDDb3(new ClassicAStar()));

           // solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar(), 1, 1)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar(), 5, 5)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar(), 10, 10)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar(), 100, 100)));
            //solvers.Add(new Trevor(new CBS_GlobalConflicts(new ClassicAStar(), 500, 500)));

            //solvers.Add(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 1, 1));
            //solvers.Add(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 5, 5));
            //solvers.Add(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 10, 10));
            //solvers.Add(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 100, 100));
           // solvers.Add(new CBS_GlobalConflicts(new AStarWithPartialExpansion(), 500, 500));
           // solvers.Add(new CBS_GlobalConflicts(new ClassicAStar()));
            //solvers.Add(new CBS_LocalConflicts(new ClassicAStar(), 0, 0));
            //solvers.Add(new CBS_LocalConflicts(new ClassicAStar(), 1, 0));
            //solvers.Add(new CBS_LocalConflicts(new ClassicAStar(), 2, 0));

           // solvers.Add(new Trevor(new AStarWithPartialExpansionBasic()));
           // solvers.Add(new Trevor(new AStarWithPartialExpansion()));
           // solvers.Add(new Trevor(new ClassicAStar()));
           // solvers.Add(new Trevor());

            outOfTimeCounter = new int[solvers.Count];
            for (int i = 0; i < outOfTimeCounter.Length; i++)
            {
                outOfTimeCounter[i] = 0;
            }
        }

        /// <summary>
        /// Generates a problem instance, including a board, start and goal locations of desired number of agents
        /// and desired precentage of obstacles
        ///  TODO: Refactor to use operators.
        /// </summary>
        /// <param name="gridSize"></param>
        /// <param name="agentsNum"></param>
        /// <param name="obstaclesNum"></param>
        /// <returns></returns>
        public ProblemInstance generateProblemInstance(int gridSize, int agentsNum, int obstaclesNum)
        {
            /**
             * Randomization based on timer is disabled for purposes of getting
             * reproducible experiments.
             */
            //Random rand = new Random();

            int x;
            int y;
            Agent[] aGoals = new Agent[agentsNum];
            AgentState[] aStart = new AgentState[agentsNum];
            bool[][] grid = new bool[gridSize][];
            bool[][] goals = new bool[gridSize][];

            // Generate a random grid
            for (int i = 0; i < gridSize; i++)
            {
                grid[i] = new bool[gridSize];
                goals[i] = new bool[gridSize];
            }
            for (int i = 0; i < obstaclesNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (grid[x][y])
                    i--;
                grid[x][y] = true;
            }

            // Choose random goal locations
            for (int i = 0; i < agentsNum; i++)
            {
                x = rand.Next(gridSize);
                y = rand.Next(gridSize);
                if (goals[x][y] || grid[x][y])
                    i--;
                else
                {
                    goals[x][y] = true;
                    aGoals[i] = new Agent(x, y, i);
                }
            }

            // Select random start/goal locations for every agent by performing a random walk
            for (int i = 0; i < agentsNum; i++)
            {
                aStart[i] = new AgentState(aGoals[i].Goal.x, aGoals[i].Goal.y, aGoals[i]);
            }

            // Initialzied here only for the IsValid() call. TODO: Think how this can be sidestepped elegantly.
            ProblemInstance problem = new ProblemInstance();
            problem.init(aStart, grid);
            
            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[aStart[i].last_move.x][aStart[i].last_move.y] = false; // We're going to move the goal somewhere else
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        aStart[i].last_move.Update(op);
                        if (problem.IsValid(aStart[i].last_move) &&
                            !goals[aStart[i].last_move.x][aStart[i].last_move.y]) // this spot isn't another agent's goal
                            break;
                        else
                            aStart[i].last_move.setOppositeMove(); // Rollback
                    }
                    goals[aStart[i].last_move.x][aStart[i].last_move.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in aStart) 
            {
                agentStart.last_move.time = 0;
            }

            // TODO: There is some repetition here of previous instantiation of ProblemInstance. Think how to elegantly bypass this.
            problem = new ProblemInstance();
            problem.init(aStart, grid);
            return problem;            
        }

        /// <summary>
        /// Solve given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to solve</param>
        public void solveGivenProblem(ProblemInstance instance)
        {
            // Preparing a list of agent indices (not agent nums) for the heuristics' init() method
            List<uint> agentList = Enumerable.Range(0, instance.m_vAgents.Length).Select<int, uint>(x=> (uint)x).ToList<uint>(); // FIXME: Must the heuristics really receive a list of uints?
            
            // Solve using the different algorithms
            Debug.WriteLine("Solving instance " + instance);
            int gridSize = instance.m_vGrid.Length;
            this.printProblemStatistics(instance);
            //double cr0 = instance.getConflictRation(0);
            //double cr1 = instance.getConflictRation(1);

            //Debug.WriteLine("Conflict ratio (first order): " + cr0);
            //Debug.WriteLine("Conflict ratio (second order): " + cr1);
            //this.resultsWriter.Write(cr0 + RESULTS_DELIMITER);
            //this.resultsWriter.Write(cr1 + RESULTS_DELIMITER);

            for (int i = 0; i < solvers.Count; i++)
            {
                if (outOfTimeCounter[i] < Constants.MAX_FAIL_COUNT) // After "MAX_FAIL_COUNT" consecutive failures of a given algorithm we stop running it.
                                                                    // Assuming problem difficulties are non-decreasing, if it consistently failed on several problems it won't suddenly succeed in solving the next problem.
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    solvers[i].GetHeuristic().init(instance, agentList);
                    this.run(solvers[i], instance);


                    Console.WriteLine();
                    if (solvers[i].GetSolutionCost() >= 0)
                    {
                        solvers[i].GetPlan().PrintPlan();
                        outOfTimeCounter[i] = 0;

                        // Validate solution:
                        if (solvers[0].GetSolutionCost() >= 0)
                        {
                            Debug.Assert(solvers[0].GetSolutionCost() == solvers[i].GetSolutionCost(), "A* solution cost is different than that of " + solvers[i]); // Assuming algs are supposed to find an optimal solution, this is an error.
                            //Debug.Assert(solvers[0].getExpanded() == solvers[i].getExpanded(), "Different Expanded");
                            //Debug.Assert(solvers[0].getGenerated() == solvers[i].getGenerated(), "Different Generated");
                            //Debug.Assert(solvers[0].GetSolutionDepth() == solvers[i].GetSolutionDepth(), "Depth Bug " + solvers[i]);

                            //if (solvers[0].GetSolutionCost() != solvers[i].GetSolutionCost())
                            //{
                            //    Console.WriteLine("A* solution cost is different than that of " + solvers[i]);
                            //    Console.ReadLine();
                            //}
                        }

                        Console.WriteLine("+SUCCESS+ (:");
                    }
                    else
                    {
                        outOfTimeCounter[i]++;
                        Console.WriteLine("-FAILURE- ):");
                    }
                }
                else
                    printNullStatistics();
                
                Console.WriteLine();
            }
            this.continueToNextLine();
          //Debug.Assert(solvers[0].GetNodesPassedPruningCounter() >= solvers[1].GetNodesPassedPruningCounter(), "");
          //Console.ReadLine();
        }

        /// <summary>
        /// Solve a given instance with the given solver
        /// </summary>
        /// <param name="solver">The solver</param>
        /// <param name="instance">The problem instance that will be solved</param>
        private void run(ISolver solver, ProblemInstance instance)
        {
            // Run the algorithm
            bool solved;
            Console.WriteLine("-----------------" + solver + "-----------------");
            this.startTime = this.ElapsedMillisecondsTotal();
            solver.Setup(instance, this);
            solved = solver.Solve();
            double elapsedTime = this.ElapsedMilliseconds();
            if (solved)
            {
                Console.WriteLine("Total cost: {0}", solver.GetSolutionCost());
            }
            else
            {
                Console.WriteLine("Failed to solve");
            }
            Console.WriteLine();
            Console.WriteLine("Time In milliseconds: {0}", elapsedTime);
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", solver.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", solver.GetHighLevelGenerated());
            Console.WriteLine("Total Expanded Nodes (Low-Level): {0}", solver.GetLowLevelExpanded());
            Console.WriteLine("Total Generated Nodes (Low-Level): {0}", solver.GetLowLevelGenerated());
           // Console.WriteLine("Total Unique/Full Expanded Nodes: {0}", solver.GetNodesPassedPruningCounter());

            this.printStatistics(instance, solver, elapsedTime);
            solver.Clear();
        }

        /// <summary>
        /// Print the header of the results file
        /// </summary>
        public void printResultsFileHeader()
        {
            this.resultsWriter.Write("GridSize");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("NumOfAgents");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("NumOfObstacles");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("InstanceId");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            //this.resultsWriter.Write("Conflict1");
            //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            //this.resultsWriter.Write("Conflict2");
            //this.resultsWriter.Write(Run.RESULTS_DELIMITER);

            for (int i = 0; i < solvers.Count; i++)
            {
                var name = solvers[i];
                this.resultsWriter.Write(name + " success");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Runtime");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " SolutionCost");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Expanded-HL");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Generated-HL");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Expanded-LL");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Generated-LL");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(name + " Max Group");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);


                //this.resultsWriter.Write(name + "Min Group / G&D");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Max depth");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Passed Nodes/Expanded Full States");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Memory Used");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            }
            this.resultsWriter.WriteLine();
        }

        /// <summary>
        /// Print the solver statistics to the results file.
        /// </summary>
        /// <param name="instance">The problem instance that was solved</param>
        /// <param name="solver">The solver that solved the problem instance</param>
        /// <param name="runtimeInMillis">The time it took the given solver to solve the given instance</param>
        private void printStatistics(ProblemInstance instance, ISolver solver, double runtimeInMillis)
        {
            if (solver.GetSolutionCost() < 0)
            {
                this.resultsWriter.Write(0 + RESULTS_DELIMITER);
            }
            else
            {
                this.resultsWriter.Write(1 + RESULTS_DELIMITER);
            }

            this.resultsWriter.Write(runtimeInMillis + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetSolutionCost() + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetHighLevelExpanded() + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetHighLevelGenerated() + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetLowLevelExpanded() + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetLowLevelGenerated() + RESULTS_DELIMITER);
            this.resultsWriter.Write(solver.GetMaxGroupSize() + RESULTS_DELIMITER);

            //solver.OutputStatistics(this.resultsWriter);
        }

        private void printProblemStatistics(ProblemInstance instance)
        {
            this.resultsWriter.Write(instance.m_vGrid.GetLength(0) + RESULTS_DELIMITER);
            this.resultsWriter.Write(instance.m_vAgents.Length + RESULTS_DELIMITER);
            this.resultsWriter.Write(instance.m_nObstacles + RESULTS_DELIMITER);
            this.resultsWriter.Write(instance.instanceId + RESULTS_DELIMITER);
        }
        private void continueToNextLine()
        {
            this.resultsWriter.WriteLine();
            this.resultsWriter.Flush();
        }

        private void printNullStatistics()
        {
            this.resultsWriter.Write(0 + RESULTS_DELIMITER);
            this.resultsWriter.Write(Constants.MAX_TIME + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
        }
        
        public void resetOutOfTimeCounter()
        {
            for (int i = 0; i < outOfTimeCounter.Length; i++)
            {
                outOfTimeCounter[i] = 0;
            }
        }

        //public void setOutOfTimeCounter()
        //{
        //    Console.WriteLine("Enter Fail Count Out Of Total - " + Constants.MAX_FAIL_COUNT);
        //    Console.WriteLine("---------------------------------------");
        //    for (int i = 0; i < solvers.Count; i++)
        //    {
        //        Console.WriteLine("Enter Fail Count For " + solvers[i]);
        //        this.outOfTimeCounter[i] = Int16.Parse(Console.ReadLine());
        //    }
        //}

        private double ElapsedMillisecondsTotal()
        {
            TimeSpan interval = Process.GetCurrentProcess().TotalProcessorTime;
            return interval.TotalMilliseconds;
        }

        public double ElapsedMilliseconds()
        {
            return ElapsedMillisecondsTotal() - this.startTime;
        }
    }
}
