using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.IO;

namespace mapf
{
    /// <summary>
    /// This is the entry point of the application. 
    /// </summary>
    class Program
    {
        private static string RESULTS_FILE_NAME = "Results.csv"; // Overridden by Main
        private static bool onlyReadInstances = false;

        /// <summary>
        /// Simplest run possible with a randomly generated problem instance.
        /// </summary>
        public void SimpleRun()
        {
            using (Run runner = new Run())
            {
                runner.OpenResultsFile(RESULTS_FILE_NAME);
                runner.PrintResultsFileHeader();
                ProblemInstance instance = runner.GenerateProblemInstance(10, 3, 10);
                instance.Export("Test.instance");
                runner.SolveGivenProblem(instance);
            }
        }

        /// <summary>
        /// Runs a single instance, imported from a given filename.
        /// </summary>
        /// <param name="fileName"></param>
        public void RunInstance(string fileName)
        {
            ProblemInstance instance;
            try
            {
                instance = ProblemInstance.Import(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "Instances", fileName));
            }
            catch (Exception e)
            {
                Console.WriteLine($"Skipping bad problem instance {fileName}. Error: {e.Message}");
                Console.WriteLine(e.StackTrace);
                return;
            }

            Run runner = new Run();
            using (runner)
            {
                bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
                runner.OpenResultsFile(RESULTS_FILE_NAME);
                if (resultsFileExisted == false)
                    runner.PrintResultsFileHeader();
                runner.SolveGivenProblem(instance);
            }
        }

        /// <summary>
        /// Runs a set of experiments.
        /// This function will generate a random instance (or load it from a file if it was already generated)
        /// </summary>
        public void RunExperimentSet(int[] gridSizes, int[] agentListSizes, int[] obstaclesProbs, int instances)
        {
            ProblemInstance instance;
            string instanceName;
            using (Run runner = new Run())
            {
                bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
                runner.OpenResultsFile(RESULTS_FILE_NAME);
                if (resultsFileExisted == false)
                    runner.PrintResultsFileHeader();

                bool continueFromLastRun = false;
                string[] LastProblemDetails = null;
                string currentProblemFileName = Path.Combine(
                    Directory.GetCurrentDirectory(), $"current-problem-{Process.GetCurrentProcess().ProcessName}");
                if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
                {
                    var lastProblemFile = new StreamReader(currentProblemFileName);
                    LastProblemDetails = lastProblemFile.ReadLine().Split(',');  //get the last problem
                    lastProblemFile.Close();
                    continueFromLastRun = true;
                }

                string allocation = Process.GetCurrentProcess().ProcessName.Substring(1);  // When executable names are of the form "g###"

                for (int gridSizeIndex = 0; gridSizeIndex < gridSizes.Length; gridSizeIndex++)
                {
                    for (int obstaclePercentageIndex = 0; obstaclePercentageIndex < obstaclesProbs.Length; obstaclePercentageIndex++)
                    {
                        runner.ResetOutOfTimeCounters();
                        for (int numOfAgentsIndex = 0; numOfAgentsIndex < agentListSizes.Length; numOfAgentsIndex++)
                        {
                            if (gridSizes[gridSizeIndex] * gridSizes[gridSizeIndex] * (1 - obstaclesProbs[obstaclePercentageIndex] / 100) < agentListSizes[numOfAgentsIndex]) // Probably not enough room for all agents
                                continue;
                            for (int i = 0; i < instances; i++)
                            {
                                //if (i % 33 != Convert.ToInt32(allocation)) // grids!
                                //    continue;

                                //if (i % 5 != 0) // grids!
                                //    continue;

                                if (continueFromLastRun)  //set the latest problem
                                {
                                    gridSizeIndex = int.Parse(LastProblemDetails[0]);
                                    obstaclePercentageIndex = int.Parse(LastProblemDetails[1]);
                                    numOfAgentsIndex = int.Parse(LastProblemDetails[2]);
                                    i = int.Parse(LastProblemDetails[3]);
                                    for (int j = 4; j < LastProblemDetails.Length; j++)
                                    {
                                        runner.outOfTimeCounters[j - 4] = int.Parse(LastProblemDetails[j]);
                                    }
                                    continueFromLastRun = false;
                                    continue; // "current problem" file describes last solved problem, no need to solve it again
                                }
                                if (runner.outOfTimeCounters.Length != 0 &&
                                    runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * Constants.MAX_FAIL_COUNT) // All algs should be skipped
                                    break;
                                instanceName = $"Instance-{gridSizes[gridSizeIndex]}-{obstaclesProbs[obstaclePercentageIndex]}-{agentListSizes[numOfAgentsIndex]}-{i}";
                                try
                                {
                                    instance = ProblemInstance.Import(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "Instances", instanceName));
                                    instance.instanceId = i;
                                }
                                catch (Exception importException)
                                {
                                    if (onlyReadInstances)
                                    {
                                        Console.WriteLine($"File {instanceName}  dosen't exist");
                                        return;
                                    }

                                    instance = runner.GenerateProblemInstance(gridSizes[gridSizeIndex], agentListSizes[numOfAgentsIndex], obstaclesProbs[obstaclePercentageIndex] * gridSizes[gridSizeIndex] * gridSizes[gridSizeIndex] / 100);
                                    instance.ComputeSingleAgentShortestPaths(); // REMOVE FOR GENERATOR
                                    instance.instanceId = i;
                                    instance.Export(instanceName);
                                }

                                runner.SolveGivenProblem(instance);

                                // Save the latest problem
                                StreamWriter lastProblemFile;
                                try
                                {
                                    lastProblemFile = new StreamWriter(currentProblemFileName);
                                }
                                catch (Exception)
                                {
                                    System.Threading.Thread.Sleep(1000);
                                    lastProblemFile = new StreamWriter(currentProblemFileName);
                                }
                                lastProblemFile.Write("{0},{1},{2},{3}", gridSizeIndex, obstaclePercentageIndex, numOfAgentsIndex, i);
                                for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                                {
                                    lastProblemFile.Write($",{runner.outOfTimeCounters[j]}");
                                }
                                lastProblemFile.Close();
                                lastProblemFile = null;
                            }
                        }
                    }
                }
            }
        }

        protected static readonly string[] daoMapPaths = {
            Path.Combine("..", "..", "..", "maps", "den520d.map"),
            Path.Combine("..", "..", "..", "maps", "ost003d.map"),
            Path.Combine("..", "..", "..", "maps", "brc202d.map")
        };

        protected static readonly string[] mazeMapPaths = {
            Path.Combine("..", "..", "..", "maps", "maze512-1-6.map"),
            Path.Combine("..", "..", "..", "maps", "maze512-1-2.map"),
            Path.Combine("..", "..", "..", "maps", "maze512-1-9.map")
        };

        protected static readonly string[] scenDirs = {
            Path.Combine("..", "..", "..", "scen", "scen-even"),
            Path.Combine("..", "..", "..", "scen", "scen-random"),
            Path.Combine("..", "..", "..", "scen", "scen-omri")
        };

        /// <summary>
        /// Dragon Age experiment
        /// </summary>
        /// <param name="numInstances"></param>
        /// <param name="mapFilePaths"></param>
        public void RunDragonAgeExperimentSet(int numInstances, string[] mapFilePaths)
        {
            ProblemInstance instance;
            string instanceName;
            using (Run runner = new Run())
            {
                bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
                runner.OpenResultsFile(RESULTS_FILE_NAME);
                if (resultsFileExisted == false)
                    runner.PrintResultsFileHeader();
                // FIXME: Code dup with RunExperimentSet

                TextWriter output;
                int[] agentListSizes = { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
                //int[] agentListSizes = { 60, 65, 70, 75, 80, 85, 90, 95, 100 };
                //int[] agentListSizes = { 100 };

                bool continueFromLastRun = false;
                string[] lineParts = null;

                string currentProblemFileName = Path.Combine(
                    Directory.GetCurrentDirectory(), $"current-problem-{Process.GetCurrentProcess().ProcessName}"); 
                if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
                {
                    TextReader input = new StreamReader(currentProblemFileName);
                    lineParts = input.ReadLine().Split(',');  //get the last problem
                    input.Close();
                    continueFromLastRun = true;
                }

                for (int ag = 0; ag < agentListSizes.Length; ag++)
                {
                    for (int i = 0; i < numInstances; i++)
                    {
                        //string name = Process.GetCurrentProcess().ProcessName.Substring(1);
                        //if (i % 33 != Convert.ToInt32(name)) // DAO!
                        //    continue;

                        for (int map = 0; map < mapFilePaths.Length; map++)
                        {
                            if (continueFromLastRun) // Set the latest problem
                            {
                                ag = int.Parse(lineParts[0]);
                                i = int.Parse(lineParts[1]);
                                map = int.Parse(lineParts[2]);
                                for (int j = 3; j < lineParts.Length && j - 3 < runner.outOfTimeCounters.Length; j++)
                                {
                                    runner.outOfTimeCounters[j - 3] = int.Parse(lineParts[j]);
                                }
                                continueFromLastRun = false;
                                continue;  // We write the details of the last problem that was solved, no need to solve it again
                            }
                            if (runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * 20) // All algs should be skipped
                                break;
                            string mapFilePath = mapFilePaths[map];
                            instanceName = $"{Path.GetFileNameWithoutExtension(mapFilePath)}-{agentListSizes[ag]}-{i}";
                            try
                            {
                                instance = ProblemInstance.Import(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "Instances", instanceName));
                            }
                            catch (Exception importException)
                            {
                                if (onlyReadInstances)
                                {
                                    Console.WriteLine($"File {instanceName}  dosen't exist");
                                    return;
                                }

                                instance = runner.GenerateDragonAgeProblemInstance(mapFilePath, agentListSizes[ag]);
                                instance.ComputeSingleAgentShortestPaths(); // Consider just importing the generated problem after exporting it to remove the duplication of this line from Import()
                                instance.instanceId = i;
                                instance.Export(instanceName);
                            }

                            runner.SolveGivenProblem(instance);

                            //save the latest problem
                            try
                            {
                                output = new StreamWriter(currentProblemFileName);
                            }
                            catch (Exception e)
                            {
                                System.Threading.Thread.Sleep(1000);
                                output = new StreamWriter(currentProblemFileName);
                            }
                            output.Write("{0},{1},{2}", ag, i, map);
                            for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                            {
                                output.Write($",{runner.outOfTimeCounters[j]}");
                            }
                            output.Close();
                        }
                    }
                }
            }
        }

        /// <summary>
        /// This is the starting point of the program. 
        /// </summary>
        static void Main(string[] args)
        {
            Program me = new Program();
            Program.RESULTS_FILE_NAME = Process.GetCurrentProcess().ProcessName + ".csv";
            if (System.Diagnostics.Debugger.IsAttached)
            {
                Constants.MAX_TIME = int.MaxValue;
                Debug.WriteLine("Debugger attached - running without a timeout!!");
            }

            if (Directory.Exists(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "Instances")) == false)
            {
                Directory.CreateDirectory(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "Instances"));
            }

            Program.onlyReadInstances = false;

            int instances = 100;

            bool runGrids = false;
            bool runDragonAge = false;
            bool runMazesWidth1 = false;
            bool runSpecific = false;
            bool runBenchmark = true;
            bool runBenchmarkIncrementally = false;  // Turn on for CA*

            if (runGrids == true)
            {
                //int[] gridSizes = new int[] { 8, };
                //int[] agentListSizes = new int[] { 2, 3, 4 };
                
                //int[] gridSizes = new int[] { 6, };
                //int[] agentListSizes = new int[] { /*2,*/ 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
                // Note that success rate drops almost to zero for EPEA* and A*+OD/SIC on 40 agents.
            
                int[] gridSizes = new int[] { 20, };
                //int[] agentListSizes = new int[] { 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, /*60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150*/ };
                int[] agentListSizes = new int[] { 40, 60, 80, 100 };

                //int[] obstaclesPercents = new int[] { 20, };
                //int[] obstaclesPercents = new int[] { /*0, 5, 10, 15, 20, 25, 30, 35, */20, 30, 40};
                int[] obstaclesPercents = new int[] { /*0, 5, 10,*/ 15, /*20, 25, 30, 35, 20, 30, 40*/ };
                me.RunExperimentSet(gridSizes, agentListSizes, obstaclesPercents, instances);
            }
            else if (runDragonAge == true)
                me.RunDragonAgeExperimentSet(instances, Program.daoMapPaths); // Obstacle percents and grid sizes built-in to the maps.
            else if (runMazesWidth1 == true)
                me.RunDragonAgeExperimentSet(instances, Program.mazeMapPaths); // Obstacle percents and grid sizes built-in to the maps.
            else if (runSpecific == true)
            {
                ProblemInstance instance;
                try
                {
                    if (args[0].EndsWith(".dll"))
                        instance = ProblemInstance.Import(args[2], args[1]);
                    else
                        instance = ProblemInstance.Import(args[1], args[0]);
                }
                catch (Exception e)
                {
                    Console.WriteLine($"Bad problem instance {args[1]}. Error: {e.Message}");
                    Console.WriteLine(e.StackTrace);
                    return;
                }
                Run runner = new Run();  // instantiates stuff unnecessarily
                runner.startTime = runner.ElapsedMillisecondsTotal();
                
                IHeuristicCalculator<WorldState> lowLevelHeuristic = new SumIndividualCosts();
                List<uint> agentList = Enumerable.Range(0, instance.agents.Length).Select(x=> (uint)x).ToList(); // FIXME: Must the heuristics really receive a list of uints?
                lowLevelHeuristic.Init(instance, agentList);
                ICbsSolver lowLevel = new A_Star(lowLevelHeuristic);
                ILazyHeuristic<CbsNode> highLevelHeuristic = new MvcHeuristicForCbs();
                highLevelHeuristic.Init(instance, agentList);
//                ISolver solver = new CBS(lowLevel, lowLevel,
//                    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
//                    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD,
//                    heuristic: highLevelHeuristic,
//                    cacheMdds: true,
//                    useOldCost: true,
//                    replanSameCostWithMdd: true
//                );
                //ISolver solver = new IndependenceDetection(lowLevel, new EPEA_Star(lowLevelHeuristic));
                //ISolver solver = new IndependenceDetection(lowLevel, new CostTreeSearchSolverOldMatching(3));
                ISolver solver = new IndependenceDetection(lowLevel, new A_Star_WithOD(lowLevelHeuristic));
                solver.Setup(instance, runner);
                bool solved = solver.Solve();
                if (solved == false)
                {
                    Console.WriteLine("Failed to solve");
                    return;
                }
                Plan plan = solver.GetPlan();
                plan.PrintPlan();
                //me.RunInstance("Instance-5-15-3-792");
                //me.RunInstance("Instance-5-15-3-792-4rows");
                //me.RunInstance("Instance-5-15-3-792-3rows");
                //me.RunInstance("Instance-5-15-3-792-2rows");
                //me.RunInstance("corridor1");
                //me.RunInstance("corridor2");
                //me.RunInstance("corridor3");
                //me.RunInstance("corridor4");
                return;
            }
            else if (runBenchmark)
            {
                Constants.MAX_FAIL_COUNT = 1;  // That's the way the benchmark is run
                foreach (var dirName in scenDirs)
                {
                    foreach (var scenPath in Directory.GetFiles(dirName))
                    {
                        ProblemInstance problem;
                        try
                        {
                            problem = ProblemInstance.Import(scenPath);
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine($"Bad problem instance {scenPath}. Error: {e.Message}");
                            Console.WriteLine(e.StackTrace);
                            return;
                        }

                        Run runner = new Run();

                        using (runner)
                        {
                            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
                            runner.OpenResultsFile(RESULTS_FILE_NAME);
                            if (resultsFileExisted == false)
                                runner.PrintResultsFileHeader();
                            foreach (var numAgents in Enumerable.Range(1, problem.agents.Length))
                            {
                                var subProblem = problem.Subproblem(problem.agents.Take(numAgents).ToArray());
                                bool success = runner.SolveGivenProblem(subProblem);
                                if (success == false)
                                    break;
                            }
                        }
                    }
                }
            }
            else if (runBenchmarkIncrementally)
            {
                Constants.MAX_FAIL_COUNT = 1;  // That's the way the benchmark is run
                foreach (var dirName in scenDirs)
                {
                    foreach (var scenPath in Directory.GetFiles(dirName))
                    {
                        ProblemInstance problem;
                        try
                        {
                            problem = ProblemInstance.Import(scenPath);
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine($"Bad problem instance {scenPath}. Error: {e.Message}");
                            Console.WriteLine(e.StackTrace);
                            return;
                        }
                        CooperativeAStar castar = new CooperativeAStar();
                        Run runner = new Run();
                        using (runner)
                        {
                            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
                            runner.OpenResultsFile(RESULTS_FILE_NAME);
                            if (resultsFileExisted == false)
                                runner.PrintResultsFileHeader();
                            runner.SolveGivenProblemIncrementally(problem);
                        }
                    }
                }
            }

            // A function to be used by Eric's PDB code
            //me.runForPdb();
            Console.WriteLine("*********************THE END**************************");
            Console.ReadLine();
        }    
    }
}
