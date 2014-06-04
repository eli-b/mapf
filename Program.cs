using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
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
            Run runner = new Run();
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            runner.PrintResultsFileHeader();
            ProblemInstance instance = runner.GenerateProblemInstance(10, 3, 10);
            instance.Export("Test.instance");
            runner.SolveGivenProblem(instance);            
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a single instance, imported from a given filename.
        /// </summary>
        /// <param name="fileName"></param>
        public void RunInstance(string fileName)
        {
            Run runner = new Run();
            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();
            ProblemInstance instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + fileName);
            runner.SolveGivenProblem(instance);
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a set of experiments.
        /// This function will generate a random instance (or load it from a file if it was already generated)
        /// </summary>
        public void RunExperimentSet(int[] gridSizes, int[] agentListSizes, int[] obstaclesProbs, int instances)
        {
            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();

            bool continueFromLastRun = false; 
            string[] LastProblemDetails = null;
            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                var lastProblemFile = new StreamReader(currentProblemFileName);
                LastProblemDetails = lastProblemFile.ReadLine().Split(',');  //get the last problem
                lastProblemFile.Close();
                continueFromLastRun = true;
            }
            
            for (int gs = 0; gs < gridSizes.Length; gs++)
            {
                for (int obs = 0; obs < obstaclesProbs.Length; obs++)
                {
                    runner.ResetOutOfTimeCounters();
                    for (int ag = 0; ag < agentListSizes.Length; ag++)
                    {
                        if (gridSizes[gs] * gridSizes[gs] * (1 - obstaclesProbs[obs] / 100) < agentListSizes[ag]) // Probably not enough room for all agents
                            continue;
                        for (int i = 0; i < instances; i++)
                        {
                            if (continueFromLastRun)  //set the latest problem
                            {
                                gs = int.Parse(LastProblemDetails[0]);
                                obs = int.Parse(LastProblemDetails[1]);
                                ag = int.Parse(LastProblemDetails[2]);
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
                            instanceName = "Instance-" + gridSizes[gs] + "-" + obstaclesProbs[obs] + "-" + agentListSizes[ag] + "-" + i;
                            try
                            {
                                instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                                instance.instanceId = i;
                            }
                            catch (Exception importException)
                            {
                                if (onlyReadInstances)
                                {
                                    Console.WriteLine("File " + instanceName + "  dosen't exist");
                                    return;
                                }

                                instance = runner.GenerateProblemInstance(gridSizes[gs], agentListSizes[ag], obstaclesProbs[obs] * gridSizes[gs] * gridSizes[gs] / 100);
                                instance.ComputeSingleAgentShortestPaths();
                                instance.instanceId = i;
                                instance.Export("Instance-" + gridSizes[gs] + "-" + obstaclesProbs[obs] + "-" + agentListSizes[ag] + "-" + i);
                            }

                            runner.SolveGivenProblem(instance);

                            // Save the latest problem
                            if (File.Exists(currentProblemFileName))
                                File.Delete(currentProblemFileName);
                            var lastProblemFile = new StreamWriter(currentProblemFileName);
                            lastProblemFile.Write("{0},{1},{2},{3}", gs, obs, ag, i);
                            for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                            {
                                lastProblemFile.Write("," + runner.outOfTimeCounters[j]);
                            }
                            lastProblemFile.Close();
                        }
                    }
                }
            }
            runner.CloseResultsFile();                    
        }
       
        
        /// <summary>
        /// Dragon Age experiment
        /// </summary>
        /// <param name="instances"></param>
        public void RunDragonAgeExperimentSet(int instances)
        {
            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();
            // FIXME: Code dup with RunExperimentSet

            TextWriter output;
            int[] agentListSizes = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 250, 300};

            bool continueFromLastRun = false;
            string[] lineParts = null;

            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                TextReader input = new StreamReader(currentProblemFileName);
                lineParts = input.ReadLine().Split(',');  //get the last problem
                input.Close();
                continueFromLastRun = true;
            }

            for (int ag = 0; ag < agentListSizes.Length; ag++)
            {
                for (int i = 0; i < instances; i++)
                {
                    if (continueFromLastRun)//set the latest problem
                    {
                        ag = int.Parse(lineParts[0]);
                        i = int.Parse(lineParts[1]);
                        for (int j = 2; j < lineParts.Length; j++)
                        {
                            runner.outOfTimeCounters[j - 2] = int.Parse(lineParts[j]);
                        }
                        continueFromLastRun = false;
                        continue;
                    }
                    if (runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * 20) // All algs should be skipped
                        break;
                    instanceName = agentListSizes[ag]+" agents i-"+i;
                    try
                    {
                        instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                    }
                    catch (Exception importException)
                    {
                        throw new Exception("missing map " + instanceName);
                    }

                    runner.SolveGivenProblem(instance);

                    //save the latest problem
                    File.Delete(Directory.GetCurrentDirectory() + "\\Instances\\current problem");
                    output = new StreamWriter(Directory.GetCurrentDirectory() + "\\Instances\\current problem");
                    output.WriteLine("{0},{1}",ag, i);
                    for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                    {
                        output.Write("," + runner.outOfTimeCounters[j]);
                    }
                    output.Close();
                }
            }
            runner.CloseResultsFile();
        }

        /// <summary>
        /// This is the starting point of the program. 
        /// </summary>
        static void Main(string[] args)
        {
            Program me = new Program();
            Program.RESULTS_FILE_NAME = Process.GetCurrentProcess().ProcessName + ".csv";
            TextWriterTraceListener tr1 = new TextWriterTraceListener(System.Console.Out);
            Debug.Listeners.Add(tr1);
            //if (System.Diagnostics.Debugger.IsAttached)
            //{
            //    int maxTime = Constants.MAX_TIME;
            //    Constants.MAX_TIME = int.MaxValue;
            //    Console.WriteLine("Use Time Limit? <(ENTER)-no,(1)-yes>");
            //    if (Console.ReadLine() == "1")
            //        Constants.MAX_TIME = maxTime;
            //}

            if (Directory.Exists(Directory.GetCurrentDirectory() + "\\Instances") == false)
            {
                Directory.CreateDirectory(Directory.GetCurrentDirectory() + "\\Instances");
            }

            Program.onlyReadInstances = false;

            int instances = 100;

            //int[] gridSizes = new int[] { 3 };
            //Constants.MAX_TIME = 2400000;
            //int[] agentListSizes = new int[] { 2, 3, 4, 5, 6, 7, 8 };

            int[] gridSizes = new int[] { 8 };
            int[] agentListSizes = new int[] { 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 };
            //int[] agentListSizes = new int[] { 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50 };

            //int[] gridSizes = new int[] { 32 };
           // int[] agentListSizes = new int[] { 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150 };
            //int[] agentListSizes = new int[] { 20 };

           // int[] agentListSizes = new int[] { 4 };
            //int[] obstaclesProbs = new int[] { 0, 5, 10, 15, 20, 25, 30, 35 };

           // int[] obstaclesProbs = new int[] { 0,10,20,30 };

            int[] obstaclesProbs = new int[] { 0};
            me.RunExperimentSet(gridSizes,agentListSizes,obstaclesProbs,instances);

           //me.RunDragonAgeExperimentSet(instances);


            // A function for running a single instance that is loaded from this file.
            // me.RunInstance("Instance-3-0-4-829");
            // A function to be used by Eric's PDB code
            //me.runForPdb();
            Console.WriteLine("*********************THE END**************************");
            Console.ReadLine();
        }    
    }
}
