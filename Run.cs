using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace mapf
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
        public static readonly string RESULTS_DELIMITER = ",";

        public static readonly int SUCCESS_CODE = 1;

        public static readonly int FAILURE_CODE = 0;

        /// <summary>
        /// Number of random steps performed when generating a new problem instance for choosing a start-goal pair.
        /// </summary>
        public static int RANDOM_WALK_STEPS = 100000;

        /// <summary>
        /// Indicates the starting time in ms for timing the different algorithms.
        /// </summary>
        public double startTime;

        /// <summary>
        /// This holds an open stream to the results file.
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
                    this.resultsWriter.Close();
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
        public void OpenResultsFile(string fileName)
        {
            this.resultsWriter = new StreamWriter(fileName, true); // 2nd argument indicates the "append" mode
        }

        /// <summary>
        /// Closes the results file.
        /// </summary>
        public void CloseResultsFile()
        {
            this.resultsWriter.Close();
        }

        /// <summary>
        /// All types of algorithms to be run
        /// </summary>
        List<ISolver> solvers;

        /// <summary>
        /// All types of A* heuristics used
        /// </summary>
        public List<IHeuristicCalculator<WorldState>> astar_heuristics; // FIXME: Make unpublic again later

        /// <summary>
        /// Counts the number of times each algorithm went out of time consecutively
        /// </summary>
        public int[] outOfTimeCounters;

        /// <summary>
        /// Construct with chosen algorithms.
        /// </summary>
        public Run()
        {
            this.watch = Stopwatch.StartNew();

            // Preparing the heuristics:
            astar_heuristics = new List<IHeuristicCalculator<WorldState>>();
            IHeuristicCalculator<WorldState> simple = null;
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
            {
                simple = new SumIndividualCosts();
            }
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
            {
                simple = new MaxIndividualCosts();
            }
            astar_heuristics.Add(simple);

            var cbs_heuristics = new List<IHeuristicCalculator<CbsNode>>();
            var mvc = new MvcHeuristicForCbs();
            cbs_heuristics.Add(mvc);
            var mddPruning = new MddPruningHeuristicForCbs();
            cbs_heuristics.Add(mddPruning);

            var astar = new A_Star(simple);
            var cbs = new CBS(astar, astar, -1);
            var astar_with_od = new A_Star_WithOD(simple);
            var epea = new EPEA_Star(simple);
            //var macbsLocal5Epea = new CBS(astar, epea, 5);
            //var macbsLocal50Epea = new CBS(astar, epea, 50);
            //var cbsHeuristicNoSolve1 = new CbsHeuristicForAStar(cbs, this, false, 1);
            //var cbsHeuristicNoSolve2 = new CbsHeuristicForAStar(cbs, this, false, 2);
            //var cbsHeuristicNoSolve3 = new CbsHeuristicForAStar(cbs, this, false, 3);
            //var cbsHeuristicNoSolve4 = new CbsHeuristicForAStar(cbs, this, false, 4);
            //var cbsHeuristicNoSolve5 = new CbsHeuristicForAStar(cbs, this, false, 5);
            //var cbsHeuristicNoSolve6 = new CbsHeuristicForAStar(cbs, this, false, 6);
            //var cbsHeuristicSolve1 = new CbsHeuristicForAStar(cbs, this, true, 1);
            //var cbsHeuristicSolve2 = new CbsHeuristicForAStar(cbs, this, true, 2);
            //var cbsHeuristicSolve3 = new CbsHeuristicForAStar(cbs, this, true, 3);
            //var cbsHeuristicSolve4 = new CbsHeuristicForAStar(cbs, this, true, 4);
            //var cbsHeuristicSolve5 = new CbsHeuristicForAStar(cbs, this, true, 5);
            //var cbsHeuristicSolve6 = new CbsHeuristicForAStar(cbs, this, true, 6);
            //var sicOrCbsh6 = new RandomChoiceOfHeuristic(cbsHeuristicSolve6, simple, 1.0 / 5);
            
            //var dynamicLazyCbsh = new DynamicLazyCbsh(cbs, this, true);
            //heuristics.Add(dynamicLazyCbsh);

            //var dynamicLazyMacbsLocal5EpeaH = new DynamicLazyCbsh(macbsLocal5Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal5EpeaH);

            //var dynamicLazyMacbsLocal50EpeaH = new DynamicLazyCbsh(macbsLocal50Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal50EpeaH);

            //var dynamicLazyMacbsLocal5EpeaHForOracleMustBeLast = new DynamicLazyCbsh(macbsLocal5Epea, this, true);
            //heuristics.Add(dynamicLazyMacbsLocal5EpeaHForOracleMustBeLast);

            // Preparing the solvers:
            solvers = new List<ISolver>();
            solvers.Add(new IndependenceDetection(astar, epea, IndependenceDetection.ConflictChoice.FIRST, true, ConflictAvoidanceTable.AvoidanceGoal.MINIMIZE_CONFLICTS)); // EPEA* + ID
            solvers.Add(new IndependenceDetection(astar, epea, IndependenceDetection.ConflictChoice.SMALLEST_RESULTING_GROUP, true, ConflictAvoidanceTable.AvoidanceGoal.MINIMIZE_LARGEST_CONFLICTING_GROUP_THEN_NUMBER_OF_SUCH_GROUPS)); // EPEA* + ID
            solvers.Add(new IndependenceDetection(astar, epea, IndependenceDetection.ConflictChoice.LARGEST_RESULTING_GROUP, true, ConflictAvoidanceTable.AvoidanceGoal.MINIMIZE_CONFLICTS)); // EPEA* + ID

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea)); // CBS/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(
            //    astar, epea, bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD)); // CBS/EPEA* with first-fit adoption 1 expansions max
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //            bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //            lookaheadMaxExpansions: 256)); // CBS/EPEA* with first-fit adoption 256 expansions max
            //solvers.Add(new MACBS_WholeTreeThreshold(
            //    astar, epea, bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    lookaheadMaxExpansions: int.MaxValue)); // CBS/EPEA* with first-fit adoption infinity expansions max

            // B is actually set according to the map in a hack elsewhere

            //IJCAI:

            //soldier: solvers.Add(new CBS(
            //     astar, epea conflictChoice: CBS.ConflictChoice.MOST_CONFLICTING)); // CBS/EPEA* + choosing most conflicting agent's conflict
            //solvers.Add(new IndependenceDetection(singleAgentSolver: astar,
            //    new MACBS_WholeTreeThreshold(
            //        singleAgentSolver: astar, generalSolver: epea,
            //        conflictChoice: CBS.ConflictChoice.MOST_CONFLICTING), simple)); // CBS/EPEA* + choosing most conflicting agent's conflict + ID
            //soldier: solvers.Add(new CBS(astar, epea,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD)); // CBS/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS(astar, epea,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_LOOKAHEAD)); // CBS/EPEA* Cardinal not using MDDs

            //soldier: 
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 2)); // MA-CBS(2)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 4)); // MA-CBS(4)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 8)); // MA-CBS(8)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 16)); // MA-CBS(16)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 32)); // MA-CBS(32)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 64)); // MA-CBS(64)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 50)); // MA-CBS(50)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 128)); // MA-CBS(128)/EPEA*
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 256)); // MA-CBS(256)/EPEA*

            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 5), simple)); // MA-CBS(5)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 10), simple)); // MA-CBS(10)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 25), simple)); // MA-CBS(25)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 100), simple)); // MA-CBS(100)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 150), simple)); // MA-CBS(150)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 200), simple)); // MA-CBS(200)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 300), simple)); // MA-CBS(300)/EPEA* + ID.
            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 500), simple)); // MA-CBS(500)/EPEA* + ID.

            //solvers.Add(new IndependenceDetection(astar,
            //    new MACBS_WholeTreeThreshold(astar, epea, 5, mergeCausesRestart: true),
            //    simple)); // MA-CBS(5)/EPEA* + ID + restart

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 5,
            //    conflictChoice: CBS.ConflictChoice.MOST_CONFLICTING)); // MA-CBS(5)/EPEA* + choosing most conflicting agent's conflict
            //solvers.Add(new CBS(astar, epea, 5,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS(astar, epea, 5,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_LOOKAHEAD)); // MA-CBS(5)/EPEA* Cardinal not using MDDs

            //soldier: solvers.Add(new CBS(astar, epea, 5, mergeCausesRestart: true)); // MA-CBS(5)/EPEA* + restart
            //solvers.Add(new CBS(astar, epea, 64, mergeCausesRestart: true)); // MA-CBS(64)/EPEA* + restart

            //soldier: solvers.Add(new CBS(astar, epea,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD)); // CBS/EPEA* + BP1
            //soldier:solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD)); // CBS/EPEA* + CARDINAL + BP1

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD)); // CBS + CARDINAL (lookahead) + BP1

            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD), simple)); // CBS + CARDINAL + BP1 + ID

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_LOOKAHEAD)); // CBS/EPEA* Cardinal not using MDDs + BP1

            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(
            //        astar, epea, 5, false, CBS.BypassStrategy.NONE, false,
            //        CBS.ConflictChoice.FIRST, false, false, int.MaxValue, true),
            //    simple)); // MA-CBS(B)/EPEA* + ID + restart

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 5, false, CBS.BypassStrategy.NONE, false,
            //    CBS.ConflictChoice.MOST_CONFLICTING, false, false, int.MaxValue, false)); // MA-CBS(5)/EPEA*
            //solvers.Add(new CBS(astar, epea, 5, false, CBS.BypassStrategy.NONE, false,
            //    CBS.ConflictChoice.CARDINAL_MDD, false, false)); // MA-CBS(5)/EPEA* Cardinal using MDDs
            //solvers.Add(new CBS(astar, epea, 5, false, CBS.BypassStrategy.NONE, false,
            //    CBS.ConflictChoice.CARDINAL_LOOKAHEAD, false, false)); // MA-CBS(5)/EPEA* Cardinal not using MDDs

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea)); // CBS/EPEA* 
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD)); // CBS/EPEA* + first cardinal
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD,
            //    hValueStrategy: CBS.HValueStrategy.GREEDY)); // CBS/EPEA* + greedy h + cardinal
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, heuristic: mvc)); // CBS/EPEA* + h + cardinal without BP
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 2,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(2)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 4,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(4)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 5,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(5)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 8,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(8)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 16,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(16)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 25,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(25)/EPEA* + cardinal + BP1 + restart, AKA ICBS(25)
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 32,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(32)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 50,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(50)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 64,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(64)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 128,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(128)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 256,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(256)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 512,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(512)/EPEA* + cardinal + BP1 + restart
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 1024,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(1024)/EPEA* + cardinal + BP1 + restart

            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 5
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(5)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 10, 
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(10)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 25,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(25)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 100,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 150,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 200,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 300,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(100)/EPEA* + cardinal + BP1 + restart + ID
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 500,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true), simple)); // MA-CBS(500)/EPEA* + cardinal + BP1 + restart + ID

            //solvers.Add(new CBS(astar, epea, 5, 
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    conflictChoice: CBS.ConflictChoice.CARDINAL_MDD, mergeCausesRestart: true)); // MA-CBS(5)/EPEA* Cardinal using MDDs + BP1 + restart

            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 5, 
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    lookaheadMaxExpansions: int.MaxValue)); // MA-CBS(5)/EPEA* with first-fit adoption infinity expansions max
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 5, 
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD), simple)); // MA-CBS(5)/EPEA* with first-fit adoption 1 expansions max + ID
            //solvers.Add(new MACBS_WholeTreeThreshold(astar, epea, 5,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    lookaheadMaxExpansions: 256)); // MA-CBS(5)/EPEA* with first-fit adoption 256 expansions max
            //solvers.Add(new IndependenceDetection(astar, new MACBS_WholeTreeThreshold(astar, epea, 5,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    lookaheadMaxExpansions: int.MaxValue), simple)); // MA-CBS(5)/EPEA* with first-fit adoption infinity expansions max + ID

            //solvers.Add(epea); // EPEA*
            //solvers.Add(new CostTreeSearchSolverOldMatching(3)); // ICTS
            //solvers.Add(new IndependenceDetection(astar, epea, simple)); // EPEA* + ID
            //solvers.Add(new IndependenceDetection(astar, new CostTreeSearchSolverOldMatching(3), simple)); // ICTS + ID

            /*
            solvers.Add(new CBS(astar, epea, 5)); // MACBS(5)/EPEA* - Works and is very fast so is a good choice for cost validation
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 1)); // MACBS(5)/EPEA* + adoption immediately
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: int.MaxValue)); // MACBS(5)/EPEA* + adoption immediately infinite expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 2)); // MACBS(5)/EPEA* + adoption immediately 2 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 4)); // MACBS(5)/EPEA* + adoption immediately 4 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 8)); // MACBS(5)/EPEA* + adoption immediately 8 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 16)); // MACBS(5)/EPEA* + adoption immediately 16 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 32)); // MACBS(5)/EPEA* + adoption immediately 32 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 64)); // MACBS(5)/EPEA* + adoption immediately 64 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 128)); // MACBS(5)/EPEA* + adoption immediately 128 expansions
            solvers.Add(new CBS(astar, epea, 5,
                bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                lookaheadMaxExpansions: 256)); // MACBS(5)/EPEA* + adoption immediately 256 expansions
            */


            //solvers.Add(new CBS(astar, epea, doShuffle: true)); // CBS + shuffle
            //solvers.Add(new CBS(astar, epea,
            //    bypassStrategy: CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
            //    heuristic: mddPruning)); // CBS + MDD Pruning heuristic + adoption
            //solvers.Add(new CBS(astar, epea,
            //    doShuffle: true,
            //    useMddHeuristic: true)); // CBS+MDD+shuffle
            //solvers.Add(new CBS(astar, epea, 5, justbreakForConflicts: true)); // MA-CBS(5) + Simply tie break for more conflicts
            //solvers.Add(new CBS(astar, epea, doMalte: true)); // CBS + Malte
            /*
            //solvers.Add(new CBS(epea, epea, -1));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, -1)); // Should be identical since no merging is done.
            //solvers.Add(new CBS(epea, epea, 0));
            solvers.Add(new CBS(astar, epea, 0));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 0));
            //solvers.Add(new CBS(epea, epea, 1));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 1));
            //solvers.Add(new CBS(epea, epea, 5));
            solvers.Add(new CBS(astar, epea, 5));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 5));
            //solvers.Add(new CBS(epea, epea, 10));
            solvers.Add(new CBS(astar, epea, 10));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 10));
            //solvers.Add(new CBS(epea, epea, 100));
            solvers.Add(new CBS(astar, epea, 100));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 100));
            //solvers.Add(new CBS(epea, epea, 500));
            //solvers.Add(new MACBS_WholeTreeThreshold(epea, epea, 500));
            
            //solvers.Add(new CBS(astar_with_od, astar_with_od, -1));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, -1)); // Should be identical since no merging is done.
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 0));
            solvers.Add(new CBS(astar, astar_with_od, 0));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 0));
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 1));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 1));
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 5));
            solvers.Add(new CBS(astar, astar_with_od, 5));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 5));
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 10));
            solvers.Add(new CBS(astar, astar_with_od, 10));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 10));
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 100));
            solvers.Add(new CBS(astar, astar_with_od, 100));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 100));
            //solvers.Add(new CBS(astar_with_od, astar_with_od, 500));
            //solvers.Add(new MACBS_WholeTreeThreshold(astar_with_od, astar_with_od, 500));
            */
            //solvers.Add(new A_Star(simple, mStar: true)); // rM*! Works
            //solvers.Add(new A_Star(simple, mStar: true, mStarShuffle: true)); // rM* shuffle! Works
            //solvers.Add(new A_Star(cbsHeuristic)); // A* with cbsHeuristic
            //solvers.Add(new A_Star_WithOD(simple));  // A* + OD
            //solvers.Add(new A_Star_WithOD(simple, mStar: true)); // rM*+OD!
            //solvers.Add(new A_Star_WithOD(simple, mStar: true, mStar: true)); // rM*+OD shuffle!
            //solvers.Add(new PEA_Star(simple)); // Works
            //solvers.Add(new PEA_Star(cbsHeuristic));
            //soldier: solvers.Add(new EPEA_Star(simple)); // Works.
            //solvers.Add(new EPEA_Star(simple, true, false)); // EPErM*
            //solvers.Add(new EPEA_Star(simple, true, true)); // EPErM* shuffle
            //soldier: solvers.Add(new CBS(astar, epea, 0)); // EPEA*+(S)ID

            //solvers.Add(new A_Star(cbsHeuristicSolve1));
            //solvers.Add(new A_Star(cbsHeuristicSolve2));
            //solvers.Add(new A_Star(cbsHeuristicSolve3));
            //solvers.Add(new A_Star(cbsHeuristicSolve4));
            //solvers.Add(new A_Star(cbsHeuristicSolve5));
            //solvers.Add(new A_Star(cbsHeuristicSolve6));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve1));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve2));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve3));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve4));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve5));
            //solvers.Add(new A_Star(cbsHeuristicNoSolve6));
            //solvers.Add(new A_Star(sicOrCbsh6));

            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve1));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve2));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve3));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve4));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve5));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicSolve6));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve1));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve2));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve3));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve4));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve5));
            //solvers.Add(new A_Star_WithOD(cbsHeuristicNoSolve6));
            //solvers.Add(new A_Star_WithOD(sicOrCbsh6));

            A_Star solver;
            // dynamic not rational lazy A*+OD/CBS/A*/SIC:
            //solver = new A_Star_WithOD(simple);
            //var dynamicLazyOpenList1 = new DynamicLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicLazyOpenList1;
            //solvers.Add(solver);

            // dynamic rational lazy A*+OD/CBS/A*/SIC:
            //solver = new A_Star_WithOD(simple);
            //var dynamicRationalLazyOpenList1 = new DynamicRationalLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicRationalLazyOpenList1;
            //solvers.Add(solver);

            // dynamic rational lazy MA-CBS-local-5/A*+OD/MA-CBS-local-5/EPEA*/SIC:
            //solver = new A_Star_WithOD(simple);
            //var dynamicRationalLazyOpenList3 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            //solver.openList = dynamicRationalLazyOpenList3;
            //solvers.Add(new CBS(astar, solver, 5));

            //solvers.Add(new EPEA_Star(cbsHeuristicSolve1));
            //solvers.Add(new EPEA_Star(cbsHeuristicSolve2));
            //solvers.Add(new EPEA_Star(cbsHeuristicSolve3));
            //solvers.Add(new EPEA_Star(cbsHeuristicSolve4));
            //solvers.Add(new EPEA_Star(cbsHeuristicSolve5));
            //solvers.Add(new EPEA_Star(cbsHeuristicSolve6));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve1));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve2));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve3));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve4));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve5));
            //solvers.Add(new EPEA_Star(cbsHeuristicNoSolve6));
            //solvers.Add(new EPEA_Star(sicOrCbsh6));

            // dynamic not rational lazy EPEA*/CBS/A*/SIC:
            //solver = new EPEA_Star(simple);
            //var dynamicLazyOpenList2 = new DynamicLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicLazyOpenList2;
            //solvers.Add(solver);

            // dynamic rational lazy EPEA*/CBS/A*/SIC:
            //solver = new EPEA_Star(simple);
            //var dynamicRationalLazyOpenList2 = new DynamicRationalLazyOpenList(solver, dynamicLazyCbsh, this);
            //solver.openList = dynamicRationalLazyOpenList2;
            //solvers.Add(solver);

            /*
             * soldiers:
            // MA-CBS-local-5 / dynamic rational lazy EPEA* / MA-CBS-local-50 / EPEA* / SIC:
            solver = new EPEA_Star(simple);
            var dynamicRationalLazyOpenList4 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal50EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList4;
            solvers.Add(new CBS(astar, solver, 5));

            // dynamic rational lazy EPEA* / MA-CBS-local-5 / EPEA* / SIC + (S)ID:
            solver = new EPEA_Star(simple);
            var dynamicRationalLazyOpenList6 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList6;
            solvers.Add(new CBS(astar, solver, 0));
             */
            
            /*
            //soldier: but can't handle 50 agents
            // dynamic rational lazy EPEA* / MA-CBS-local-5 / EPEA* / SIC:
            solver = new EPEA_Star(simple);
            var dynamicRationalLazyOpenList8 = new DynamicRationalLazyOpenList(solver, dynamicLazyMacbsLocal5EpeaH, this);
            solver.openList = dynamicRationalLazyOpenList8;
            solvers.Add(solver);
             */

            //solvers.Add(new CostTreeSearchSolverNoPruning());
            //solvers.Add(new CostTreeSearchSolverKMatch(2));
            //solvers.Add(new CostTreeSearchSolverOldMatching(2));
            //solvers.Add(new CostTreeSearchSolverRepeatedMatch(2));
            //solvers.Add(new CostTreeSearchSolverKMatch(3));
            //!@# USE ME solvers.Add(new CostTreeSearchSolverOldMatching(3)); // Use this parameter. Best according to paper. 3RE
            //solvers.Add(new CostTreeSearchSolverRepeatedMatch(3));

            //solvers.Add(new CostTreeSearchNoPruning());
            //solvers.Add(new CostTreeSearchKMatch(2));
            //solvers.Add(new CostTreeSearchOldMatching(2));
            //solvers.Add(new CostTreeSearchRepeatedMatch(2));
            //solvers.Add(new CostTreeSearchKMatch(3));
            //solvers.Add(new CostTreeSearchOldMatching(3));
            //solvers.Add(new CostTreeSearchRepeatedMatch(3));

            //solvers.Add(new IndependenceDetection(new EPEA_Star()));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new EPEA_Star(), 1, 1)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new EPEA_Star(), 5, 5)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new EPEA_Star(), 10, 10)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new EPEA_Star(), 100, 100)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new EPEA_Star(), 500, 500)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star())));

            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star(), 1, 1)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star(), 5, 5)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star(), 10, 10)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star(), 100, 100)));
            //solvers.Add(new IndependenceDetection(new MACBS_WholeTreeThreshold(new A_Star(), 500, 500)));

            //solvers.Add(new IndependenceDetection(new PEA_Star()));
            //solvers.Add(new IndependenceDetection(new EPEA_Star()));
            //solvers.Add(new IndependenceDetection(new A_Star()));
            //solvers.Add(new IndependenceDetection());

            //solvers.Add(new CBS_IDA(new A_Star())); // Don't run! Uses must conds

            //solvers.Add(new MACBS_WholeTreeThreshold(new A_Star())); // Works

            //solvers.Add(new CBS_NoDD(new A_Star()));
            //solvers.Add(new CBS_NoDDb3(new A_Star()));
            //solvers.Add(new MACBS_WholeTreeThreshold(new A_Star(), 1, 1)); // Run this!

            outOfTimeCounters = new int[solvers.Count];
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        /// <summary>
        /// Generates a problem instance, including a board, start and goal locations of desired number of agents
        /// and desired precentage of obstacles
        /// TODO: Refactor to use operators.
        /// </summary>
        /// <param name="gridSize"></param>
        /// <param name="agentsNum"></param>
        /// <param name="obstaclesNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateProblemInstance(int gridSize, int agentsNum, int obstaclesNum)
        {
            // Randomization based on timer is disabled for purposes of getting
            // reproducible experiments.
            //Random rand = new Random();
            Debug.WriteLine($"Generating instance with {agentsNum} agents, {obstaclesNum} obstacles of size {gridSize}");
            if (agentsNum + obstaclesNum + 1 > gridSize * gridSize)
                throw new Exception($"Not enough room for {agentsNum}, {obstaclesNum} and one empty space in a {gridSize}x{gridSize} map.");

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
                if (grid[x][y]) // Already an obstacle
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

            // Initialized here only for the IsValid() call. TODO: Think how this can be sidestepped elegantly.
            ProblemInstance problem = new ProblemInstance();
            problem.Init(aStart, grid);
            
            for (int j = 0; j < RANDOM_WALK_STEPS; j++)
            {
                for (int i = 0; i < agentsNum; i++)
                {
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = false; // We're going to move the goal somewhere else
                    while (true)
                    {
                        Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                        aStart[i].lastMove.Update(op);
                        if (problem.IsValid(aStart[i].lastMove) &&
                            !goals[aStart[i].lastMove.x][aStart[i].lastMove.y]) // this spot isn't another agent's goal
                            break;
                        else
                            aStart[i].lastMove.setOppositeMove(); // Rollback
                    }
                    goals[aStart[i].lastMove.x][aStart[i].lastMove.y] = true; // Claim agent's new goal
                }
            }

            // Zero the agents' timesteps
            foreach (AgentState agentStart in aStart) 
            {
                agentStart.lastMove.time = 0;
            }

            // TODO: There is some repetition here of previous instantiation of ProblemInstance. Think how to elegantly bypass this.
            problem = new ProblemInstance();
            problem.Init(aStart, grid);
            return problem;            
        }

        /// <summary>
        /// Generates a problem instance based on a DAO map file.
        /// TODO: Fix code dup with GenerateProblemInstance and Import later.
        /// </summary>
        /// <param name="mapFilePath"></param>
        /// <param name="agentsNum"></param>
        /// <returns></returns>
        public ProblemInstance GenerateDragonAgeProblemInstance(string mapFilePath, int agentsNum)
        {
            Debug.WriteLine($"Generating instance with {agentsNum} agents");
            using (TextReader input = new StreamReader(mapFilePath))
            {
                string[] lineParts;
                string line;

                line = input.ReadLine();
                Trace.Assert(line.StartsWith("type octile"));

                // Read grid dimensions
                line = input.ReadLine();
                lineParts = line.Split(' ');
                Trace.Assert(lineParts[0].StartsWith("height"));
                int maxX = int.Parse(lineParts[1]);
                line = input.ReadLine();
                lineParts = line.Split(' ');
                Trace.Assert(lineParts[0].StartsWith("width"));
                int maxY = int.Parse(lineParts[1]);
                line = input.ReadLine();
                Trace.Assert(line.StartsWith("map"));
                bool[][] grid = new bool[maxX][];
                char cell;
                for (int i = 0; i < maxX; i++)
                {
                    grid[i] = new bool[maxY];
                    line = input.ReadLine();
                    for (int j = 0; j < maxY; j++)
                    {
                        cell = line[j];
                        if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                            grid[i][j] = true;
                        else
                            grid[i][j] = false;
                    }
                }

                int x;
                int y;
                Agent[] agentGoals = new Agent[agentsNum];
                AgentState[] agentStates = new AgentState[agentsNum];
                bool[][] goals = new bool[maxX][];

                for (int i = 0; i < maxX; i++)
                    goals[i] = new bool[maxY];

                // Choose random valid unclaimed goal locations
                for (int i = 0; i < agentsNum; i++)
                {
                    x = rand.Next(maxX);
                    y = rand.Next(maxY);
                    if (goals[x][y] || grid[x][y])
                        i--;
                    else
                    {
                        goals[x][y] = true;
                        agentGoals[i] = new Agent(x, y, i);
                    }
                }

                // Select random start/goal locations for every agent by performing a random walk
                for (int i = 0; i < agentsNum; i++)
                {
                    agentStates[i] = new AgentState(agentGoals[i].Goal.x, agentGoals[i].Goal.y, agentGoals[i]);
                }

                ProblemInstance problem = new ProblemInstance();
                problem.parameters[ProblemInstance.GRID_NAME_KEY] = Path.GetFileNameWithoutExtension(mapFilePath);
                problem.Init(agentStates, grid);

                for (int j = 0; j < RANDOM_WALK_STEPS; j++)
                {
                    for (int i = 0; i < agentsNum; i++)
                    {
                        goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = false; // We're going to move the goal somewhere else.
                        // Move in a random legal direction:
                        while (true)
                        {
                            Move.Direction op = (Move.Direction)rand.Next(0, 5); // TODO: fixme
                            agentStates[i].lastMove.Update(op);
                            if (problem.IsValid(agentStates[i].lastMove) &&
                                !goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y]) // This spot isn't another agent's goal
                                break;
                            else
                                agentStates[i].lastMove.setOppositeMove(); // Rollback
                        }
                        goals[agentStates[i].lastMove.x][agentStates[i].lastMove.y] = true; // Claim agent's new goal
                    }
                }

                // Zero the agents' timesteps
                foreach (AgentState agentStart in agentStates)
                    agentStart.lastMove.time = 0;

                return problem;
            }
        }

        /// <summary>
        /// Solve given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to solve</param>
        /// <returns>Whether any solver succeeded in solving the instance</returns>
        public bool SolveGivenProblem(ProblemInstance instance)
        {
            //return; // add for generator
            // Preparing a list of agent indices (not agent nums) for the heuristics' Init() method
            List<uint> agentList = Enumerable.Range(0, instance.agents.Length).Select(x=> (uint)x).ToList(); // FIXME: Must the heuristics really receive a list of uints?
            
            // Solve using the different algorithms
            Console.WriteLine($"Solving {instance}");
            this.PrintProblemStatistics(instance);

            // Initializing all heuristics, whereever they're used
            for (int i = 0; i < astar_heuristics.Count; i++)
                astar_heuristics[i].Init(instance, agentList);

            int solutionCost = -1;
            int firstSolverToSolveIndex = -1;

            for (int i = 0; i < solvers.Count; i++)
            {
                if (outOfTimeCounters[i] < Constants.MAX_FAIL_COUNT) // After "MAX_FAIL_COUNT" consecutive failures of a given algorithm we stop running it.
                                                                     // Assuming problem difficulties are non-decreasing, if it consistently failed on several problems it won't suddenly succeed in solving the next problem.
                {
                    GC.Collect();
                    GC.WaitForPendingFinalizers();

                    //if (i != 2)
                    //    continue;
                    //if (i == 1)
                    //    ((A_Star)solvers[i]).debug = true;
                    //if (i == 4)
                    //    ((CBS)solvers[i]).debug = true;
                    //if (i == 4)
                    //    ((CBS)((IndependenceDetection)solvers[i]).groupSolver).debug = true;
                    if (solvers[i].GetType() == typeof(CBS) || solvers[i].GetType() == typeof(MACBS_WholeTreeThreshold))
                    {
                        if (((CBS)solvers[i]).mergeThreshold == 314159) // MAGIC NUMBER WHICH MAKES US ADJUST B according to map
                        {
                            string gridName = (string)instance.parameters[ProblemInstance.GRID_NAME_KEY];
                            if (gridName.StartsWith("den"))
                                ((CBS)solvers[i]).mergeThreshold = 10;
                            else if (gridName.StartsWith("brc") || gridName.StartsWith("ost"))
                                ((CBS)solvers[i]).mergeThreshold = 100;
                        }
                    }


                    if (
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(CBS)) ||
                        (solvers[i].GetType() == typeof(IndependenceDetection) &&
                         ((IndependenceDetection)solvers[i]).groupSolver.GetType() == typeof(MACBS_WholeTreeThreshold))
                       )
                    {
                        if (((CBS)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold == 314159) // MAGIC NUMBER SEE ABOVE
                        {
                            string gridName = (string)instance.parameters[ProblemInstance.GRID_NAME_KEY];
                            if (gridName.StartsWith("den"))
                                ((CBS)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold = 10;
                            else if (gridName.StartsWith("brc") || gridName.StartsWith("ost"))
                                ((CBS)((IndependenceDetection)solvers[i]).groupSolver).mergeThreshold = 100;
                        }
                    }

                    this.run(solvers[i], instance);

                    Console.WriteLine();

                    int solverSolutionCost = solvers[i].GetSolutionCost();

                    if (solverSolutionCost >= 0) // Solved successfully
                    {
                        Plan plan = solvers[i].GetPlan();
                        int planSize = plan.GetSize();
                        if (planSize < 200)
                            plan.PrintPlan();
                        else
                            Console.WriteLine($"Plan is too long to print ({planSize} steps).");
                        outOfTimeCounters[i] = 0;

                        // Validate solution:
                        if (solutionCost == -1) // This is the first time the problem is successfully solved
                        {
                            solutionCost = solverSolutionCost;
                            firstSolverToSolveIndex = i;

                            plan.Check(instance);
                        }
                        else // Problem solved before
                        {
                            plan.Check(instance);

                            Trace.Assert(solutionCost == solverSolutionCost,
                                $"{solvers[firstSolverToSolveIndex]} solution cost is different than that of {solvers[i]}"); // Assuming algs are supposed to find an optimal solution, this is an error.
                            //Trace.Assert(solvers[0].GetExpanded() == solvers[i].GetExpanded(), "Different Expanded");
                            //Trace.Assert(solvers[0].GetGenerated() == solvers[i].GetGenerated(), "Different Generated");
                            //Trace.Assert(solvers[0].GetSolutionDepth() == solvers[i].GetSolutionDepth(), "Depth Bug " + solvers[i]);
                        }

                        Console.WriteLine("+SUCCESS+ (:");
                    }
                    else
                    {
                        outOfTimeCounters[i]++;
                        Console.WriteLine("-FAILURE- ):");
                    }
                }
                else
                    PrintNullStatistics(solvers[i]);
                
                Console.WriteLine();
            }
            this.ContinueToNextLine();
            return solutionCost != -1;
        }

        /// <summary>
        /// Solve given instance with a list of algorithms 
        /// </summary>
        /// <param name="instance">The instance to solve</param>
        public void SolveGivenProblemIncrementally(ProblemInstance instance)
        {
            // Preparing a list of agent indices (not agent nums) for the heuristics' Init() method
            List<uint> agentList = Enumerable.Range(0, instance.agents.Length).Select(x => (uint)x).ToList(); // FIXME: Must the heuristics really receive a list of uints?

            CooperativeAStar cooperativeAStar = new CooperativeAStar();
            cooperativeAStar.Setup(instance, this);
            this.startTime = this.ElapsedMillisecondsTotal();
            double handlingStartTime = this.ElapsedMillisecondsTotal();
            double elapsedTime = 0;

            foreach (var agentIndex in Enumerable.Range(0, instance.agents.Length))
            {
                // Solve using the different algorithms
                Console.WriteLine($"Solving {instance} agent {agentIndex}");
                this.PrintProblemStatistics(instance);

                GC.Collect();
                GC.WaitForPendingFinalizers();

                this.startTime += this.ElapsedMillisecondsTotal() - handlingStartTime;
                bool solved = cooperativeAStar.AddOneAgent(agentIndex);
                elapsedTime = this.ElapsedMilliseconds();
                handlingStartTime = this.ElapsedMillisecondsTotal();
                if (solved)
                {
                    Console.WriteLine("Total cost: {0}", cooperativeAStar.GetSolutionCost());
                    Console.WriteLine("Solution depth: {0}", cooperativeAStar.GetSolutionDepth());
                }
                else
                {
                    Console.WriteLine("Failed to solve");
                    Console.WriteLine("Solution depth lower bound: {0}", cooperativeAStar.GetSolutionDepth());
                }
                Console.WriteLine();

                Console.WriteLine("Time In milliseconds: {0}", elapsedTime);

                this.PrintStatistics(instance, cooperativeAStar, elapsedTime + instance.shortestPathComputeTime);

                Console.WriteLine();

                int solverSolutionCost = cooperativeAStar.GetSolutionCost();

                if (solverSolutionCost >= 0) // Solved successfully
                {
                    Plan plan = cooperativeAStar.GetPlan();
                    int planSize = plan.GetSize();
                    if (planSize < 50)
                        plan.PrintPlan();
                    else
                        Console.WriteLine($"Plan is too long to print ({planSize} steps).");

                    Console.WriteLine("+SUCCESS+ (:");
                }
                else
                {
                    Console.WriteLine("-FAILURE- ):");
                    break;
                }

                Console.WriteLine();
                this.ContinueToNextLine();
            }
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
            Console.WriteLine($"-----------------{solver}-----------------");
            this.startTime = this.ElapsedMillisecondsTotal();
            solver.Setup(instance, this);
            solved = solver.Solve();
            double elapsedTime = this.ElapsedMilliseconds();
            if (solved)
            {
                Console.WriteLine("Total cost: {0}", solver.GetSolutionCost());
                Console.WriteLine("Makespan: {0}", solver.GetPlan().GetSize() - 1);
                Console.WriteLine("Solution depth: {0}", solver.GetSolutionDepth());
            }
            else
            {
                Console.WriteLine("Failed to solve");
                Console.WriteLine("Solution depth lower bound: {0}", solver.GetSolutionDepth());
            }
            Console.WriteLine();

            Console.WriteLine("Time In milliseconds: {0}", elapsedTime + instance.shortestPathComputeTime);
            // TODO: Allow solvers to claim they don't use this heuristic and don't add the time to
            //       compute it to their runtime.

            this.PrintStatistics(instance, solver, elapsedTime + instance.shortestPathComputeTime);
            // Solver clears itself when it finishes the search.
            solver.ClearStatistics();
        }

        /// <summary>
        /// Print the header of the results file
        /// </summary>
        public void PrintResultsFileHeader()
        {
            this.resultsWriter.Write("Grid Name");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Rows");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Grid Columns");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Agents");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Num Of Obstacles");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            this.resultsWriter.Write("Instance Id");
            this.resultsWriter.Write(Run.RESULTS_DELIMITER);

            for (int i = 0; i < solvers.Count; i++)
            {
                var solver = solvers[i];
                this.resultsWriter.Write(solver + " Success");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Runtime");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                this.resultsWriter.Write(solver + " Solution Cost");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                solver.OutputStatisticsHeader(this.resultsWriter);
                this.resultsWriter.Write(solver + " Solution Depth");
                this.resultsWriter.Write(Run.RESULTS_DELIMITER);

                //this.resultsWriter.Write(name + "Min Group / G&D");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Max depth");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
                //this.resultsWriter.Write(name + "Memory Used");
                //this.resultsWriter.Write(Run.RESULTS_DELIMITER);
            }
            this.ContinueToNextLine();
        }

        /// <summary>
        /// Print the solver statistics to the results file.
        /// </summary>
        /// <param name="instance">The problem instance that was solved. Not used!</param>
        /// <param name="solver">The solver that solved the problem instance</param>
        /// <param name="runtimeInMillis">The time it took the given solver to solve the given instance</param>
        private void PrintStatistics(ProblemInstance instance, ISolver solver, double runtimeInMillis)
        {
            // Success col:
            if (solver.GetSolutionCost() < 0)
                this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(Run.SUCCESS_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(runtimeInMillis + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write(solver.GetSolutionCost() + RESULTS_DELIMITER);
            // Algorithm specific cols:
            solver.OutputStatistics(this.resultsWriter);
            // Solution Depth col:
            this.resultsWriter.Write(solver.GetSolutionDepth() + RESULTS_DELIMITER);
            //this.resultsWriter.Flush();
        }

        private void PrintProblemStatistics(ProblemInstance instance)
        {
            // Grid Name col:
            if (instance.parameters.ContainsKey(ProblemInstance.GRID_NAME_KEY))
                this.resultsWriter.Write(instance.parameters[ProblemInstance.GRID_NAME_KEY] + RESULTS_DELIMITER);
            else
                this.resultsWriter.Write(RESULTS_DELIMITER);
            // Grid Rows col:
            this.resultsWriter.Write(instance.grid.Length + RESULTS_DELIMITER);
            // Grid Columns col:
            this.resultsWriter.Write(instance.grid[0].Length + RESULTS_DELIMITER);
            // Num Of Agents col:
            this.resultsWriter.Write(instance.agents.Length + RESULTS_DELIMITER);
            // Num Of Obstacles col:
            this.resultsWriter.Write(instance.numObstacles + RESULTS_DELIMITER);
            // Instance Id col:
            this.resultsWriter.Write(instance.instanceId + RESULTS_DELIMITER);
        }

        private void ContinueToNextLine()
        {
            this.resultsWriter.WriteLine();
            this.resultsWriter.Flush();
        }

        private void PrintNullStatistics(ISolver solver)
        {
            // Success col:
            this.resultsWriter.Write(Run.FAILURE_CODE + RESULTS_DELIMITER);
            // Runtime col:
            this.resultsWriter.Write(Constants.MAX_TIME + RESULTS_DELIMITER);
            // Solution Cost col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Algorithm specific cols:
            for (int i = 0; i < solver.NumStatsColumns; ++i)
                this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
            // Solution Depth col:
            this.resultsWriter.Write("irrelevant" + RESULTS_DELIMITER);
        }
        
        public void ResetOutOfTimeCounters()
        {
            for (int i = 0; i < outOfTimeCounters.Length; i++)
            {
                outOfTimeCounters[i] = 0;
            }
        }

        private Stopwatch watch;
        public double ElapsedMillisecondsTotal()
        {
            return this.watch.Elapsed.TotalMilliseconds;
        }

        public double ElapsedMilliseconds()
        {
            return ElapsedMillisecondsTotal() - this.startTime;
        }

        public void StartOracle()
        {
            this.watch.Stop();
            // NOTE: This allows the algorithm with the oracle to solve harder problems without timing out.
            // Care must be taken when comparing average runtimes of algorithms, to avoid the average
            // runtime of algorithms with an oracle appearing longer since they managed to solve
            // harder problems.
        }

        public void StopOracle()
        {
            this.watch.Start();
        }
    }
}
