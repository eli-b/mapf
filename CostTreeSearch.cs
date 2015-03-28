
namespace CPF_experiment
{
    ///// <summary>
    ///// Ignore. Implementation of ID for ICTS.
    ///// </summary>
    //class CostTreeSearch : IndependenceDetection
    //{
    //    public static string PARENT_GROUP1_KEY = "parentGroup1";
    //    public static string PARENT_GROUP2_KEY = "parentGroup2";


    //    public CostTreeSearch(HeuristicCalculator heuristic)
    //        : base(, new CostTreeSearchSolver(), heuristic) { }

    //    public CostTreeSearch(ISolver singleSolver, ISolver groupSolver, HeuristicCalculator heuristic)
    //        : base(singleSolver, groupSolver, heuristic) { }

    //    public override string GetName() { return "CostTreeSearch+ID "; }

    //    public override string ToString()
    //    {
    //        return GetName();
    //    }

    //    /// <summary>
    //    /// Join the conflicting groups into a single group
    //    /// </summary>
    //    /// <param name="conflict">An object that describes the conflict</param>
    //    /// <returns>The composite group of agents</returns>
    //    protected override AgentsGroup JoinGroups(Conflict conflict)
    //    {
    //        AgentsGroup answer = conflict.group1.Join(conflict.group2);
    //        // TODO: Currently storing the previous groups - this might lead to a memory problem when there are many agents
    //        // (if this happens then store only the costs)
    //        answer.instance.parameters[PARENT_GROUP1_KEY] = conflict.group1;
    //        answer.instance.parameters[PARENT_GROUP2_KEY] = conflict.group2;

    //        // Free memory of grandparents
    //        conflict.group1.instance.parameters.Remove(PARENT_GROUP1_KEY);
    //        conflict.group1.instance.parameters.Remove(PARENT_GROUP2_KEY);
    //        conflict.group2.instance.parameters.Remove(PARENT_GROUP1_KEY);
    //        conflict.group2.instance.parameters.Remove(PARENT_GROUP2_KEY);

    //        return answer;
    //    }

    //}
    //class CostTreeSearchOldMatching : CostTreeSearch
    //{
    //    int sycSize;
    //    public CostTreeSearchOldMatching(int sycSize, HeuristicCalculator heuristic)
    //        : base(new CostTreeSearchSolverOldMatching(sycSize), heuristic) { this.sycSize = sycSize; }

    //    public override string GetName() { return "ICTS " + sycSize + "E+ID "; }
    //}
    //class CostTreeSearchNoPruning : CostTreeSearch
    //{
    //    public CostTreeSearchNoPruning(HeuristicCalculator heuristic)
    //        : base(new CostTreeSearchSolverNoPruning(), heuristic) { }

    //    public override string GetName() { return "ICTS " + "+ID "; }

    //}

    //class CostTreeSearchKMatch : CostTreeSearch
    //{
    //    int maxGroupChecked;
    //    public CostTreeSearchKMatch(int maxGroupChecked, HeuristicCalculator heuristic)
    //        : base(new CostTreeSearchSolverKMatch(maxGroupChecked), heuristic) { this.maxGroupChecked = maxGroupChecked; }


    //    public override string GetName() { return "ICTS " + maxGroupChecked + "S+ID "; }

    //}

    //class CostTreeSearchRepatedMatch : CostTreeSearch
    //{
    //    int sycSize;
    //    public CostTreeSearchRepatedMatch(int sycSize, HeuristicCalculator heuristic)
    //        : base(new CostTreeSearchSolverRepeatedMatch(sycSize), heuristic) { this.sycSize = sycSize; }

    //    public override string GetName() { return "ICTS " + sycSize + "RE+ID "; }
    //}
}
